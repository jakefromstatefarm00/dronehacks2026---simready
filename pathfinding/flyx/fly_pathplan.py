"""Fly the course in the FlyX lab with the pathplan online planner (native PX4 Offboard).

    cd <repo>/pathfinding
    python3 -m flyx.fly_pathplan                       # SHADOW: maps + plans, sends nothing to PX4
    python3 -m flyx.fly_pathplan --execute             # flies: Offboard, arm, takeoff, course, land

Run ``python3 -m flyx.check_lab`` first and pass the topics it reports (--depth-topic,
--camera-info-topic).  Shadow first; --execute only after the team approves the flight
and the simulator was Reset (see flyx/README.md).

Inputs : PX4 vehicle_odometry, vehicle_status, vehicle_land_detected, vehicle_local_position
         (/fmu/out/*, version suffix discovered), a depth image + its CameraInfo.
Outputs: shadow -> nothing to PX4; ~/status and a JSONL journal only.
         execute -> /fmu/in/offboard_control_mode + trajectory_setpoint (20 Hz), vehicle_command
         (Offboard, arm, land) exactly as mission.py decides.
"""
from __future__ import annotations

import argparse
from collections import deque
import json
import signal
from pathlib import Path
import sys
import threading
import time

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from flyx.mission import Mission, MissionConfig, Telemetry      # noqa: E402
from flyx.px4_link import NAN3, find_topic, odom_to_course      # noqa: E402

OUT_TOPICS = {                      # basename -> px4_msgs type (discovered with optional _vN suffix)
    "odom": ("vehicle_odometry", "VehicleOdometry"),
    "status": ("vehicle_status", "VehicleStatus"),
    "land": ("vehicle_land_detected", "VehicleLandDetected"),
    "local": ("vehicle_local_position", "VehicleLocalPosition"),
}


def parse_args(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--execute", action="store_true", help="actually fly (default: shadow, nothing sent to PX4)")
    ap.add_argument("--depth-topic", default="", help="depth sensor_msgs/Image (from check_lab); '' = no sensing")
    ap.add_argument("--camera-info-topic", default="", help="CameraInfo of the depth image (default: sibling camera_info)")
    ap.add_argument("--camera-offset", type=float, nargs=3, default=[.115, 0.0, .029],
                    help="depth camera origin in body FLU, m (Sky Beta CAD)")
    ap.add_argument("--camera-pitch-down-deg", type=float, default=0.0)
    ap.add_argument("--planner", default="theta", choices=["theta", "astar", "static"])
    ap.add_argument("--altitude", type=float, default=.9)
    ap.add_argument("--carrot", type=float, default=.5)
    ap.add_argument("--launch-xy", type=float, nargs=2, default=[-2.515, .914])
    ap.add_argument("--launch-heading", type=float, default=1.5707963, help="course-frame heading on the mark, rad")
    ap.add_argument("--launch-body-agl", type=float, default=.05)
    ap.add_argument("--output", type=Path, default=ROOT / "runs" / f"flyx-{time.strftime('%Y%m%d-%H%M%S')}")
    ap.add_argument("--wall-clock", action="store_true",
                    help="use the system clock instead of /clock (only for tests without a simulator)")
    return ap.parse_args(argv)


def planner_process(conn, planner_name, altitude, camera_offset, pitch_down_deg):
    """The planner and depth projection, in their own process: both are pure-Python CPU work
    and, sharing the ROS process's GIL, stalled the PX4 callbacks for 0.4 s at a time (FlyX run
    4: "odometry stale (0.42s)" in TAKEOFF; reproduced on a loaded CPU with simulated depth)."""
    signal.signal(signal.SIGINT, signal.SIG_IGN)          # the parent handles Ctrl-C and lands
    from pathplan.depth_projection import CameraMount, Intrinsics, decode_depth_image, project_depth
    from pathplan.ros_node import build_planner
    planner, spec, env, prior = build_planner(planner_name, altitude)
    mount = CameraMount(offset=tuple(camera_offset), pitch_down_deg=pitch_down_deg)
    conn.send("ready")
    while True:
        msg = conn.recv()
        if msg is None:
            return
        xy, yaw, frame = msg
        t0 = time.perf_counter()
        if frame is not None:
            data, width, height, encoding, step, k, xyz, rotation = frame
            depth = decode_depth_image(data, width, height, encoding, step)
            planner.observe(*project_depth(depth, Intrinsics.from_k(k, width, height), xyz, rotation, spec,
                                           altitude=altitude, half_height=env.half_height + env.tracking_margin,
                                           mount=mount, expected=prior))
        cmd = planner.update(xy, yaw)
        conn.send((cmd, planner.events, planner.cleared, 1e3 * (time.perf_counter() - t0)))


class PlannerProcess:
    """Runs ``planner_process``; offers what Mission uses (update, events, cleared). Create it
    before rclpy.init: it forks, which is only safe before ROS threads exist."""

    def __init__(self, a):
        import multiprocessing as mp
        ctx = mp.get_context("fork")
        self.conn, child = ctx.Pipe()
        self.proc = ctx.Process(target=planner_process, daemon=True,
                                args=(child, a.planner, a.altitude, a.camera_offset, a.camera_pitch_down_deg))
        self.proc.start()
        if self.conn.recv() != "ready":
            raise RuntimeError("planner process did not start")
        self.events, self.cleared, self.ms = [], [], 0.0
        self.frame, self.lock = None, threading.Lock()      # latest depth frame, from the depth callback

    def give_frame(self, frame):
        with self.lock:
            self.frame = frame

    def update(self, xy, yaw):
        """One planning step. Blocks only the calling thread; waiting on the pipe frees the GIL."""
        with self.lock:
            frame, self.frame = self.frame, None
        try:
            self.conn.send((xy, yaw, frame))
            cmd, self.events, self.cleared, self.ms = self.conn.recv()
            return cmd
        except (EOFError, OSError):
            from pathplan.online import Command
            self.events = self.events + [{"event": "planner process stopped"}]
            return Command("failed", reason="planner process stopped")

    def close(self):
        try:
            self.conn.send(None)
        except OSError:
            pass
        self.proc.join(2.0)
        if self.proc.is_alive():
            self.proc.terminate()


def main(argv=None):
    a = parse_args(argv)
    planner = PlannerProcess(a)          # first: it forks
    import rclpy
    from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
    from rclpy.executors import MultiThreadedExecutor
    from rclpy.node import Node
    from rclpy.parameter import Parameter
    from rclpy.qos import qos_profile_sensor_data
    import px4_msgs.msg as px4
    from sensor_msgs.msg import CameraInfo, Image
    from std_msgs.msg import String

    mission = Mission(planner, MissionConfig(
        execute=a.execute, altitude=a.altitude, carrot_m=a.carrot, launch_xy=tuple(a.launch_xy),
        launch_heading=a.launch_heading, launch_body_agl=a.launch_body_agl))
    a.output.mkdir(parents=True, exist_ok=False)
    journal = open(a.output / "journal.jsonl", "w", encoding="utf-8")

    log_lock = threading.Lock()

    def log(record):
        with log_lock:                        # callbacks may still fire while shutting down
            if journal.closed:
                return
            journal.write(json.dumps(record, default=float) + "\n")
            journal.flush()

    class FlyNode(Node):
        def __init__(self):
            super().__init__("flyx_pathplan")
            self.set_parameters([Parameter("use_sim_time", value=not a.wall_clock)])
            self.tel = Telemetry(now=0.0)
            self.poses = deque(maxlen=300)      # (ROS time, NED position, q): ~3 s of odometry
            self.depth_stats = {"frames": 0, "matched": 0, "fallback": 0}
            self.intr = None
            self.subs = {}
            # PX4 telemetry, depth (slow: ~50-100 ms per frame), planning and control each get their
            # own group so a depth frame or a slow plan never delays pose updates or setpoints
            sensors, depth_group, planning, control = (MutuallyExclusiveCallbackGroup() for _ in range(4))
            self.groups = sensors
            self.create_timer(1.0, self.discover, callback_group=sensors)
            self.create_timer(.2, self.on_plan, callback_group=planning)
            self.create_timer(.05, self.on_control, callback_group=control)
            self.status_pub = self.create_publisher(String, "~/status", 10)
            if a.execute:
                self.mode_pub = self.create_publisher(px4.OffboardControlMode, "/fmu/in/offboard_control_mode",
                                                      qos_profile_sensor_data)
                self.sp_pub = self.create_publisher(px4.TrajectorySetpoint, "/fmu/in/trajectory_setpoint",
                                                    qos_profile_sensor_data)
                self.cmd_pub = self.create_publisher(px4.VehicleCommand, "/fmu/in/vehicle_command",
                                                     qos_profile_sensor_data)
            if a.depth_topic:
                info = a.camera_info_topic or a.depth_topic.rsplit("/", 1)[0] + "/camera_info"
                self.create_subscription(Image, a.depth_topic, self.on_depth, qos_profile_sensor_data,
                                         callback_group=depth_group)
                self.create_subscription(CameraInfo, info, self.on_info, qos_profile_sensor_data,
                                         callback_group=depth_group)
            else:
                self.get_logger().warn("no --depth-topic: planning on the published map only")
            self.last_state = None
            self.last_tick = None
            self.get_logger().info(f"flyx_pathplan up: {'EXECUTE' if a.execute else 'SHADOW'}; journal {a.output}")

        def now(self):
            return self.get_clock().now().nanoseconds * 1e-9

        # ---------------------------------------------------------- inputs
        def discover(self):
            graph = self.get_topic_names_and_types()
            for key, (base, typ) in OUT_TOPICS.items():
                if key in self.subs:
                    continue
                topic = find_topic(graph, base, f"px4_msgs/msg/{typ}")
                if topic:
                    self.subs[key] = self.create_subscription(
                        getattr(px4, typ), topic, lambda msg, k=key: self.on_px4(k, msg), qos_profile_sensor_data,
                        callback_group=self.groups)
                    self.get_logger().info(f"{key}: {topic}")
                    log({"kind": "topic", "key": key, "topic": topic})

        def on_px4(self, key, msg):
            t, tel = time.monotonic(), self.tel        # arrival time, monotonic (see Telemetry)
            if key == "odom":
                tel.odom_t, tel.position_ned, tel.q_ned_frd = t, tuple(msg.position), tuple(msg.q)
                self.poses.append((self.now(), tel.position_ned, tel.q_ned_frd))
                tel.pose_frame = msg.pose_frame
            elif key == "status":
                tel.status_t, tel.arming_state, tel.nav_state = t, msg.arming_state, msg.nav_state
            elif key == "land":
                tel.landed_t, tel.landed = t, bool(msg.landed)
            elif key == "local":
                tel.local_t, tel.xy_valid, tel.z_valid = t, bool(msg.xy_valid), bool(msg.z_valid)

        def on_info(self, msg):
            if self.intr is None:
                self.intr = list(msg.k)
                log({"kind": "camera_info", "k": list(msg.k), "width": msg.width, "height": msg.height,
                     "frame_id": msg.header.frame_id})

        def pose_at(self, stamp):
            """Course pose when the image was taken (odometry nearest its header stamp, same ROS
            clock). Projecting with the latest pose instead smears walls sideways during turns."""
            if stamp > 0 and self.poses:
                t, pos, q = min(self.poses, key=lambda p: abs(p[0] - stamp))
                if abs(t - stamp) <= .05:
                    self.depth_stats["matched"] += 1
                    return odom_to_course(mission.frame, pos, q)
            self.depth_stats["fallback"] += 1           # unstamped / other clock: latest pose
            return mission.pose_course

        def on_depth(self, msg):
            if mission.pose_course is None or mission.frame is None or self.intr is None:
                return
            self.depth_stats["frames"] += 1
            if self.depth_stats["frames"] % 100 == 1:
                log({"kind": "depth", "t": self.now(), **self.depth_stats})
            xyz, _, rotation = self.pose_at(msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9)
            # the planner process takes the latest frame at its next step (<= 5 Hz)
            planner.give_frame((bytes(msg.data), msg.width, msg.height, msg.encoding, msg.step, self.intr,
                                xyz, rotation))

        # ---------------------------------------------------------- planning
        def on_plan(self):
            t0 = time.perf_counter()
            cmd = mission.plan()
            ms = 1e3 * (time.perf_counter() - t0)
            if cmd is not None:
                log({"kind": "plan", "t": self.now(), "mode": cmd.mode, "reason": cmd.reason, "ms": round(ms, 1),
                     "planner_ms": round(planner.ms, 1),
                     "target": None if mission.target is None else [list(mission.target[0]), mission.target[1]],
                     "cleared": [c["name"] for c in planner.cleared]})

        # ---------------------------------------------------------- control
        def on_control(self):
            mono, sim = time.monotonic(), self.now()
            if self.last_tick is not None:                     # control timer is 20 Hz
                gap_mono, gap_sim = mono - self.last_tick[0], sim - self.last_tick[1]
                if gap_mono > .15 or abs(gap_sim - gap_mono) > .15:
                    log({"kind": "timing", "t": sim, "gap_s": round(gap_mono, 3), "clock_gap_s": round(gap_sim, 3)})
            self.last_tick = (mono, sim)
            self.tel.now, self.tel.sim_now = mono, sim
            out = mission.tick(self.tel)
            if out.state != self.last_state:
                self.get_logger().info(f"state {self.last_state} -> {out.state} {out.note}")
                log({"kind": "state", "t": sim, "state": out.state, "note": out.note,
                     "events": mission.events[-1:]})
                self.last_state = out.state
            self.status_pub.publish(String(data=json.dumps({
                "state": out.state, "note": out.note, "planner": mission.planner_status,
                "cleared": [c["name"] for c in planner.cleared]})))
            if not a.execute:
                return
            us = int(self.get_clock().now().nanoseconds / 1000)
            if out.setpoint is not None:
                mode = px4.OffboardControlMode()
                mode.timestamp, mode.position = us, True
                self.mode_pub.publish(mode)
                pos, yaw = out.setpoint
                sp = px4.TrajectorySetpoint()
                sp.timestamp = us
                sp.position = [float(v) for v in pos]
                sp.velocity, sp.acceleration, sp.jerk = NAN3, NAN3, NAN3
                sp.yaw, sp.yawspeed = float(yaw), float("nan")
                self.sp_pub.publish(sp)
            if out.command is not None:
                cmd = px4.VehicleCommand()
                cmd.timestamp, cmd.command = us, out.command.command
                cmd.param1, cmd.param2 = float(out.command.param1), float(out.command.param2)
                cmd.target_system, cmd.target_component = 1, 1
                cmd.source_system, cmd.source_component, cmd.from_external = 1, 1, True
                self.cmd_pub.publish(cmd)
                log({"kind": "command", "t": sim, "command": out.command.command,
                     "param1": out.command.param1, "param2": out.command.param2, "state": out.state})

        def land_if_airborne(self):
            if a.execute and mission.state in ("WAIT_ARM", "TAKEOFF", "NAVIGATE"):
                cmd = px4.VehicleCommand()
                cmd.timestamp = int(self.get_clock().now().nanoseconds / 1000)
                cmd.command, cmd.target_system, cmd.target_component = 21, 1, 1
                cmd.source_system, cmd.source_component, cmd.from_external = 1, 1, True
                self.cmd_pub.publish(cmd)
                log({"kind": "command", "t": self.now(), "command": 21, "note": "shutdown while airborne"})

    # Ctrl-C must reach our handler while ROS still works, so Land can still be published
    # (rclpy's default handler shuts the context down first).
    from rclpy.signals import SignalHandlerOptions
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)

    def interrupt(signum, frame):              # Ctrl-C and SIGTERM (a lab Stop) both land
        raise KeyboardInterrupt
    signal.signal(signal.SIGINT, interrupt)
    signal.signal(signal.SIGTERM, interrupt)
    node = FlyNode()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    # the executor spins on its own thread; the main thread only waits for the end (or Ctrl-C)
    spinner = threading.Thread(target=executor.spin, daemon=True)
    spinner.start()
    try:
        while rclpy.ok() and spinner.is_alive() and mission.state not in ("DONE", "FAILED", "TAKEN_OVER"):
            time.sleep(.1)
    except KeyboardInterrupt:
        node.land_if_airborne()
        time.sleep(.3)                        # let the Land command go out before tearing down
    finally:
        log({"kind": "end", "state": mission.state, "events": mission.events, "depth": node.depth_stats,
             "cleared": planner.cleared, "planner_events": planner.events})
        print(f"finished in state {mission.state}; journal {a.output / 'journal.jsonl'}", flush=True)
        executor.shutdown(timeout_sec=1.0)
        planner.close()
        node.destroy_node()
        with log_lock:
            journal.close()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
