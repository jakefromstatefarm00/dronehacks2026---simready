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
import math
import queue
import signal
from pathlib import Path
import sys
import threading
import time

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from flyx.mission import Mission, MissionConfig, Telemetry      # noqa: E402
from flyx.px4_link import NAN3, find_topic, odom_to_course, yaw_from_q_ned  # noqa: E402
from pathplan.planner_process import MAX_TURN_RATE, PlannerProcess      # noqa: E402


OUT_TOPICS = {                      # basename -> px4_msgs type (discovered with optional _vN suffix)
    "odom": ("vehicle_odometry", "VehicleOdometry"),
    "status": ("vehicle_status", "VehicleStatus"),
    "land": ("vehicle_land_detected", "VehicleLandDetected"),
    "local": ("vehicle_local_position", "VehicleLocalPosition"),
    "failsafe_flags": ("failsafe_flags", "FailsafeFlags"),     # optional: the lab may not export it
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
    ap.add_argument("--turns", choices=["on", "off"], default="on",
                    help="speed-gated sharper turning when slow (v5 default; off = v4c's 0.25 rad/s cap)")
    ap.add_argument("--yaw-speed-max", type=float, default=.10, help="turns: bound on yaw rate x speed, rad m/s^2")
    ap.add_argument("--yaw-rate-slow-max", type=float, default=.50, help="turns: yaw-rate ceiling when slow, rad/s")
    ap.add_argument("--yaw-rate-in-place-max", type=float, default=.35, help="turns: yaw-rate ceiling turning in place")
    ap.add_argument("--explore-look-deg", type=float, default=0.0,
                    help="turns: one-off look this far past the route heading at slow corners (0 = off)")
    ap.add_argument("--wall-clock", action="store_true",
                    help="use the system clock instead of /clock (only for tests without a simulator)")
    return ap.parse_args(argv)



def turns_config(a):
    """MissionConfig fields for --turns (empty for off: the v3 defaults)."""
    if a.turns != "on":
        return {}
    return dict(turn_gate=True, yaw_speed_max=a.yaw_speed_max, yaw_rate_slow_max=a.yaw_rate_slow_max,
                yaw_rate_in_place_max=a.yaw_rate_in_place_max, explore_look_deg=a.explore_look_deg)


def planner_options(a):
    """Planner-child options: None = planner_process.V5_OPTIONS; --turns on adds the view / wall report
    the mission's turn gate needs (the other [v5] switches keep their defaults)."""
    if a.turns != "on":
        return None
    return {"view_m": True}


def print_summary(mission, planner, node, journal_path):
    """What happened, on the terminal: result, why, checkpoints, PX4 failsafes, stalls, depth."""
    last = mission.events[-1] if mission.events else {}
    lines = ["", "=" * 72, f"RESULT: {mission.state}" + (f" - {last.get('note')}" if last.get("note") else "")]
    lines.append("checkpoints cleared: " + (", ".join(c["name"] for c in planner.cleared) or "none")
                 + f"  ({len(planner.cleared)}/5)")
    lines.append("states: " + " -> ".join(e["to"] for e in mission.events))
    if mission.cfg.turn_gate:
        ts = mission.turn_stats
        lines.append(f"turns: on (yaw x speed <= {mission.cfg.yaw_speed_max}); highest cap {ts['cap_max']:.2f} rad/s, "
                     f"highest commanded yaw x speed {ts['yv_cmd_max']:.3f}, exploratory looks {ts['look']}")
    if node.px4_modes:
        lines.append("PX4 mode changes (nav_state / failsafe / flags):")
        lines += [f"  {m}" for m in node.px4_modes[-4:]]
    if node.stalls:
        lines.append(f"program stalls > 0.15 s: {len(node.stalls)}, worst {max(node.stalls):.2f} s")
    if mission.px4_failsafe_waits:
        lines.append(f"PX4 failsafe blips ridden out: {mission.px4_failsafe_waits * .05:.1f} s in total")
    q = planner.depth_quality or {}
    if q.get("hits"):
        lines.append(f"depth: {node.depth_stats['frames']} frames; {q['hits_on_published_walls'] / q['hits']:.0%} of "
                     f"hits on published walls (low = drift or wrong camera mount); new obstacles {q['hits_new_confirmed']}")
        if q.get("drift_filter_frames"):
            lines.append(f"drift-aware phantom filter on for {q['drift_filter_frames']} depth frames (pose had drifted)")
    elif node.depth_stats["frames"] == 0:
        lines.append("depth: no frames used (check --depth-topic / camera_info)")
    if planner.events:
        lines.append("planner events (last 5):")
        lines += [f"  {e}" for e in planner.events[-5:]]
    lines += [f"journal: {journal_path}", "=" * 72]
    print("\n".join(lines), flush=True)


def main(argv=None):
    a = parse_args(argv)
    from pathplan.ros_node import build_planner
    planner = PlannerProcess(build_planner, (a.planner, a.altitude, a.camera_offset, a.camera_pitch_down_deg,
                                             a.carrot, planner_options(a)))     # first: it forks
    import rclpy
    from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
    from rclpy.clock import Clock, ClockType
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.node import Node
    from rclpy.parameter import Parameter
    from rclpy.qos import qos_profile_sensor_data
    import px4_msgs.msg as px4
    from sensor_msgs.msg import CameraInfo, Image
    from std_msgs.msg import String

    mission = Mission(planner, MissionConfig(
        execute=a.execute, altitude=a.altitude, carrot_m=a.carrot, launch_xy=tuple(a.launch_xy),
        launch_heading=a.launch_heading, launch_body_agl=a.launch_body_agl, pad_xy=tuple(planner.pad_xy),
        **turns_config(a)))
    a.output.mkdir(parents=True, exist_ok=False)
    journal = open(a.output / "journal.jsonl", "w", encoding="utf-8")

    # The journal is written by its own thread: a slow disk must never hold up the setpoint loop.
    log_queue, log_closed = queue.Queue(), threading.Event()

    def journal_writer():
        while True:
            line = log_queue.get()
            if line is None:
                return
            journal.write(line)
            journal.flush()

    writer = threading.Thread(target=journal_writer, daemon=True)
    writer.start()

    def log(record):
        if not log_closed.is_set():           # callbacks may still fire while shutting down
            log_queue.put(json.dumps(record, default=float) + "\n")

    class FlyNode(Node):
        def __init__(self):
            super().__init__("flyx_pathplan")
            self.set_parameters([Parameter("use_sim_time", value=not a.wall_clock)])
            self.tel = Telemetry(now=0.0)
            self.poses = deque(maxlen=300)      # (ROS time, NED position, q): ~3 s of odometry
            self.poses_lock = threading.Lock()  # odometry thread appends, depth thread reads
            self.depth_stats = {"frames": 0, "matched": 0, "fallback": 0, "unmatched": 0, "turning": 0, "max_odom_delay_s": 0.0}
            self.failsafe_flags = None
            self.px4_modes, self.stalls = [], []
            self.offsets = deque(maxlen=200)    # arrival (ROS clock) - PX4 sample time, last ~2 s
            self.intr = None
            self.subs = {}
            # PX4 telemetry, depth (slow: ~50-100 ms per frame), planning and control each get their
            # own group so a depth frame or a slow plan never delays pose updates or setpoints
            sensors, depth_group, planning, control = (MutuallyExclusiveCallbackGroup() for _ in range(4))
            self.groups = sensors
            self.create_timer(1.0, self.discover, callback_group=sensors)
            # steady clock: a stalled or slow /clock must not stop the setpoint stream (PX4 drops
            # out of Offboard after COM_OF_LOSS_T without it)
            steady = Clock(clock_type=ClockType.STEADY_TIME)
            # planning (blocks on the planner process for up to ~1 s) runs on its own thread, see main()
            self.create_timer(.05, self.on_control, callback_group=control, clock=steady)
            self.px4_ref = None                 # (latest odometry timestamp us, monotonic arrival)
            self.status_pub = self.create_publisher(String, "~/status", 10)
            if a.execute:
                self.mode_pub = self.create_publisher(px4.OffboardControlMode, "/fmu/in/offboard_control_mode",
                                                      qos_profile_sensor_data)
                self.sp_pub = self.create_publisher(px4.TrajectorySetpoint, "/fmu/in/trajectory_setpoint",
                                                    qos_profile_sensor_data)
                self.cmd_pub = self.create_publisher(px4.VehicleCommand, "/fmu/in/vehicle_command",
                                                     qos_profile_sensor_data)
            from rclpy.qos import DurabilityPolicy, QoSProfile
            from tf2_msgs.msg import TFMessage
            self.create_subscription(TFMessage, "/tf_static", self.on_tf_static,
                                     QoSProfile(depth=100, durability=DurabilityPolicy.TRANSIENT_LOCAL),
                                     callback_group=sensors)
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
            self.check_foreign()
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

        def check_foreign(self):
            """Another program sending PX4 setpoints (a teammate's script still running, a flight
            interface): never arm with one; if one appears in flight, the guard lands."""
            own = 1 if a.execute else 0
            found = [f"{t} x{n}" for t in ("/fmu/in/trajectory_setpoint", "/fmu/in/offboard_control_mode")
                     for n in [self.count_publishers(t) - own] if n > 0]
            foreign = "; ".join(found)
            if foreign != self.tel.foreign:
                (self.get_logger().error if foreign else self.get_logger().info)(
                    "other PX4 setpoint publishers: " + foreign if foreign else "no other PX4 setpoint publisher")
                log({"kind": "foreign_publishers", "t": self.now(), "note": foreign})
            self.tel.foreign = foreign

        def on_px4(self, key, msg):
            t, tel = time.monotonic(), self.tel        # arrival time, monotonic (see Telemetry)
            if key == "odom":
                tel.odom_t, tel.position_ned, tel.q_ned_frd = t, tuple(msg.position), tuple(msg.q)
                self.px4_ref = (int(msg.timestamp), t)
                with self.poses_lock:
                    self.poses.append((self.sample_time(msg), tel.position_ned, tel.q_ned_frd))
                tel.pose_frame = msg.pose_frame
            elif key == "status":
                if msg.nav_state != tel.nav_state or bool(msg.failsafe) != tel.failsafe:
                    self.px4_modes.append(f"nav_state {int(msg.nav_state)} (asked {int(msg.nav_state_user_intention)})"
                                          f" failsafe={bool(msg.failsafe)} {self.failsafe_flags or ''}")
                    log({"kind": "px4_mode", "t": self.now(), "nav_state": int(msg.nav_state),
                         "requested": int(msg.nav_state_user_intention), "failsafe": bool(msg.failsafe),
                         "user_took_over": bool(msg.failsafe_and_user_took_over),
                         "arming_state": int(msg.arming_state), "failsafe_flags": self.failsafe_flags})
                tel.status_t, tel.arming_state, tel.nav_state = t, msg.arming_state, msg.nav_state
                tel.failsafe, tel.user_intention = bool(msg.failsafe), int(msg.nav_state_user_intention)
            elif key == "land":
                tel.landed_t, tel.landed = t, bool(msg.landed)
            elif key == "local":
                tel.local_t, tel.xy_valid, tel.z_valid = t, bool(msg.xy_valid), bool(msg.z_valid)
            elif key == "failsafe_flags":            # which failsafe condition is set (if exported)
                self.failsafe_flags = [f for f in msg.get_fields_and_field_types()
                                       if f != "timestamp" and getattr(msg, f) is True]

        def on_tf_static(self, msg):
            """Log the sim's static transforms once (camera mount vs our --camera-offset)."""
            log({"kind": "tf_static", "transforms": [
                {"parent": tr.header.frame_id, "child": tr.child_frame_id,
                 "xyz": [tr.transform.translation.x, tr.transform.translation.y, tr.transform.translation.z],
                 "xyzw": [tr.transform.rotation.x, tr.transform.rotation.y, tr.transform.rotation.z,
                          tr.transform.rotation.w]} for tr in msg.transforms]})

        def on_info(self, msg):
            if self.intr is None:
                self.intr = list(msg.k)
                log({"kind": "camera_info", "k": list(msg.k), "width": msg.width, "height": msg.height,
                     "frame_id": msg.header.frame_id})

        def sample_time(self, msg):
            """When the odometry was measured, on the ROS clock. Its arrival time can lag by the
            time it sat in our queue (100s of ms on a loaded CPU), which misplaces depth during
            turns. PX4's own sample time is on PX4's clock: map it with the smallest
            arrival - sample offset of the last ~2 s (the least-delayed message)."""
            px4_t = (msg.timestamp_sample or msg.timestamp) * 1e-6
            arrival = self.now()
            self.offsets.append(arrival - px4_t)
            t = px4_t + min(self.offsets)
            self.depth_stats["max_odom_delay_s"] = max(self.depth_stats["max_odom_delay_s"], round(arrival - t, 3))
            return t

        def pose_at(self, stamp):
            """(course pose when the image was taken, yaw rate then): odometry nearest its header
            stamp, same ROS clock. Projecting with the latest pose smears walls during turns."""
            with self.poses_lock:
                poses = list(self.poses)
            if stamp > 0 and poses:
                t, pos, q = min(poses, key=lambda p: abs(p[0] - stamp))
                if abs(t - stamp) <= .05:
                    self.depth_stats["matched"] += 1
                    t0, _, q0 = min(poses, key=lambda p: abs(p[0] - (stamp - .15)))
                    dyaw = yaw_from_q_ned(q) - yaw_from_q_ned(q0)
                    rate = abs(math.atan2(math.sin(dyaw), math.cos(dyaw))) / max(t - t0, .05)
                    return odom_to_course(mission.frame, pos, q), rate
            if self.depth_stats["matched"] >= 20:      # stamps do match normally: this frame fell in a
                self.depth_stats["unmatched"] += 1      # gap in the pose history (a stall) - skip it
                return None, 0.0
            self.depth_stats["fallback"] += 1           # unstamped / other clock: latest pose
            return mission.pose_course, 0.0

        def on_depth(self, msg):
            if mission.frame is None or self.intr is None or not mission.planning_allowed():
                return
            self.depth_stats["frames"] += 1
            if self.depth_stats["frames"] % 100 == 1:
                log({"kind": "depth", "t": self.now(), **self.depth_stats, "quality": planner.depth_quality})
            pose, rate = self.pose_at(msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9)
            if pose is None:
                return
            xyz, _, rotation = pose
            if rate > MAX_TURN_RATE:
                self.depth_stats["turning"] += 1
                return
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
                if gap_mono > .15:
                    self.stalls.append(gap_mono)
                if gap_mono > .15 or abs(gap_sim - gap_mono) > .15:
                    log({"kind": "timing", "t": sim, "gap_s": round(gap_mono, 3), "clock_gap_s": round(gap_sim, 3)})
            self.last_tick = (mono, sim)
            self.tel.now, self.tel.sim_now = mono, sim
            out = mission.tick(self.tel)
            if out.state != self.last_state:
                note = mission.events[-1]["note"] if mission.events and mission.events[-1]["to"] == out.state else ""
                self.get_logger().info(f"state {self.last_state} -> {out.state} {note or out.note}")
                log({"kind": "state", "t": sim, "state": out.state, "note": out.note,
                     "events": mission.events[-1:]})
                self.last_state = out.state
            self.status_pub.publish(String(data=json.dumps({
                "state": out.state, "note": out.note, "planner": mission.planner_status,
                "cleared": [c["name"] for c in planner.cleared]})))
            if not a.execute:
                return
            us = self.px4_us()
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

        def px4_us(self):
            """A timestamp in PX4's own time base (what its odometry carries), 20 ms in the past
            so it can never read as future. Sim-time stamps run slow against PX4's clock when
            the sim is slower than real time, and PX4 then sees our Offboard stream as stale."""
            if self.px4_ref is None:
                return int(self.get_clock().now().nanoseconds / 1000)
            ts, arrival = self.px4_ref
            return max(0, int(ts + (time.monotonic() - arrival) * 1e6) - 20_000)

        def land_if_airborne(self):
            if a.execute and mission.state in ("WAIT_ARM", "TAKEOFF", "NAVIGATE"):
                cmd = px4.VehicleCommand()
                cmd.timestamp = self.px4_us()
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
    # Single-threaded executor: every ROS callback is short. The MultiThreadedExecutor burnt ~73 % of
    # a core on this message load (single: ~13 %), and a CPU-hungry process is the one a crowded lab
    # machine starves (FlyX run 7: 86 stalls up to 1.95 s, even before arming).
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    planning_stop = threading.Event()

    def planning_loop():                        # 5 Hz, outside the executor
        while not planning_stop.is_set():
            t0 = time.monotonic()
            try:
                node.on_plan()
            except Exception:
                import traceback
                log({"kind": "error", "where": "planning", "error": traceback.format_exc()})
            planning_stop.wait(max(0.0, .2 - (time.monotonic() - t0)))

    planning_thread = threading.Thread(target=planning_loop, daemon=True)
    planning_thread.start()
    # the executor spins on its own thread; the main thread only waits for the end (or Ctrl-C)
    def spin():
        try:
            executor.spin()
        except Exception:                       # a callback raised: rclpy stops the executor
            import traceback
            log({"kind": "error", "where": "callback", "error": traceback.format_exc()})
            print(traceback.format_exc(), file=sys.stderr, flush=True)

    spinner = threading.Thread(target=spin, daemon=True)
    spinner.start()
    try:
        while rclpy.ok() and spinner.is_alive() and mission.state not in ("DONE", "FAILED", "TAKEN_OVER"):
            time.sleep(.1)
        if not spinner.is_alive():              # never leave the vehicle airborne without us
            node.get_logger().error(f"executor stopped in {mission.state}; landing if airborne")
            node.land_if_airborne()
            time.sleep(.3)
    except KeyboardInterrupt:
        node.land_if_airborne()
        time.sleep(.3)                        # let the Land command go out before tearing down
    finally:
        log({"kind": "end", "state": mission.state, "events": mission.events, "depth": node.depth_stats,
             "depth_quality": planner.depth_quality,
             "cleared": planner.cleared, "planner_events": planner.events})
        print_summary(mission, planner, node, a.output / "journal.jsonl")
        planning_stop.set()
        planning_thread.join(timeout=3.0)       # finish the step in flight before closing its pipe
        executor.shutdown(timeout_sec=1.0)
        planner.close()
        node.destroy_node()
        log_closed.set()
        log_queue.put(None)
        writer.join(timeout=3.0)
        journal.close()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
