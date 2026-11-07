"""Jolie's online path planner flying the course on the Sky Beta contract - the same interface
as on the drone. No /fmu topics: the organizer's skybeta_flight_interface owns PX4.

    bash flyx_skybeta/run.sh jolie                 # FlyX / sim: interface + this + sim pilot
    python3 jolie_course/jolie_course.py           # on the drone (interface up; the pilot arms)
    --shadow: plan and log, publish no setpoints

Inputs : /visual_slam/tracking/odometry (cuVSLAM, ENU odom), /skybeta/state, optionally a depth
         image + its CameraInfo (--depth-topic; FlyX: /camera/depth/image_raw).
Outputs: /skybeta/setpoint/pose (ENU odom), ~/status (JSON), a JSONL journal.
It never arms, changes mode or lands: the safety pilot arms (the interface then enters Offboard)
and lands. mission.py's Offboard / arm / Land requests are therefore not sent (they are journalled
as pilot actions); its LAND state holds over the pad (or where a guard fired) and reports LAND,
which is the pilot's cue.
Frames: odometry is mapped ENU -> NED with the REP-103 map the flight interface itself uses for
setpoints ((x, y, z)_ned = (y, x, -z)_enu, yaw_ned = pi/2 - yaw_enu), so mission.py's NED logic
is unchanged; setpoints go back the same way.
"""
from __future__ import annotations

import argparse
from collections import deque
import json
import math
import os
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
from flyx.px4_link import POSE_FRAME_NED, odom_to_course, yaw_from_q_ned  # noqa: E402

WALL_SNAP_M = .25       # depth returns this close to a published wall are that wall (pose error,
                        # stereo noise): FlyX run 6 lost the tight passage past maze wall M1 to them
HIT_CONFIRM = 3         # vote threshold for a new obstacle cell: +2 per frame that hits it, -1 per
                        # frame, so it must be seen in 2 of ~3 frames (single noisy frames are ignored)
MAX_TURN_RATE = .5      # rad/s: depth frames taken while turning faster are dropped (pose error)

ARMED, DISARMED, NAV_OFFBOARD = 2, 1, 14
_ENU_TO_NED = np.array([[0.0, 1.0, 0.0], [1.0, 0.0, 0.0], [0.0, 0.0, -1.0]])   # its own inverse
_FRD_TO_FLU = np.diag([1.0, -1.0, -1.0])


def enu_to_ned(p):
    """(x, y, z)_ned = (y, x, -z)_enu; the same map converts NED back to ENU."""
    return (float(p[1]), float(p[0]), -float(p[2]))


def quat_wxyz_from_matrix(r):
    w = np.sqrt(max(0.0, 1 + r[0, 0] + r[1, 1] + r[2, 2])) / 2
    x = np.copysign(np.sqrt(max(0.0, 1 + r[0, 0] - r[1, 1] - r[2, 2])) / 2, r[2, 1] - r[1, 2])
    y = np.copysign(np.sqrt(max(0.0, 1 - r[0, 0] + r[1, 1] - r[2, 2])) / 2, r[0, 2] - r[2, 0])
    z = np.copysign(np.sqrt(max(0.0, 1 - r[0, 0] - r[1, 1] + r[2, 2])) / 2, r[1, 0] - r[0, 1])
    q = np.array([w, x, y, z])
    return tuple(q / np.linalg.norm(q))


def enu_flu_to_ned_frd(q_wxyz):
    """Orientation of the FLU body in ENU (cuVSLAM) -> FRD body in NED (what mission.py expects)."""
    from flyx.px4_link import rotation_wxyz
    r = _ENU_TO_NED @ rotation_wxyz(np.asarray(q_wxyz, float) / np.linalg.norm(q_wxyz)) @ _FRD_TO_FLU
    return quat_wxyz_from_matrix(r)


def parse_args(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--shadow", action="store_true", help="plan and log; publish no setpoints")
    ap.add_argument("--odometry-topic", default="/visual_slam/tracking/odometry")
    ap.add_argument("--depth-topic", default="", help="depth sensor_msgs/Image; '' = no sensing (published map only)")
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
    ap.add_argument("--output", type=Path, default=ROOT.parent / "jolie_course" / "runs"
                    / f"jolie-{time.strftime('%Y%m%d-%H%M%S')}")
    a, _ = ap.parse_known_args(argv)
    return a


def safe_carrot_path(cost, spec, xy, path, carrot_m):
    """[xy, setpoint]: the point carrot_m along the plan, pulled in until the straight line to
    it stays out of the keep-out. PX4 flies straight at the setpoint, so a far carrot round a
    bend cuts the corner into the wall (fake-PX4 runs: "escape" 18 cm from the net)."""
    from pathplan.grid_search import line_cost
    from pathplan.online import _point_along
    cell = lambda p: np.clip((np.asarray(p)[::-1] + spec.half_size) / spec.resolution - .5, 0, cost.shape[0] - 1)
    for d in (carrot_m, .35, .25, .15):
        target = _point_along(path, d)
        if np.isfinite(line_cost(cost, cell(xy), [cell(target)])[0]):
            return np.array([xy, target])
    return np.array([xy, _point_along(path, .1)])


def planner_process(conn, planner_name, altitude, camera_offset, pitch_down_deg, carrot_m):
    """The planner and depth projection, in their own process: both are pure-Python CPU work
    and, sharing the ROS process's GIL, stalled the PX4 callbacks for 0.4 s at a time (FlyX run
    4: "odometry stale (0.42s)" in TAKEOFF; reproduced on a loaded CPU with simulated depth)."""
    signal.signal(signal.SIGINT, signal.SIG_IGN)          # the parent handles Ctrl-C and lands
    # Lower CPU priority than the ROS process: FlyX run 6's control loop stalled 1.05 s when this
    # process got busy at the start of NAVIGATE, and PX4 failsafed on offboard_control_signal_lost.
    try:
        os.nice(10)
    except OSError:
        pass
    from pathplan.depth_projection import (CameraMount, Intrinsics, decode_depth_image, drop_misregistered,
                                           project_depth)
    from pathplan.ros_node import build_planner
    from pathplan.world import clearance
    planner, spec, env, prior = build_planner(planner_name, altitude)
    near_wall = (clearance(prior, spec) < WALL_SNAP_M) & ~prior
    mount = CameraMount(offset=tuple(camera_offset), pitch_down_deg=pitch_down_deg)
    votes = np.zeros(prior.shape, np.int8)
    quality = {"hits": 0, "hits_on_published_walls": 0, "hits_new_confirmed": 0, "hits_new_ignored": 0}
    conn.send(("ready", [float(v) for v in planner.checkpoints[-1].centre]))
    while True:
        msg = conn.recv()
        if msg is None:
            return
        xy, yaw, frame = msg
        t0 = time.perf_counter()
        if frame is not None:
            data, width, height, encoding, step, k, xyz, rotation = frame
            depth = decode_depth_image(data, width, height, encoding, step)
            cells = project_depth(depth, Intrinsics.from_k(k, width, height), xyz, rotation, spec,
                                  altitude=altitude, half_height=env.half_height + env.tracking_margin,
                                  mount=mount, expected=prior)
            raw_hit = cells[1]
            if len(raw_hit):                           # diagnostic: a wrong camera mount shows up here
                on_wall = (prior | near_wall)[raw_hit[:, 0], raw_hit[:, 1]]
                quality["hits"] += len(raw_hit)
                quality["hits_on_published_walls"] += int(on_wall.sum())
            free, hit, behind = drop_misregistered(*cells, prior, near_wall)
            votes = np.maximum(votes - 1, 0).astype(np.int8)
            if len(hit):
                votes[hit[:, 0], hit[:, 1]] = np.minimum(votes[hit[:, 0], hit[:, 1]] + 2, 6)
                known = planner.belief.occ[hit[:, 0], hit[:, 1]]
                ok = known | (votes[hit[:, 0], hit[:, 1]] >= HIT_CONFIRM)
                quality["hits_new_confirmed"] += int((ok & ~known).sum())
                quality["hits_new_ignored"] += int((~ok).sum())
                hit = hit[ok]
            planner.observe(free, hit, behind)
        cmd = planner.update(xy, yaw)
        if cmd.mode == "move" and cmd.path is not None and len(cmd.path) >= 2:
            cmd.path = safe_carrot_path(planner.belief.cost, spec, np.asarray(xy, float), cmd.path, carrot_m)
        conn.send((cmd, planner.events, planner.cleared, 1e3 * (time.perf_counter() - t0), quality))


class PlannerProcess:
    """Runs ``planner_process``; offers what Mission uses (update, events, cleared). Create it
    before rclpy.init: it forks, which is only safe before ROS threads exist."""

    def __init__(self, a):
        import multiprocessing as mp
        ctx = mp.get_context("fork")
        self.conn, child = ctx.Pipe()
        self.proc = ctx.Process(target=planner_process, daemon=True,
                                args=(child, a.planner, a.altitude, a.camera_offset, a.camera_pitch_down_deg,
                                      a.carrot))
        self.proc.start()
        ready, self.pad_xy = self.conn.recv()
        if ready != "ready":
            raise RuntimeError("planner process did not start")
        self.events, self.cleared, self.ms, self.depth_quality = [], [], 0.0, {}
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
            cmd, self.events, self.cleared, self.ms, self.depth_quality = self.conn.recv()
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


def print_summary(mission, planner, node, journal_path):
    """What happened, on the terminal: result, why, checkpoints, PX4 failsafes, stalls, depth."""
    last = mission.events[-1] if mission.events else {}
    lines = ["", "=" * 72, f"RESULT: {mission.state}" + (f" - {last.get('note')}" if last.get("note") else "")]
    lines.append("checkpoints cleared: " + (", ".join(c["name"] for c in planner.cleared) or "none")
                 + f"  ({len(planner.cleared)}/5)")
    lines.append("states: " + " -> ".join(e["to"] for e in mission.events))
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
                     f"hits on published walls (low = wrong camera mount); new obstacles {q['hits_new_confirmed']}")
    elif node.depth_stats["frames"] == 0:
        lines.append("depth: no frames used (check --depth-topic / camera_info)")
    if planner.events:
        lines.append("planner events (last 5):")
        lines += [f"  {e}" for e in planner.events[-5:]]
    lines += [f"journal: {journal_path}", "=" * 72]
    print("\n".join(lines), flush=True)


def main(argv=None):
    a = parse_args(argv)
    planner = PlannerProcess(a)          # first: it forks
    import rclpy
    from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
    from rclpy.clock import Clock, ClockType
    from rclpy.executors import MultiThreadedExecutor
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from geometry_msgs.msg import PoseStamped
    from nav_msgs.msg import Odometry
    from sensor_msgs.msg import CameraInfo, Image
    from std_msgs.msg import String

    # the pilot arms and the interface enters Offboard whenever they are ready: no ack timeouts
    mission = Mission(planner, MissionConfig(
        execute=not a.shadow, altitude=a.altitude, carrot_m=a.carrot, launch_xy=tuple(a.launch_xy),
        launch_heading=a.launch_heading, launch_body_agl=a.launch_body_agl, pad_xy=tuple(planner.pad_xy),
        ack_timeout_s=1e9))
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

    class SkybetaNode(Node):
        def __init__(self):
            super().__init__("jolie_course")
            self.tel = Telemetry(now=0.0)
            self.tel.pose_frame = POSE_FRAME_NED  # odometry is mapped to NED below
            self.poses = deque(maxlen=300)      # (ROS time, NED position, q): ~3 s of odometry
            self.poses_lock = threading.Lock()  # odometry thread appends, depth thread reads
            self.depth_stats = {"frames": 0, "matched": 0, "fallback": 0, "turning": 0, "max_odom_delay_s": 0.0}
            self.px4_modes, self.stalls = [], []
            self.intr = None
            # odometry/state, depth (slow: ~50-100 ms per frame), planning and control each get their
            # own group so a depth frame or a slow plan never delays pose updates or setpoints
            sensors, depth_group, planning, control = (MutuallyExclusiveCallbackGroup() for _ in range(4))
            # steady clock: a stalled, slow or missing /clock must not stop the setpoint stream
            steady = Clock(clock_type=ClockType.STEADY_TIME)
            self.create_timer(.2, self.on_plan, callback_group=planning, clock=steady)
            self.create_timer(.05, self.on_control, callback_group=control, clock=steady)
            self.status_pub = self.create_publisher(String, "~/status", 10)
            self.sp_pub = None if a.shadow else self.create_publisher(PoseStamped, "/skybeta/setpoint/pose", 10)
            self.create_subscription(Odometry, a.odometry_topic, self.on_odom, qos_profile_sensor_data,
                                     callback_group=sensors)
            self.create_subscription(String, "/skybeta/state", self.on_state, 10, callback_group=sensors)
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
            self.last_sky = {}
            self.get_logger().info(f"jolie_course up (Sky Beta contract, {'SHADOW' if a.shadow else 'FLYING'}); "
                                   f"odometry {a.odometry_topic}; journal {a.output}. "
                                   "The pilot arms; the interface enters Offboard.")

        def now(self):
            return self.get_clock().now().nanoseconds * 1e-9

        # ---------------------------------------------------------- inputs
        def on_odom(self, msg):
            t, tel = time.monotonic(), self.tel        # arrival time, monotonic (see Telemetry)
            p, o = msg.pose.pose.position, msg.pose.pose.orientation
            try:
                q = enu_flu_to_ned_frd((o.w, o.x, o.y, o.z))
            except ValueError:
                return
            tel.odom_t, tel.position_ned, tel.q_ned_frd = t, enu_to_ned((p.x, p.y, p.z)), q
            stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            with self.poses_lock:
                self.poses.append((stamp if stamp > 0 else self.now(), tel.position_ned, tel.q_ned_frd))

        def on_state(self, msg):
            try:
                st = json.loads(msg.data)
            except ValueError:
                return
            t, tel = time.monotonic(), self.tel
            armed, offboard = bool(st.get("armed")), bool(st.get("offboard_active"))
            nav = NAV_OFFBOARD if offboard else 0
            if nav != tel.nav_state or (ARMED if armed else DISARMED) != tel.arming_state:
                self.px4_modes.append(f"{st.get('nav_state_name')} armed={armed} offboard_active={offboard}")
                log({"kind": "px4_mode", "t": self.now(), "nav_state": st.get("nav_state"),
                     "nav_state_name": st.get("nav_state_name"), "armed": armed, "offboard_active": offboard})
            tel.status_t = tel.local_t = tel.landed_t = t
            tel.arming_state, tel.nav_state, tel.user_intention = (ARMED if armed else DISARMED), nav, nav
            tel.landed = not armed                      # the pilot lands and disarms
            tel.xy_valid = tel.z_valid = bool(st.get("px4_link")) and st.get("local_enu") is not None
            if st.get("clamped") and st.get("clamped") != self.last_sky.get("clamped"):
                log({"kind": "clamped", "t": self.now(), "clamped": st.get("clamped")})
            self.last_sky = st

        def on_info(self, msg):
            if self.intr is None:
                self.intr = list(msg.k)
                log({"kind": "camera_info", "k": list(msg.k), "width": msg.width, "height": msg.height,
                     "frame_id": msg.header.frame_id})

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
            self.depth_stats["fallback"] += 1           # unstamped / other clock: latest pose
            return mission.pose_course, 0.0

        def on_depth(self, msg):
            if mission.frame is None or self.intr is None or not mission.planning_allowed():
                return
            self.depth_stats["frames"] += 1
            if self.depth_stats["frames"] % 100 == 1:
                log({"kind": "depth", "t": self.now(), **self.depth_stats, "quality": planner.depth_quality})
            (xyz, _, rotation), rate = self.pose_at(msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9)
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
                gap = mono - self.last_tick
                if gap > .15:
                    self.stalls.append(gap)
                    log({"kind": "timing", "t": sim, "gap_s": round(gap, 3)})
            self.last_tick = mono
            self.tel.now, self.tel.sim_now = mono, None        # flight limit on the steady clock
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
            if out.command is not None:                        # Offboard / arm / Land: the pilot does these
                log({"kind": "pilot_action", "t": sim, "command": out.command.command, "state": out.state})
            if out.setpoint is None or self.sp_pub is None:
                return                                         # pose setpoints persist: the interface holds
            pos, yaw = out.setpoint
            msg = PoseStamped()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = "odom"
            msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = enu_to_ned(pos)   # same map back
            yaw_enu = math.pi / 2 - float(yaw)
            msg.pose.orientation.w, msg.pose.orientation.z = math.cos(yaw_enu / 2), math.sin(yaw_enu / 2)
            self.sp_pub.publish(msg)

    from rclpy.signals import SignalHandlerOptions
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)

    def interrupt(signum, frame):
        raise KeyboardInterrupt
    signal.signal(signal.SIGINT, interrupt)
    signal.signal(signal.SIGTERM, interrupt)
    node = SkybetaNode()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)

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
        if not spinner.is_alive():              # the interface holds the last pose; the pilot lands
            node.get_logger().error(f"executor stopped in {mission.state}: the pilot must land")
    except KeyboardInterrupt:
        pass                                    # pose setpoints persist: the interface holds; the pilot lands
    finally:
        log({"kind": "end", "state": mission.state, "events": mission.events, "depth": node.depth_stats,
             "depth_quality": planner.depth_quality,
             "cleared": planner.cleared, "planner_events": planner.events})
        print_summary(mission, planner, node, a.output / "journal.jsonl")
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
