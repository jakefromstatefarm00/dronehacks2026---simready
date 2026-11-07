"""FlyX lab: a square flight, native ROS 2 / PX4.  Run:  python3 flyx_square/fly_square.py

Self-contained: only rclpy, px4_msgs and std_msgs. Like the lab's takeoff/hover/land
example, but after the climb it flies one square, then lands where it took off:

    climb HEIGHT_M -> forward SIDE_M -> up SIDE_M -> back SIDE_M -> down SIDE_M -> land

The square is in the VERTICAL plane straight up the first corridor. A flat square does not
fit at the launch mark: the corridor there is ~1 m wide, and with Sky Beta's 0.28 m radius
only the centre line keeps >= 0.4 m from the walls (published-map clearance). Up the
corridor centre line the clearance is 0.5 m for the first ~1.2 m. Defaults (0.9 m climb,
0.6 m side) keep the top at 1.5 m (walls 1.73 m) and the far edge 0.6 m up the corridor.

Start ON THE LAUNCH MARK facing up the first corridor, after Reset world.
Everything is in PX4's own local frame (NED) relative to the take-off pose, so no
course-frame alignment is needed; "forward" is the heading PX4 reports at take-off.

Sequence (the FlyX-tested flow of pathfinding/flyx/mission.py):
    WAIT_HEALTH  fresh odometry, status, local position xy/z valid, landed + disarmed, SETTLE_S (10 s)
    STREAM       hold setpoints for 1 s (PX4 needs a stream before Offboard)
    OFFBOARD -> ARM -> TAKEOFF -> legs of the square (setpoint moves at SPEED_MPS) -> LAND -> DONE
Message pauses: the lab's PX4 -> ROS stream pauses for up to ~1 s at a time (FlyX runs: 0.69 s,
1.1 s). PX4 keeps estimating and flying through them, so a pause in odometry (> ODOM_HOLD_S) or in
local position HOLDS: the last setpoint keeps streaming (PX4 holds position on its own EKF) and the
square's progress is frozen until messages return. Land only on a real outage: odometry or local
position missing > OUTAGE_S, status > 3 s, estimator invalid, a corner not reached in
LEG_TIMEOUT_S of flying time, or total flight over MAX_FLIGHT_S.
If PX4 leaves Offboard by itself (failsafe, RC) it stops commanding (TAKEN_OVER).
Ctrl-C / SIGTERM (lab Stop) while airborne -> Land.

    python3 flyx_square/fly_square.py             # flies
    python3 flyx_square/fly_square.py --shadow    # does everything except publish to PX4
Journal: flyx_square/runs/square-<time>/journal.jsonl (else ~/flyx-runs or /tmp/flyx-runs).
"""
HEIGHT_M = 0.9       # climb above the take-off point (course flight altitude; walls are 1.73 m)
SIDE_M = 0.6         # side of the square (forward, then up); see the clearance note above
SPEED_MPS = 0.3      # how fast the setpoint moves along each edge
HOLD_S = 1.0         # pause at each corner
TOLERANCE_M = 0.12   # corner reached when PX4's position is this close (3-D)
LEG_TIMEOUT_S = 20.0
MAX_FLIGHT_S = 120.0
ODOM_HOLD_S = 0.4    # odometry this old: hold (freeze the square) until it returns
OUTAGE_S = 3.0       # odometry / local position missing this long: a real outage -> land
SETTLE_S = 10.0     # healthy + landed this long before Offboard/arm. 2 s (pathfinding/flyx) was too
                     # short in our Gazebo run v122 (OpenVINS lost all features at liftoff 4 s after the
                     # rig set the drone down); v123 with a longer settle flew the square cleanly.

import argparse
import json
from math import atan2, cos, isfinite, nan, sin
import os
from pathlib import Path
import signal
import sys
import threading
import time

CMD_DO_SET_MODE, CMD_ARM_DISARM, CMD_NAV_LAND = 176, 400, 21
ARMED, DISARMED = 2, 1
NAV_OFFBOARD = 14
POSE_FRAME_NED = 1
NAN3 = [nan, nan, nan]
OUT_TOPICS = {"odom": ("vehicle_odometry", "VehicleOdometry"), "status": ("vehicle_status", "VehicleStatus"),
              "land": ("vehicle_land_detected", "VehicleLandDetected"),
              "local": ("vehicle_local_position", "VehicleLocalPosition")}


def yaw_from_q_ned(q):
    w, x, y, z = q
    return atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))


def dist(a, b):
    return sum((p - q) ** 2 for p, q in zip(a, b)) ** .5


def find_topic(graph, basename, type_name):
    import re
    rx = re.compile(f"/fmu/out/{re.escape(basename)}(_v[0-9]+)?")
    hits = [t for t, types in graph if rx.fullmatch(t) and type_name in types]
    return hits[0] if len(hits) == 1 else None


class Telemetry:
    def __init__(self):
        self.now = 0.0          # monotonic wall clock
        self.sim_now = None     # simulation clock (None: not available, e.g. tests on wall clock)
        self.odom_t = self.status_t = self.landed_t = self.local_t = None
        self.odom_sim = None    # sim time at the last odometry arrival
        self.position = (0.0, 0.0, 0.0)
        self.q = (1.0, 0.0, 0.0, 0.0)
        self.pose_frame = POSE_FRAME_NED
        self.arming_state, self.nav_state = DISARMED, 0
        self.landed = False
        self.xy_valid = self.z_valid = False


class SquareMission:
    """Pure state machine: tick(telemetry) -> (state, setpoint (pos NED, yaw) or None, command or None, note)."""

    def __init__(self, height=HEIGHT_M, side=SIDE_M, speed=SPEED_MPS, hold=HOLD_S, tol=TOLERANCE_M,
                 leg_timeout=LEG_TIMEOUT_S, max_flight=MAX_FLIGHT_S, settle=SETTLE_S):
        self.height, self.side, self.speed, self.hold, self.tol = height, side, speed, hold, tol
        self.leg_timeout, self.max_flight, self.settle = leg_timeout, max_flight, settle
        self.state, self.events = "WAIT_HEALTH", []
        self.t_state = self.healthy_since = self.t_takeoff = self.offboard_lost = None
        self.corners, self.leg, self.leg_t0, self.leg_from = [], 0, None, None
        self.yaw = 0.0
        self.land_sent = False
        self.last_sp = self.last_now = self.pausing = None

    def go(self, state, now, note=""):
        self.events.append({"t": round(now, 3), "from": self.state, "to": state, "note": note})
        self.state, self.t_state = state, now

    @staticmethod
    def paused(tel):
        """Telemetry pause (hold, don't land): odometry older than ODOM_HOLD_S or local position > 1 s.

        FlyX lab 2026-11-07: run 1 landed 2.8 s after arming on a 0.3 s odometry gap, run 2 landed
        after 20 s of good flight on a 1.08 s gap (sim time advanced 1.1 s too). PX4 publishes
        vehicle_odometry continuously from its EKF, so these are pauses in the lab's PX4 -> ROS
        stream, not estimator failures: hold through them, land only past OUTAGE_S.
        """
        now = tel.now
        if tel.odom_t is not None and now - tel.odom_t > ODOM_HOLD_S:
            return f"odometry paused {now - tel.odom_t:.2f} s"
        if tel.local_t is not None and now - tel.local_t > 1.0:
            return f"local position paused {now - tel.local_t:.2f} s"
        return ""

    @staticmethod
    def unhealthy(tel):
        """Reasons to land (airborne) / not to start (on the ground)."""
        now = tel.now
        if tel.odom_t is None:
            return "odometry never received"
        if now - tel.odom_t > OUTAGE_S:
            return f"odometry lost ({now - tel.odom_t:.2f} s)"
        if tel.pose_frame != POSE_FRAME_NED:
            return f"odometry frame {tel.pose_frame} is not NED"
        if tel.status_t is None or now - tel.status_t > 3.0:
            return "vehicle_status stale"
        if tel.local_t is None or now - tel.local_t > OUTAGE_S:
            return "local position lost"
        if not (tel.xy_valid and tel.z_valid):
            return f"estimator invalid (xy {tel.xy_valid}, z {tel.z_valid})"
        if not all(isfinite(v) for v in tel.position):
            return "odometry not finite"
        return ""

    def plan(self, p0, yaw):
        """Corners in NED from the ground pose p0 and heading yaw: takeoff point, then the square."""
        fx, fy = cos(yaw), sin(yaw)                      # forward, NED (x north, y east)
        z_top = p0[2] - self.height                      # NED z is down
        a = (p0[0], p0[1], z_top)
        b = (p0[0] + fx * self.side, p0[1] + fy * self.side, z_top)
        c = (b[0], b[1], z_top - self.side)
        d = (a[0], a[1], z_top - self.side)
        return [("TAKEOFF", a), ("FORWARD", b), ("UP", c), ("BACK", d), ("DOWN", a)]

    def hold_here(self, tel):
        return tuple(tel.position), yaw_from_q_ned(tel.q)

    def tick(self, tel):
        now = tel.now
        bad = self.unhealthy(tel)
        s = self.state
        if s == "WAIT_HEALTH":
            pause = self.paused(tel)
            ok = not bad and not pause and tel.landed and tel.landed_t is not None \
                and now - tel.landed_t <= 3.0 and tel.arming_state == DISARMED
            if not ok:
                self.healthy_since = None
                return s, None, None, "waiting: " + (bad or pause or "landed + disarmed")
            self.healthy_since = self.healthy_since if self.healthy_since is not None else now
            if now - self.healthy_since < self.settle:
                return s, None, None, "healthy, confirming"
            self.yaw = yaw_from_q_ned(tel.q)
            self.corners = self.plan(tuple(tel.position), self.yaw)
            self.go("STREAM", now, f"plan from ground pose {tuple(round(v, 3) for v in tel.position)}, "
                                   f"heading {self.yaw:.3f} rad: {[(n, [round(v, 3) for v in p]) for n, p in self.corners]}")
            return self.state, self.hold_here(tel), None, "streaming hold setpoints"
        if s == "STREAM":
            if bad:
                self.go("FAILED", now, "unhealthy before Offboard: " + bad)
                return self.state, None, None, bad
            if now - self.t_state >= 1.0:
                self.go("WAIT_OFFBOARD", now)
                return self.state, self.hold_here(tel), (CMD_DO_SET_MODE, 1.0, 6.0), "requesting Offboard"
            return s, self.hold_here(tel), None, "streaming hold setpoints"
        if s == "WAIT_OFFBOARD":
            if tel.nav_state == NAV_OFFBOARD:
                self.go("WAIT_ARM", now)
                return self.state, self.hold_here(tel), (CMD_ARM_DISARM, 1.0, 0.0), "Offboard active, arming"
            if now - self.t_state > 3.0:
                self.go("FAILED", now, "PX4 did not enter Offboard")
                return self.state, None, None, ""
            return s, self.hold_here(tel), None, "waiting for Offboard"
        if s == "WAIT_ARM":
            if tel.arming_state == ARMED:
                self.t_takeoff = now
                self.leg, self.leg_t0, self.leg_from = 0, now, tuple(tel.position)
                self.last_sp, self.last_now, self.pausing = tuple(tel.position), now, None
                self.go("FLY", now, "armed")
            elif now - self.t_state > 3.0:
                self.go("FAILED", now, "PX4 refused to arm")
                return self.state, None, None, ""
            else:
                return s, self.hold_here(tel), None, "waiting for arm"
        if self.state == "FLY":
            if tel.nav_state != NAV_OFFBOARD:
                self.offboard_lost = self.offboard_lost if self.offboard_lost is not None else now
                if now - self.offboard_lost > .5:
                    self.go("TAKEN_OVER", now, f"PX4 left Offboard (nav_state {tel.nav_state}); not commanding")
                    return self.state, None, None, ""
            else:
                self.offboard_lost = None
            if bad:
                return self.land(now, "unhealthy: " + bad)
            if now - self.t_takeoff > self.max_flight:
                return self.land(now, "flight time limit")
            dt = 0.0 if self.last_now is None else max(0.0, now - self.last_now)
            self.last_now = now
            pause = self.paused(tel)
            if pause:
                # freeze the square: shift this leg's clock so the setpoint stops advancing and keep
                # streaming the last setpoint (PX4 holds position on its own estimate meanwhile)
                self.leg_t0 += dt
                if self.hold_since is not None:
                    self.hold_since += dt
                if not self.pausing:
                    self.pausing = now
                    self.events.append({"t": round(now, 3), "pause": pause, "leg": self.corners[self.leg][0]})
                return self.state, (self.last_sp, self.yaw), None, "holding: " + pause
            if self.pausing:
                self.events.append({"t": round(now, 3), "resume_after_s": round(now - self.pausing, 3)})
                self.pausing = None
            name, target = self.corners[self.leg]
            # setpoint slides from the leg start to the corner at SPEED_MPS
            length = max(dist(self.leg_from, target), 1e-6)
            frac = min(1.0, (now - self.leg_t0) * self.speed / length)
            sp = tuple(f + (t - f) * frac for f, t in zip(self.leg_from, target))
            self.last_sp = sp
            reached = frac >= 1.0 and dist(tel.position, target) <= self.tol
            if reached:
                if self.hold_since is None:
                    self.hold_since = now
                if now - self.hold_since >= self.hold:
                    self.hold_since = None
                    self.leg += 1
                    if self.leg >= len(self.corners):
                        return self.land(now, "square complete")
                    self.leg_t0, self.leg_from = now, target
                    self.events.append({"t": round(now, 3), "corner": name, "position": list(tel.position)})
            else:
                self.hold_since = None
                if now - self.leg_t0 > self.leg_timeout + length / self.speed:
                    return self.land(now, f"{name} corner not reached in time "
                                          f"({dist(tel.position, target):.2f} m away)")
            return self.state, (sp, self.yaw), None, f"{name} -> {[round(v, 2) for v in target]}"
        if self.state == "LAND":
            if tel.landed and tel.arming_state == DISARMED:
                self.go("DONE", now, "landed and disarmed")
            return self.state, None, None, "landing"
        return self.state, None, None, ""

    hold_since = None

    def land(self, now, why):
        self.go("LAND", now, why)
        if not self.land_sent:
            self.land_sent = True
            return self.state, None, (CMD_NAV_LAND, 0.0, 0.0), why
        return self.state, None, None, why


def journal_dir():
    stamp = time.strftime("%Y%m%d-%H%M%S")
    for base in (Path(__file__).resolve().parent / "runs", Path.home() / "flyx-runs", Path("/tmp/flyx-runs")):
        try:
            d = base / f"square-{stamp}"
            d.mkdir(parents=True, exist_ok=False)
            return d
        except OSError:
            continue
    return None


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--shadow", action="store_true", help="never publish to PX4")
    ap.add_argument("--height", type=float, default=HEIGHT_M)
    ap.add_argument("--side", type=float, default=SIDE_M)
    ap.add_argument("--settle-s", type=float, default=SETTLE_S, help="healthy+landed time before arming")
    ap.add_argument("--wall-clock", action="store_true", help="tests without a simulator /clock")
    a, _unknown = ap.parse_known_args(argv)           # ignore launcher extras (e.g. --ros-args)
    import rclpy
    from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
    from rclpy.executors import MultiThreadedExecutor
    from rclpy.node import Node
    from rclpy.parameter import Parameter
    from rclpy.qos import qos_profile_sensor_data
    from rclpy.signals import SignalHandlerOptions
    import px4_msgs.msg as px4
    from std_msgs.msg import String

    mission = SquareMission(height=a.height, side=a.side, settle=a.settle_s)
    out_dir = journal_dir()
    journal = open(out_dir / "journal.jsonl", "w", encoding="utf-8") if out_dir else None
    jlock = threading.Lock()

    def log(rec):
        with jlock:
            if journal and not journal.closed:
                journal.write(json.dumps(rec, default=float) + "\n")
                journal.flush()

    class SquareNode(Node):
        def __init__(self):
            super().__init__("flyx_square")
            self.set_parameters([Parameter("use_sim_time", value=not a.wall_clock)])
            self.tel, self.subs, self.last_state = Telemetry(), {}, None
            self.last_arrival, self.last_tel_log, self.last_tick = {}, 0.0, None
            # telemetry and control in separate callback groups: a slow callback never delays the other
            self.sensors, self.ctrl = MutuallyExclusiveCallbackGroup(), MutuallyExclusiveCallbackGroup()
            self.status_pub = self.create_publisher(String, "~/status", 10)
            if not a.shadow:
                self.mode_pub = self.create_publisher(px4.OffboardControlMode, "/fmu/in/offboard_control_mode",
                                                      qos_profile_sensor_data)
                self.sp_pub = self.create_publisher(px4.TrajectorySetpoint, "/fmu/in/trajectory_setpoint",
                                                    qos_profile_sensor_data)
                self.cmd_pub = self.create_publisher(px4.VehicleCommand, "/fmu/in/vehicle_command",
                                                     qos_profile_sensor_data)
            self.create_timer(1.0, self.discover, callback_group=self.sensors)
            self.create_timer(.05, self.control, callback_group=self.ctrl)
            self.get_logger().info(f"flyx_square up ({'SHADOW' if a.shadow else 'FLYING'}): climb {a.height} m, "
                                   f"square side {a.side} m; journal {out_dir}")

        def discover(self):
            graph = self.get_topic_names_and_types()
            for key, (base, typ) in OUT_TOPICS.items():
                if key not in self.subs:
                    topic = find_topic(graph, base, f"px4_msgs/msg/{typ}")
                    if topic:
                        self.subs[key] = self.create_subscription(getattr(px4, typ), topic,
                                                                  lambda m, k=key: self.on_px4(k, m),
                                                                  qos_profile_sensor_data,
                                                                  callback_group=self.sensors)
                        self.get_logger().info(f"{key}: {topic}")
                        log({"kind": "topic", "key": key, "topic": topic})

        def on_px4(self, key, m):
            t, tel = time.monotonic(), self.tel       # arrival on a monotonic clock (sim/WSL clocks jump)
            sim = self.get_clock().now().nanoseconds * 1e-9 if not a.wall_clock else None
            prev = self.last_arrival.get(key)
            # record stream pauses beyond each topic's normal spacing (status/land_detected are ~1-2 Hz)
            if prev is not None and t - prev[0] > {"odom": .15, "local": .3}.get(key, 2.0):
                log({"kind": "gap", "key": key, "wall_s": round(t - prev[0], 3),
                     "sim_s": None if sim is None or prev[1] is None else round(sim - prev[1], 3),
                     "state": mission.state})
            self.last_arrival[key] = (t, sim)
            if key == "odom":
                tel.odom_t, tel.position, tel.q, tel.pose_frame = t, tuple(m.position), tuple(m.q), m.pose_frame
                tel.odom_sim = sim
            elif key == "status":
                tel.status_t, tel.arming_state, tel.nav_state = t, m.arming_state, m.nav_state
            elif key == "land":
                tel.landed_t, tel.landed = t, bool(m.landed)
            elif key == "local":
                tel.local_t, tel.xy_valid, tel.z_valid = t, bool(m.xy_valid), bool(m.z_valid)

        def us(self):
            return int(self.get_clock().now().nanoseconds / 1000)

        def command(self, c):
            cmd = px4.VehicleCommand()
            cmd.timestamp, cmd.command, cmd.param1, cmd.param2 = self.us(), int(c[0]), float(c[1]), float(c[2])
            cmd.target_system, cmd.target_component = 1, 1
            cmd.source_system, cmd.source_component, cmd.from_external = 1, 1, True
            self.cmd_pub.publish(cmd)

        def control(self):
            mono = time.monotonic()
            sim = None if a.wall_clock else self.get_clock().now().nanoseconds * 1e-9
            if self.last_tick is not None and mono - self.last_tick > .15:   # control loop runs at 20 Hz
                log({"kind": "control_gap", "wall_s": round(mono - self.last_tick, 3), "state": mission.state})
            self.last_tick = mono
            self.tel.now, self.tel.sim_now = mono, sim
            state, sp, cmd, note = mission.tick(self.tel)
            if mono - self.last_tel_log >= .5:                                # 2 Hz telemetry record
                self.last_tel_log = mono
                tel = self.tel
                age = lambda t: None if t is None else round(mono - t, 3)
                log({"kind": "tel", "sim_t": sim, "state": state, "pos": [round(v, 3) for v in tel.position],
                     "odom_age": age(tel.odom_t), "local_age": age(tel.local_t), "status_age": age(tel.status_t),
                     "odom_sim_age": None if sim is None or tel.odom_sim is None else round(sim - tel.odom_sim, 3),
                     "xy_valid": tel.xy_valid, "z_valid": tel.z_valid, "nav": tel.nav_state,
                     "arming": tel.arming_state, "landed": tel.landed})
            if state != self.last_state:
                self.get_logger().info(f"{self.last_state} -> {state}: {mission.events[-1]['note'] if mission.events and 'note' in mission.events[-1] else note}")
                log({"kind": "state", "sim_t": self.get_clock().now().nanoseconds * 1e-9, "state": state,
                     "note": note, "event": mission.events[-1] if mission.events else None})
                self.last_state = state
            self.status_pub.publish(String(data=json.dumps({"state": state, "note": note})))
            if a.shadow:
                return
            if sp is not None:
                mode = px4.OffboardControlMode()
                mode.timestamp, mode.position = self.us(), True
                self.mode_pub.publish(mode)
                pos, yaw = sp
                msg = px4.TrajectorySetpoint()
                msg.timestamp = self.us()
                msg.position = [float(v) for v in pos]
                msg.velocity, msg.acceleration, msg.jerk = NAN3, NAN3, NAN3
                msg.yaw, msg.yawspeed = float(yaw), nan
                self.sp_pub.publish(msg)
            if cmd is not None:
                self.command(cmd)
                log({"kind": "command", "command": cmd[0], "state": state, "note": note})

        def land_if_airborne(self):
            if not a.shadow and mission.state in ("WAIT_ARM", "FLY"):
                self.command((CMD_NAV_LAND, 0.0, 0.0))
                log({"kind": "command", "command": CMD_NAV_LAND, "note": "stopped while airborne"})

    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)

    def interrupt(signum, frame):
        raise KeyboardInterrupt
    signal.signal(signal.SIGINT, interrupt)
    signal.signal(signal.SIGTERM, interrupt)
    node = SquareNode()
    executor = MultiThreadedExecutor(num_threads=3)
    executor.add_node(node)
    spinner = threading.Thread(target=executor.spin, daemon=True)
    spinner.start()
    try:
        while rclpy.ok() and spinner.is_alive() and mission.state not in ("DONE", "FAILED", "TAKEN_OVER"):
            time.sleep(.1)
    except KeyboardInterrupt:
        node.land_if_airborne()
        time.sleep(.3)
    finally:
        log({"kind": "end", "state": mission.state, "events": mission.events})
        print(f"flyx_square finished in state {mission.state}; journal {out_dir}", flush=True)
        # stop the executor and let its thread finish before destroying the node; destroying
        # while timers were still firing printed "cannot use Destroyable because destruction was
        # requested" at the end of the lab run (harmless, now avoided)
        executor.shutdown(timeout_sec=2.0)
        spinner.join(timeout=2.0)
        try:
            node.destroy_node()
        except Exception:
            pass
        with jlock:
            if journal:
                journal.close()
        rclpy.try_shutdown()
    return 0 if mission.state == "DONE" else 1


if __name__ == "__main__":
    sys.exit(main())
