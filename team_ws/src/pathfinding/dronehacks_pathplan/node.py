"""The Sky Beta competitor node: our online path planner (the FlyX program's logic) on the contract.

    subscribe  /visual_slam/tracking/odometry   where we are (ENU, odom)          [cuVSLAM]
    subscribe  /skybeta/state                    what PX4 is doing (JSON, 5 Hz)    [flight_interface]
    subscribe  <depth_topic> + CameraInfo        optional: obstacles the published map lacks
    publish    /skybeta/setpoint/pose            where we want to be (ENU, odom)   -> flight_interface
    call       /skybeta/hold                     only on a FAULT (lost odometry / PX4 position / link)
    publish    ~/status (JSON), ~/path           for people and for the sim pilot

It never touches /fmu/*, never arms, never changes mode. ``skybeta_flight_interface`` owns PX4,
the safety envelope and the switch into Offboard (when the pilot arms, then selects Position).
The state machine is ``mission.py``; this file only moves messages in and out, keeps a journal,
and runs the planner in its own process (``planner_core.py``).

Threads (the structure of the FlyX-proven flyx/fly_pathplan.py): ONE SingleThreadedExecutor runs
every ROS callback, all short -- odometry/state (just store), depth (decodes nothing: hands the
frame to the planner process), control 20 Hz and graph 1 Hz timers on a STEADY clock (a stalled
/clock or slow sim time must never stop the setpoint stream). Planning, which waits on the planner
process for up to ~1 s, runs on its own thread (runtime.PlanningLoop, 5 Hz), never on the
executor; the journal is written by its own thread from a queue (runtime.Journal). The
MultiThreadedExecutor used ~73 % of a core against ~13 % for the single-threaded one, and a
CPU-hungry process is the one a crowded lab machine starves (FlyX run 7: 86 stalls up to 1.95 s).
Freshness is judged on a monotonic clock; the ROS clock (sim time in a simulator) only times the
3-minute run limit.

Journal: <journal_dir or team_ws/runs>/pathplan-<time>/journal.jsonl, else ~/flyx-runs/...,
else /tmp/flyx-runs/... Summarise with ``ros2 run dronehacks_pathplan report``.
"""
from __future__ import annotations

from collections import deque
import json
import math
import os
from pathlib import Path
import threading
import time

import rclpy
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.clock import Clock, ClockType
from rclpy.executors import ExternalShutdownException, SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path as PathMsg
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import String
from std_srvs.srv import Trigger

from dronehacks_pathplan.mission import AIRBORNE, NAV_STATE_NAMES, Mission, MissionConfig, Telemetry
from dronehacks_pathplan.runtime import (VISION_TOPIC, Journal, PlanningLoop, foreign_publishers,
                                         make_run_dir, match_pose)

MAX_TURN_RATE = .5          # rad/s: depth frames taken while turning faster are dropped (pose error)

PARAMS = {                  # name: (default, description) -- launch arguments of the same name
    'shadow': (False, 'plan and log only; never publish a setpoint (Offboard can then not start)'),
    'flight_altitude_m': (0.9, 'cruise height above the floor (walls are 1.73 m)'),
    'carrot_m': (0.5, 'setpoint distance ahead on the plan; pose setpoints are not speed-capped'),
    'planner': ('theta', 'theta | astar | static (published map only)'),
    'planner_process': (True, 'run the planner in its own process (recommended)'),
    'depth_topic': ('', 'depth sensor_msgs/Image (16UC1 mm or 32FC1 m); empty = published map only'),
    'camera_info_topic': ('', 'CameraInfo of the depth image; empty = <depth topic parent>/camera_info'),
    'camera_offset_flu_m': ([0.115, 0.0, 0.029], 'depth camera origin in body FLU (drone.env CAMERA_X/Y/Z)'),
    'camera_pitch_down_deg': (0.0, 'depth camera pitch, down positive'),
    'launch_xy_m': ([-2.515, 0.914], 'launch mark, course frame'),
    'launch_heading_course_rad': (math.pi / 2, 'heading on the mark, course frame (up the first corridor)'),
    'launch_height_m': (0.05, 'body origin above the floor when sitting on the mark'),
    'frame_latch': ('arm', 'arm: course frame from the pose when the pilot arms | slam_origin: SLAM started on the mark'),
    'min_altitude_m': (0.3, "flight_interface's floor: MUST equal its min_altitude_m"),
    'max_flight_s': (175.0, 'run limit from takeoff (rule 7.9: 3 min); then descend + hold'),
    'yaw_rate_max': (0.25, 'rad/s cap on the commanded heading (VIO loses tracking on fast turns)'),
    'climb_rate': (0.4, 'm/s takeoff ramp'),
    'descend_rate': (0.3, 'm/s final descent ramp'),
    'odometry_topic': ('/visual_slam/tracking/odometry', 'cuVSLAM odometry'),
    'state_topic': ('/skybeta/state', 'flight_interface state'),
    'setpoint_topic': ('/skybeta/setpoint/pose', 'flight_interface pose setpoint'),
    'journal_dir': ('', 'where runs go; empty = <team_ws>/runs (then ~/flyx-runs, /tmp/flyx-runs)'),
}


def _runs_dir():
    here = Path(os.path.realpath(__file__))
    for p in here.parents:
        if p.name == 'team_ws':
            return p / 'runs'
    return Path.home() / 'dronehacks_runs'


class PathplanNode(Node):

    def __init__(self, **kwargs):
        super().__init__('dronehacks_pathplan', **kwargs)
        for name, (default, help_) in PARAMS.items():     # dynamic: "175" from a launch file is fine
            self.declare_parameter(name, default, ParameterDescriptor(description=help_, dynamic_typing=True))
        g = lambda n: self.get_parameter(n).value
        f = lambda n: float(g(n))
        b = lambda n: str(g(n)).lower() in ('true', '1', 'yes')
        self.shadow = b('shadow')
        self.cfg = MissionConfig(
            shadow=self.shadow, altitude=f('flight_altitude_m'), carrot_m=f('carrot_m'),
            launch_xy=tuple(float(v) for v in g('launch_xy_m')), launch_heading=f('launch_heading_course_rad'),
            launch_height=f('launch_height_m'), frame_latch=str(g('frame_latch')),
            min_altitude_m=f('min_altitude_m'), max_flight_s=f('max_flight_s'), yaw_rate_max=f('yaw_rate_max'),
            climb_rate=f('climb_rate'), descend_rate=f('descend_rate'))
        if self.cfg.frame_latch not in ('arm', 'slam_origin'):
            raise ValueError("frame_latch must be 'arm' or 'slam_origin'")

        # ---- journal (its own writer thread: a slow disk never holds up a callback)
        out = make_run_dir(str(g('journal_dir')) or _runs_dir(), time.strftime('pathplan-%Y%m%d-%H%M%S'))
        self.journal_path = out / 'journal.jsonl'
        self._journal = Journal(self.journal_path)
        self.log({'kind': 'start', 'params': {n: g(n) for n in PARAMS}})

        # ---- the planner (its own process: planning must never stall the setpoint stream)
        kwargs = dict(planner_name=str(g('planner')), altitude=self.cfg.altitude,
                      camera_offset=tuple(float(v) for v in g('camera_offset_flu_m')),
                      pitch_down_deg=f('camera_pitch_down_deg'), carrot_m=self.cfg.carrot_m)
        self.get_logger().info('building the course map from the published layout...')
        from dronehacks_pathplan.planner_core import InlinePlanner, PlannerProcess
        self.planner = PlannerProcess(**kwargs) if b('planner_process') else InlinePlanner(**kwargs)
        self.mission = Mission(self.planner, self.cfg)

        # ---- telemetry (callbacks write, the control loop reads)
        self.tel = Telemetry(now=time.monotonic())
        self.poses = deque(maxlen=150)          # (stamp s, position, q): ~5 s of odometry for depth
        self.poses_lock = threading.Lock()
        self.intr = None
        self.depth_stats = {'frames': 0, 'given': 0, 'turning': 0, 'matched': 0, 'fallback': 0, 'unmatched': 0}
        self.last_px4 = None
        self.stalls, self.last_tick = [], None
        self.last_state_out = None
        self.reset_pending = False

        # one SingleThreadedExecutor runs all of these (main()); planning is NOT a callback
        self.create_subscription(Odometry, str(g('odometry_topic')), self.on_odom, qos_profile_sensor_data)
        self.create_subscription(String, str(g('state_topic')), self.on_state, 10)
        depth = str(g('depth_topic'))
        if depth:
            info = str(g('camera_info_topic')) or depth.rsplit('/', 1)[0] + '/camera_info'
            self.create_subscription(Image, depth, self.on_depth, qos_profile_sensor_data)
            self.create_subscription(CameraInfo, info, self.on_info, qos_profile_sensor_data)
        self.setpoint_topic = str(g('setpoint_topic'))
        self.sp_pub = None if self.shadow else self.create_publisher(PoseStamped, self.setpoint_topic, 10)
        self.status_pub = self.create_publisher(String, '~/status', 10)
        self.path_pub = self.create_publisher(PathMsg, '~/path', 10)
        self.hold_client = self.create_client(Trigger, '/skybeta/hold')
        steady = Clock(clock_type=ClockType.STEADY_TIME)
        self.create_timer(.05, self.on_control, clock=steady)
        self.create_timer(1.0, self.on_graph, clock=steady)
        # planning: its own thread, because it blocks on the planner process; main() starts it
        self.planning = PlanningLoop(self.on_plan, .2, on_error=lambda tb: self.log(
            {'kind': 'error', 'where': 'planning', 'error': tb}))
        self.get_logger().info(
            f'dronehacks_pathplan up ({"SHADOW: no setpoints" if self.shadow else "flying"}): planner '
            f'{g("planner")}, altitude {self.cfg.altitude:.2f} m, depth {depth or "OFF (published map only)"}, '
            f'frame latched at {"arming" if self.cfg.frame_latch == "arm" else "SLAM start"}.\n'
            f'  Place the drone ON THE LAUNCH MARK facing up the first corridor. Journal: {self.journal_path}')

    # ------------------------------------------------------------------ journal
    def log(self, record):
        self._journal.log(record)                   # queued; the journal thread writes it

    def ros_now(self):
        t = self.get_clock().now().nanoseconds * 1e-9
        return t if t > 0 else None              # None: sim time with no /clock yet

    # ------------------------------------------------------------------ inputs
    def on_odom(self, msg):
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        pos, quat = (float(p.x), float(p.y), float(p.z)), (float(q.x), float(q.y), float(q.z), float(q.w))
        if not all(math.isfinite(v) for v in pos + quat) or sum(v * v for v in quat) < .5:
            return
        tel = self.tel
        tel.odom_t, tel.position, tel.q_xyzw = time.monotonic(), pos, quat
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        with self.poses_lock:
            self.poses.append((stamp, pos, quat))

    def on_state(self, msg):
        try:
            s = json.loads(msg.data)
        except ValueError:
            return
        if not isinstance(s, dict):
            return
        tel = self.tel
        tel.px4_link = bool(s.get('px4_link', False))
        tel.armed = bool(s.get('armed', False))
        tel.landed = s.get('landed')
        tel.nav_state = int(s.get('nav_state', -1))
        tel.offboard_active = bool(s.get('offboard_active', False))
        tel.local_valid = s.get('local_enu') is not None
        tel.frozen = bool(s.get('frozen', False))
        origin = s.get('origin_enu')
        tel.origin_z = float(origin[2]) if origin else None
        tel.state_t = time.monotonic()
        key = (tel.nav_state, tel.armed, bool(tel.landed), tuple(s.get('clamped') or ()))
        if key != self.last_px4:
            self.last_px4 = key
            odom, local = s.get('odom_enu'), s.get('local_enu')
            gap = (round(math.dist(odom, local), 3) if odom and local else None)
            self.log({'kind': 'px4', 't': self.ros_now(), 'nav_state': tel.nav_state,
                      'mode': NAV_STATE_NAMES.get(tel.nav_state, str(tel.nav_state)), 'armed': tel.armed,
                      'landed': tel.landed, 'clamped': s.get('clamped'), 'odom_local_gap_m': gap})
            if s.get('clamped') and self.mission.state in AIRBORNE:   # on the ground the 0.3 m floor always clamps
                self.get_logger().warn(f'flight_interface is CLAMPING our setpoints: {s["clamped"]}')
        self._state_raw = s

    def on_info(self, msg):
        if self.intr is None:
            self.intr = (list(msg.k), msg.width, msg.height)
            self.log({'kind': 'camera_info', 'k': list(msg.k), 'width': msg.width, 'height': msg.height,
                      'frame_id': msg.header.frame_id})
            self.get_logger().info(f'depth camera {msg.width}x{msg.height}, fx {msg.k[0]:.1f}')

    def pose_at(self, stamp):
        """(position, q, turn rate) at the image stamp, or None: skip the frame (runtime.match_pose;
        after a stall an unmatched frame is skipped once stamps have matched, as in the flown file)."""
        with self.poses_lock:
            poses = list(self.poses)
        return match_pose(poses, stamp, self.depth_stats)

    def on_depth(self, msg):
        m = self.mission
        if m.frame is None or self.intr is None or not m.planning_allowed():
            return
        st = self.depth_stats
        st['frames'] += 1
        found = self.pose_at(msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9)
        if found is None:
            return
        pos, q, rate = found
        if rate > MAX_TURN_RATE:
            st['turning'] += 1
            return
        xyz, _, rotation = m.frame.odometry_to_course(pos, q)
        k, w, h = self.intr
        self.planner.give_frame((bytes(msg.data), msg.width, msg.height, msg.encoding, msg.step, k, xyz, rotation))
        st['given'] += 1
        if st['frames'] % 100 == 1:
            self.log({'kind': 'depth', 't': self.ros_now(), **st, 'quality': self.planner.depth_quality})

    # ------------------------------------------------------------------ graph checks (1 Hz)
    def on_graph(self):
        topics = (self.setpoint_topic, '/skybeta/setpoint/twist', '/skybeta/state', VISION_TOPIC)
        foreign = foreign_publishers({t: self.count_publishers(t) for t in topics}, self.setpoint_topic,
                                     own_setpoint=self.sp_pub is not None)
        if foreign != self.tel.foreign:
            # one call site per severity: rclpy raises "Logger severity cannot be changed between calls"
            if foreign:
                self.get_logger().error(foreign)
            else:
                self.get_logger().info('no other setpoint publisher / flight interface')
            self.log({'kind': 'foreign', 't': self.ros_now(), 'note': foreign})
        self.tel.foreign = foreign

    # ------------------------------------------------------------------ planning (5 Hz)
    def on_plan(self):
        if self.reset_pending:
            self.planner.reset()
            self.reset_pending = False
            self.log({'kind': 'planner_reset', 't': self.ros_now()})
        t0 = time.perf_counter()
        cmd = self.mission.plan()
        if cmd is None:
            return
        target = self.mission.target
        self.log({'kind': 'plan', 't': self.ros_now(), 'mode': cmd.mode, 'reason': cmd.reason,
                  'ms': round(1e3 * (time.perf_counter() - t0), 1), 'planner_ms': round(self.planner.ms, 1),
                  'pose': [round(float(v), 3) for v in self.mission.pose_course[0]],
                  'target': None if target is None else [[round(float(v), 3) for v in target[0]], round(float(target[1]), 3)],
                  'cleared': [c['name'] for c in self.planner.cleared]})
        if cmd.path is not None and self.mission.frame is not None:
            out = PathMsg()
            out.header.frame_id = 'odom'
            out.header.stamp = self.get_clock().now().to_msg()
            for x, y in cmd.path:
                ps = PoseStamped()
                ps.header = out.header
                o = self.mission.frame.to_odom((x, y, self.cfg.altitude))
                ps.pose.position.x, ps.pose.position.y, ps.pose.position.z = (float(v) for v in o)
                out.poses.append(ps)
            self.path_pub.publish(out)

    # ------------------------------------------------------------------ control (20 Hz, steady clock)
    def on_control(self):
        mono = time.monotonic()
        if self.last_tick is not None and mono - self.last_tick > .15:
            self.stalls.append(round(mono - self.last_tick, 3))
            self.log({'kind': 'timing', 't': self.ros_now(), 'gap_s': round(mono - self.last_tick, 3)})
        self.last_tick = mono
        tel = self.tel
        tel.now, tel.flight_clock = mono, self.ros_now()
        before = self.mission.state
        out = self.mission.tick(tel)
        if out.reset_planner:
            self.reset_pending = True
        if out.state != before:
            ev = self.mission.events[-1] if self.mission.events else {}
            note = ev.get('note', '')
            if out.state in ('FAULT', 'TAKEN_OVER', 'LOW_HOLD'):
                self.get_logger().warn(f'{before} -> {out.state}: {note}')
            else:
                self.get_logger().info(f'{before} -> {out.state}: {note}')
            self.log({'kind': 'state', 't': self.ros_now(), 'from': before, 'state': out.state, 'note': note,
                      'pose': None if self.mission.pose_course is None
                      else [round(float(v), 3) for v in self.mission.pose_course[0]]})
            if out.state in ('DONE',):
                self.print_summary()
        if out.setpoint is not None and self.sp_pub is not None:
            self.publish_setpoint(*out.setpoint)
        if out.hold_request:
            self.request_hold(out.note)
        if int(mono * 20) % 2 == 0:                       # status at ~10 Hz
            m = self.mission
            pc = m.pose_course
            self.status_pub.publish(String(data=json.dumps({
                'state': out.state, 'note': out.note, 'land_request': out.land_request,
                'planner': m.planner_status, 'cleared': [c['name'] for c in self.planner.cleared],
                'n_cleared': len(self.planner.cleared), 'n_checkpoints': self.planner.n_checkpoints,
                'pose_course': None if pc is None else [round(float(v), 3) for v in pc[0]] + [round(float(pc[1]), 3)],
                'flight_s': round(m.flight_time(tel), 1), 'result': m.result, 'shadow': self.shadow,
                'journal': str(self.journal_path)})))

    def publish_setpoint(self, position, yaw):
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'odom'
        msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = (float(v) for v in position)
        if yaw is None:
            msg.pose.orientation.w = 0.0                   # all-zero quaternion: do not command yaw
        else:
            msg.pose.orientation.z, msg.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
        self.sp_pub.publish(msg)

    def request_hold(self, why=''):
        self.log({'kind': 'hold_request', 't': self.ros_now(), 'why': why})
        if self.hold_client.service_is_ready():
            self.hold_client.call_async(Trigger.Request())          # never waits on the executor thread
            self.get_logger().error(f'called /skybeta/hold: {why}')
        else:
            self.get_logger().error(f'/skybeta/hold not available; setpoints stopped: {why}')

    # ------------------------------------------------------------------ summary
    def print_summary(self):
        m, p = self.mission, self.planner
        lines = ['', '=' * 72, f'RESULT: {m.result or m.state}',
                 f'checkpoints cleared: {len(p.cleared)}/{p.n_checkpoints}: ' + ', '.join(c['name'] for c in p.cleared),
                 'states: ' + ' -> '.join(e['to'] for e in m.events[-14:])]
        if self.stalls:
            lines.append(f'control-loop stalls > 0.15 s: {len(self.stalls)}, worst {max(self.stalls):.2f} s')
        if m.px4_failsafe_waits:
            lines.append(f'PX4 failsafe Holds / link gaps ridden out: {m.px4_failsafe_waits * .05:.1f} s in total')
        q = p.depth_quality or {}
        if q.get('hits'):
            lines.append(f"depth: {self.depth_stats['given']} frames used, {self.depth_stats['turning']} dropped (turning); "
                         f"{q['hits_on_published_walls'] / q['hits']:.0%} of hits on published walls "
                         f"(low = wrong camera mount / frame); new obstacle cells {q['hits_new_confirmed']}")
        lines += [f'journal: {self.journal_path}', '=' * 72]
        for line in lines:
            self.get_logger().info(line)
        self.log({'kind': 'summary', 't': self.ros_now(), 'result': m.result, 'state': m.state,
                  'cleared': [c['name'] for c in p.cleared], 'events': m.events[-30:], 'stalls': self.stalls[-50:],
                  'depth': self.depth_stats, 'depth_quality': q, 'planner_events': p.events[-30:]})

    def close(self):
        if self.mission.state in AIRBORNE:
            self.get_logger().error(f'shutting down in {self.mission.state}: flight_interface keeps holding the '
                                    'last setpoint. PILOT: take over and land.')
        self.planning.stop()                    # finish the step in flight before closing its pipe
        self.print_summary()
        self.planner.close()
        self._journal.close()


def main(args=None):
    rclpy.init(args=args)
    node = PathplanNode()
    # Single-threaded executor: every callback is short. The MultiThreadedExecutor burnt ~73 % of a
    # core on this message load (single: ~13 %); FlyX run 7 saw 86 stalls up to 1.95 s with it.
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    node.planning.start()
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.close()
        executor.shutdown()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
