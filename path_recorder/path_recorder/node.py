"""path_recorder: records the path a pilot flies on RC, one recording per arm -> disarm.

    subscribe  /visual_slam/tracking/odometry   the pose (ENU odom: the frame /skybeta/setpoint/pose uses)
    subscribe  /skybeta/state                    armed / landed / mode / PX4 position valid  [flight_interface]
    subscribe  depth image + camera_info         clearance ahead at each point (optional, auto-detected)
    publish    /path_recorder/status             JSON, 2 Hz, for the dashboard (ros2 run path_recorder dashboard)

It publishes NOTHING to /skybeta/setpoint/* and nothing to /fmu/*: with flight_interface running
beside it (path_recorder.launch.py), a switch to Position can never start Offboard, so the pilot
always has the drone. Recordings go to <team_ws>/runs/paths/path-<date>-<time>-<n>/ (see README).
"""
from __future__ import annotations

from collections import deque
import json
import math
import time

import numpy as np
import rclpy
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import Odometry
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import String

from path_recorder.recording import NAV_STATE_NAMES, Recording, clearance, decode_depth, runs_dir, yaw_of

PARAMS = {
    'spacing_m': (0.15, 'save a point every this many metres of travel'),
    'turn_deg': (15.0, 'and every this many degrees of heading change'),
    'odometry_topic': ('/visual_slam/tracking/odometry', 'cuVSLAM odometry (ENU odom)'),
    'state_topic': ('/skybeta/state', 'flight_interface state JSON'),
    'depth_topics': (['/camera/depth/image_raw', '/oak/stereo/image_raw'],
                     'depth images to try, in order; the first one with a publisher is used'),
    'gap_s': (0.3, 'odometry pause longer than this flags the next point odom_gap'),
    'jump_m': (0.3, 'a pose step larger than this between two odometry messages flags odom_jump'),
    'output_dir': ('', 'where recordings go; empty = <team_ws>/runs/paths'),
}


class PathRecorder(Node):

    def __init__(self):
        super().__init__('path_recorder')
        for name, (default, help_) in PARAMS.items():
            self.declare_parameter(name, default, ParameterDescriptor(description=help_, dynamic_typing=True))
        g = lambda n: self.get_parameter(n).value
        self.spacing, self.turn = float(g('spacing_m')), float(g('turn_deg'))
        self.gap_s, self.jump_m = float(g('gap_s')), float(g('jump_m'))
        self.out = runs_dir() if not g('output_dir') else __import__('pathlib').Path(str(g('output_dir')))
        self.out.mkdir(parents=True, exist_ok=True)
        self.depth_candidates = list(g('depth_topics'))
        self.odom_topic = str(g('odometry_topic'))

        self.rec = None
        self.done = []                       # summaries of this boot's finished recordings
        self.state = {}
        self.state_t = None
        self.armed_prev = None               # None until the first /skybeta/state
        self.odom_t, self.odom_pos, self.odom_q = None, None, None
        self.odom_times = deque(maxlen=200)
        self.depth_times = deque(maxlen=100)
        self.depth_topic, self.depth_sub, self.info_sub = None, None, None
        self.fx = None
        self.clear, self.clear_t = (None, None, None), None
        self.last_depth_calc = 0.0
        self.warnings = {}
        self.count = 0

        sensors, depth, slow = (MutuallyExclusiveCallbackGroup() for _ in range(3))
        self.depth_group = depth
        self.create_subscription(Odometry, self.odom_topic, self.on_odom, qos_profile_sensor_data,
                                 callback_group=sensors)
        self.create_subscription(String, str(g('state_topic')), self.on_state, 10, callback_group=sensors)
        self.status_pub = self.create_publisher(String, '~/status', 10)
        self.create_timer(.5, self.on_status, callback_group=slow)
        self.create_timer(2.0, self.find_depth, callback_group=slow)
        self.get_logger().info(
            f'path_recorder up: a recording starts when the drone is ARMED and ends when it is DISARMED. '
            f'Points every {self.spacing:.2f} m / {self.turn:.0f} deg. Saving to {self.out}. '
            'This node never commands the drone. Watch it: ros2 run path_recorder dashboard')

    # ------------------------------------------------------------------ depth (optional)
    def find_depth(self):
        if self.depth_topic is not None:
            if time.monotonic() - (self.depth_times[-1] if self.depth_times else 0) > 5.0:
                self.warn('depth', f'no depth images on {self.depth_topic} for 5 s: clearance columns empty')
            return
        for t in self.depth_candidates:
            if self.count_publishers(t) > 0:
                self.depth_topic = t
                info = t.rsplit('/', 1)[0] + '/camera_info'
                self.depth_sub = self.create_subscription(Image, t, self.on_depth, qos_profile_sensor_data,
                                                          callback_group=self.depth_group)
                self.info_sub = self.create_subscription(CameraInfo, info, self.on_info, qos_profile_sensor_data,
                                                         callback_group=self.depth_group)
                self.get_logger().info(f'depth: using {t}')
                self.warnings.pop('depth', None)
                return
        self.warn('depth', 'no depth topic found (' + ', '.join(self.depth_candidates) + '): clearance columns empty')

    def on_info(self, msg):
        self.fx = float(msg.k[0])

    def on_depth(self, msg):
        now = time.monotonic()
        self.depth_times.append(now)
        if now - self.last_depth_calc < .2:          # 5 Hz is plenty for clearance; keeps the CPU low
            return
        self.last_depth_calc = now
        try:
            d = decode_depth(bytes(msg.data), msg.width, msg.height, msg.encoding, msg.step)
            self.clear, self.clear_t = clearance(d), now
            self.warnings.pop('depth_enc', None)
        except ValueError as e:
            self.warn('depth_enc', str(e))

    # ------------------------------------------------------------------ state + odometry
    def warn(self, key, text):
        if self.warnings.get(key) != text:
            self.get_logger().warn(text)
        self.warnings[key] = text

    def on_state(self, msg):
        try:
            s = json.loads(msg.data)
        except ValueError:
            return
        if not isinstance(s, dict):
            return
        prev = self.state
        self.state, self.state_t = s, time.monotonic()
        armed = bool(s.get('armed', False))
        if self.rec is not None:
            for k in ('nav_state', 'landed', 'offboard_active', 'px4_link'):
                if prev.get(k) != s.get(k):
                    v = NAV_STATE_NAMES.get(s.get(k), s.get(k)) if k == 'nav_state' else s.get(k)
                    self.rec.event(k, value=v)
            if (prev.get('local_enu') is None) != (s.get('local_enu') is None):
                self.rec.event('px4_position_valid', value=s.get('local_enu') is not None)
        if self.armed_prev is None:                   # first state after (re)start
            if armed:
                self.start(started_armed=True)
        elif armed and not self.armed_prev:
            self.start(started_armed=False)
        elif not armed and self.armed_prev:
            self.stop('disarm')
        self.armed_prev = armed

    def start(self, started_armed):
        if self.rec is not None:
            self.stop('re-armed')
        if self.odom_pos is None or time.monotonic() - self.odom_t > 1.0:
            self.warn('anchor', 'ARMED but no fresh odometry: anchoring this recording on the last known pose '
                                '(is cuVSLAM running?)')
        pos = self.odom_pos if self.odom_pos is not None else (0.0, 0.0, 0.0)
        q = self.odom_q if self.odom_q is not None else (0.0, 0.0, 0.0, 1.0)
        self.count += 1
        folder = self.out / time.strftime(f'path-%Y%m%d-%H%M%S-{self.count:03d}')
        self.rec = Recording(folder, pos, q, started_armed=started_armed, spacing_m=self.spacing, turn_deg=self.turn,
                             info={'odometry_topic': self.odom_topic, 'depth_topic': self.depth_topic,
                                   'depth_columns': 'clear_left/center/right: metres to the nearest thing in the '
                                                    'forward-left / forward / forward-right thirds of the depth view'})
        self.get_logger().warn(f'ARMED -> RECORDING {folder.name}' + (' (recorder started while armed: anchor is '
                               'NOT the start spot)' if started_armed else ''))

    def stop(self, why):
        if self.rec is None:
            return
        r = self.rec
        r.close(why)
        m = r.meta()
        self.done.append({'name': r.folder.name, 'points': m['points'], 'length_m': m['length_m'],
                          'duration_s': m['duration_s'], 'aborted_or_short': m['aborted_or_short']})
        self.get_logger().warn(f'{why.upper()} -> SAVED {r.folder} : {m["points"]} points, {m["length_m"]:.2f} m, '
                               f'{m["duration_s"]:.1f} s, airborne {m["airborne_s"]:.1f} s, flags {m["flags"]}'
                               + ('  (short / aborted?)' if m['aborted_or_short'] else ''))
        self.rec = None

    def on_odom(self, msg):
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        pos, quat = (p.x, p.y, p.z), (q.x, q.y, q.z, q.w)
        if not all(math.isfinite(v) for v in pos + quat) or sum(v * v for v in quat) < .5:
            return
        now = time.monotonic()
        gap = self.odom_t is not None and now - self.odom_t > self.gap_s
        jump = self.odom_pos is not None and math.dist(pos, self.odom_pos) > self.jump_m
        if jump:
            self.warn('jump', f'odometry jumped {math.dist(pos, self.odom_pos):.2f} m (cuVSLAM reset/relocalised?)')
        if self.rec is not None and gap:
            self.rec.event('odom_gap', seconds=round(now - self.odom_t, 3))
        if self.rec is not None and jump:
            self.rec.event('odom_jump', metres=round(math.dist(pos, self.odom_pos), 3))
        self.odom_t, self.odom_pos, self.odom_q = now, pos, quat
        self.odom_times.append(now)
        if self.rec is None:
            return
        s = self.state
        depth_age = None if self.clear_t is None else now - self.clear_t
        self.rec.odometry(now, pos, quat, landed=bool(s.get('landed', True)), nav_state=int(s.get('nav_state', -1)),
                          clear=self.clear if depth_age is not None and depth_age < .5 else (None, None, None),
                          depth_age=depth_age, local_valid=s.get('local_enu') is not None, gap=gap, jump=jump)

    # ------------------------------------------------------------------ status for the dashboard
    def rate(self, times, window=2.0):
        now = time.monotonic()
        return round(sum(1 for t in times if now - t <= window) / window, 1)

    def on_status(self):
        now = time.monotonic()
        s, r = self.state, self.rec
        if self.state_t is None or now - self.state_t > 2.0:
            self.warn('state', 'no /skybeta/state: is flight_interface running? (arming will not start a recording)')
        else:
            self.warnings.pop('state', None)
        rel, last_points = None, []
        if r is not None:
            try:                                      # the odometry thread may be appending right now
                last_points = list(r.recent)[-5:]
            except RuntimeError:
                pass
        if r is not None and r.last_pose is not None:
            _, xyz, ryaw = r.last_pose
            rel = [round(float(v), 3) for v in xyz] + [round(math.degrees(ryaw), 1)]
        self.status_pub.publish(String(data=json.dumps({
            'recording': r is not None,
            'folder': None if r is None else str(r.folder),
            'points': 0 if r is None else r.n,
            'length_m': 0.0 if r is None else round(r.length, 2),
            'duration_s': 0.0 if r is None else round(r.duration, 1),
            'airborne_s': 0.0 if r is None else round(r.airborne_s, 1),
            'max_height_m': 0.0 if r is None else round(r.max_z, 2),
            'flags': {} if r is None else r.flag_counts,
            'rel_pose': rel,
            'last_points': last_points,
            'armed': bool(s.get('armed', False)), 'landed': s.get('landed'),
            'mode': NAV_STATE_NAMES.get(s.get('nav_state', -1), str(s.get('nav_state'))),
            'offboard': bool(s.get('offboard_active', False)),
            'px4_link': bool(s.get('px4_link', False)), 'px4_position_valid': s.get('local_enu') is not None,
            'state_age_s': None if self.state_t is None else round(now - self.state_t, 2),
            'odom_hz': self.rate(self.odom_times), 'odom_age_s': None if self.odom_t is None else round(now - self.odom_t, 2),
            'depth_topic': self.depth_topic, 'depth_hz': self.rate(self.depth_times),
            'clearance': list(self.clear) if self.clear_t is not None and now - self.clear_t < 1.0 else None,
            'output_dir': str(self.out), 'finished_this_boot': self.done[-5:],
            'warnings': list(self.warnings.values()),
        })))

    def close(self):
        if self.rec is not None:
            self.stop('shutdown')


def main(args=None):
    rclpy.init(args=args)
    node = PathRecorder()
    ex = MultiThreadedExecutor(num_threads=3)
    ex.add_node(node)
    try:
        ex.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.close()                     # saves an open recording first
        try:                             # a second Ctrl-C during teardown must not matter
            ex.shutdown()
            node.destroy_node()
        except (KeyboardInterrupt, Exception):
            pass
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
