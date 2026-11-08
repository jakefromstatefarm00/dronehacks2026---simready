"""READ-ONLY pipeline check: is everything our node needs actually flowing? Sends nothing.

    ros2 run dronehacks_pathplan check_pipeline                    # ~15 s
    ros2 run dronehacks_pathplan check_pipeline --depth-topic ''   # the stock drone (no depth)

Run it with the drone/sim sitting on the launch mark, before launching (it then reports
flight_interface missing as a WARN) or with the stack up (the competitor window running). It
checks, for real messages over the window (topic names alone prove nothing):

  cuVSLAM odometry      rate, freshness, finite pose
  flight_interface      /skybeta/state: PX4 link, landed, disarmed, PX4 local position valid,
                        odom_enu vs local_enu (does PX4's frame match the vision frame?)
  vision into PX4       exactly one publisher on /fmu/in/vehicle_visual_odometry
  who commands PX4      at most one flight_interface; nobody else on /fmu/in setpoints or
                        /skybeta/setpoint/*
  depth (optional)      rate, encoding, size, CameraInfo
  sim clock             /clock present (simulator) or absent (drone)
"""
from __future__ import annotations

import argparse
import json
import math
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import Odometry
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import String


class Probe(Node):
    def __init__(self, a):
        super().__init__('dronehacks_check_pipeline')
        self.a = a
        self.odom, self.state, self.depth, self.info, self.clock = [], [], [], None, 0
        self.create_subscription(Odometry, a.odometry_topic, lambda m: self.odom.append((time.monotonic(), m)),
                                 qos_profile_sensor_data)
        self.create_subscription(String, '/skybeta/state', lambda m: self.state.append((time.monotonic(), m.data)), 10)
        self.create_subscription(Clock, '/clock', self.on_clock, qos_profile_sensor_data)
        if a.depth_topic:
            info = a.depth_topic.rsplit('/', 1)[0] + '/camera_info'
            self.create_subscription(Image, a.depth_topic, self.on_depth, qos_profile_sensor_data)
            self.create_subscription(CameraInfo, info, self.on_info, qos_profile_sensor_data)

    def on_clock(self, _):
        self.clock += 1

    def on_depth(self, m):
        self.depth.append((time.monotonic(), m.encoding, m.width, m.height))

    def on_info(self, m):
        self.info = (m.width, m.height, [round(float(v), 2) for v in list(m.k)[:6]])


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--seconds', type=float, default=15.0)
    ap.add_argument('--odometry-topic', default='/visual_slam/tracking/odometry')
    ap.add_argument('--depth-topic', default='/camera/depth/image_raw', help="'' to skip (stock drone)")
    a = ap.parse_args([x for x in (argv if argv is not None else sys.argv[1:]) if not x.startswith('--ros-args')])
    rclpy.init()
    node = Probe(a)
    print(f'listening for {a.seconds:.0f} s (read-only)...', flush=True)
    end = time.monotonic() + a.seconds
    while time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=.1)
    results = []

    def res(level, what, detail=''):
        results.append(level)
        print(f'[{level:4s}] {what}' + (f' - {detail}' if detail else ''))

    now, span = time.monotonic(), a.seconds
    # ---- odometry
    if not node.odom:
        res('FAIL', f'{a.odometry_topic}: no messages', 'cuVSLAM not running / not tracking (or wrong topic)')
    else:
        rate = len(node.odom) / span
        age = now - node.odom[-1][0]
        p = node.odom[-1][1].pose.pose.position
        ok = rate > 10 and age < .5 and all(math.isfinite(v) for v in (p.x, p.y, p.z))
        res('OK' if ok else 'WARN', f'{a.odometry_topic}: {rate:.1f} Hz, last {age:.2f} s ago',
            f'pose ({p.x:.2f}, {p.y:.2f}, {p.z:.2f}) frame {node.odom[-1][1].header.frame_id!r}')
    # ---- flight interface
    n_state = node.count_publishers('/skybeta/state')
    if not node.state:
        res('WARN' if n_state == 0 else 'FAIL', '/skybeta/state: no messages',
            'flight_interface not running yet (fine before you launch)' if n_state == 0 else 'publisher exists but silent')
    else:
        s = json.loads(node.state[-1][1])
        res('OK' if n_state == 1 else 'FAIL', f'flight_interface instances: {n_state}',
            '' if n_state == 1 else 'two interfaces fight over PX4: stop one')
        res('OK' if s.get('px4_link') else 'FAIL', f"PX4 link: {s.get('px4_link')}",
            '' if s.get('px4_link') else 'vehicle_status not arriving: XRCE agent / topic versions')
        res('OK' if s.get('local_enu') else 'FAIL', f"PX4 local position valid: {s.get('local_enu') is not None}",
            '' if s.get('local_enu') else 'EKF2 has no position: is vision reaching PX4?')
        res('OK' if s.get('landed') and not s.get('armed') else 'WARN',
            f"landed {s.get('landed')}, armed {s.get('armed')}, mode {s.get('nav_state_name')}")
        o, loc = s.get('odom_enu'), s.get('local_enu')
        if o and loc:
            gap = math.dist(o, loc)
            res('OK' if gap < .15 else 'WARN', f'odom_enu vs local_enu: {gap:.3f} m apart',
                'PX4 frame = vision frame' if gap < .15 else
                'PX4 local frame differs from odom: setpoints land offset by this (anchor_setpoints?)')
        if s.get('clamped'):
            res('WARN', f"setpoints being clamped: {s.get('clamped')}")
    # ---- vision into PX4, who commands PX4
    n = node.count_publishers('/fmu/in/vehicle_visual_odometry')
    res('OK' if n == 1 else ('WARN' if n == 0 else 'FAIL'), f'/fmu/in/vehicle_visual_odometry publishers: {n}',
        {0: 'PX4 gets no vision (fine in a GPS simulator; on the drone the bridge window is down)',
         1: ''}.get(n, 'two vision bridges: PX4 fuses every sample twice - stop one'))
    for t in ('/fmu/in/trajectory_setpoint', '/fmu/in/offboard_control_mode'):
        n = node.count_publishers(t)
        res('OK' if n <= 1 else 'FAIL', f'{t} publishers: {n}',
            '' if n <= 1 else 'something besides flight_interface commands PX4 (an old script? upload_to_flyx?)')
    for t in ('/skybeta/setpoint/pose', '/skybeta/setpoint/twist'):
        n = node.count_publishers(t)
        res('OK' if n <= 1 else 'FAIL', f'{t} publishers: {n}', '' if n <= 1 else 'two competitor nodes running')
    # ---- depth
    if a.depth_topic:
        if not node.depth:
            res('WARN', f'{a.depth_topic}: no images', 'the node then flies the published map only')
        else:
            _, enc, w, h = node.depth[-1]
            res('OK' if enc in ('16UC1', '32FC1') else 'FAIL', f'{a.depth_topic}: {len(node.depth) / span:.1f} Hz, {w}x{h} {enc}')
            res('OK' if node.info else 'WARN', f'camera_info: {node.info}' if node.info else 'no camera_info')
    # ---- clock
    res('OK', f"/clock: {'present (simulator: use_sim_time:=true)' if node.clock else 'absent (real time)'}")
    node.destroy_node()
    rclpy.try_shutdown()
    bad = results.count('FAIL')
    print(f'\n{bad} FAIL, {results.count("WARN")} WARN.', 'Fix the FAILs before flying.' if bad else 'Pipeline looks usable.')
    return 1 if bad else 0


if __name__ == '__main__':
    sys.exit(main())
