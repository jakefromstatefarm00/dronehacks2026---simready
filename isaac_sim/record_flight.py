"""Flight recorder for the organizer sim profile (evaluation only; publishes nothing).

Writes into --output:
  truth.csv      Pegasus ground truth /drone1/state/pose (ENU, Isaac world), 20 Hz
  estimate.csv   cuVSLAM /visual_slam/tracking/odometry (odom frame), 20 Hz
  setpoint.csv   /skybeta/setpoint/pose (what the competitor commanded), all
  events.jsonl   /skybeta/state changes, PX4 arming/nav_state changes, mission status
                 changes (/dronehacks_pathplan/status), cuVSLAM vo_state changes
  frames/        onboard left camera JPEGs every --frame-dt s (for the video)
Times are wall-clock seconds since recorder start (t) plus message stamps (stamp).
"""
import argparse
import json
import math
import time
from pathlib import Path


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('--output', type=Path, required=True)
    p.add_argument('--frame-dt', type=float, default=.1)
    a = p.parse_args()
    a.output.mkdir(parents=True, exist_ok=True)
    (a.output/'frames').mkdir(exist_ok=True)

    import cv2
    import numpy as np
    import rclpy
    from rclpy.qos import qos_profile_sensor_data, QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy, QoSHistoryPolicy
    from geometry_msgs.msg import PoseStamped
    from nav_msgs.msg import Odometry
    from sensor_msgs.msg import Image
    from std_msgs.msg import String
    from px4_msgs.msg import VehicleStatus
    from isaac_ros_visual_slam_interfaces.msg import VisualSlamStatus
    from skybeta_flight_interface.topics import fmu_topic

    rclpy.init()
    node = rclpy.create_node('organizer_sim_recorder')
    t0 = time.monotonic()
    files = {k: open(a.output/f'{k}.csv', 'w') for k in ('truth', 'estimate', 'setpoint')}
    for f in files.values():
        f.write('t,stamp,x,y,z,qx,qy,qz,qw\n')
    events = open(a.output/'events.jsonl', 'w')
    last = {'truth': -1., 'estimate': -1., 'frame': -1., 'frames': 0}
    seen = {}

    def now():
        return time.monotonic()-t0

    def stamp(h):
        return h.stamp.sec+h.stamp.nanosec*1e-9

    def row(kind, h, pose):
        q, pp = pose.orientation, pose.position
        files[kind].write(f'{now():.3f},{stamp(h):.3f},{pp.x:.4f},{pp.y:.4f},{pp.z:.4f},{q.x:.5f},{q.y:.5f},{q.z:.5f},{q.w:.5f}\n')

    def event(kind, **kw):
        events.write(json.dumps({'t': round(now(), 3), 'kind': kind, **kw})+'\n')
        events.flush()

    def changed(key, value, **kw):
        if seen.get(key) != value:
            seen[key] = value
            event(key, value=value, **kw)

    def on_truth(m):
        if now()-last['truth'] >= .05:
            last['truth'] = now(); row('truth', m.header, m.pose)

    def on_est(m):
        if now()-last['estimate'] >= .05:
            last['estimate'] = now(); row('estimate', m.header, m.pose.pose)

    def on_sp(m):
        row('setpoint', m.header, m.pose)

    def on_state(m):
        try:
            s = json.loads(m.data)
        except ValueError:
            return
        changed('skybeta_state', {k: s.get(k) for k in ('armed', 'landed', 'nav_state_name', 'offboard_active')},
                clamped=s.get('clamped'))

    def on_mission(m):
        try:
            s = json.loads(m.data)
        except ValueError:
            return
        changed('mission_status', {k: s.get(k) for k in ('state', 'next', 'cleared')})

    def on_status(m):
        changed('px4_arming_state', int(m.arming_state))
        changed('px4_nav_state', int(m.nav_state))

    def on_vo(m):
        changed('vo_state', int(m.vo_state))

    def on_image(m):
        if now()-last['frame'] < a.frame_dt:
            return
        last['frame'] = now()
        img = np.frombuffer(m.data, np.uint8).reshape(m.height, m.step)[:, :m.width*3].reshape(m.height, m.width, 3)
        img = cv2.resize(cv2.cvtColor(img, cv2.COLOR_RGB2BGR), (m.width//2, m.height//2))
        cv2.imwrite(str(a.output/'frames'/f'{now():09.3f}.jpg'), img, [cv2.IMWRITE_JPEG_QUALITY, 85])
        last['frames'] += 1

    px4_qos = QoSProfile(reliability=QoSReliabilityPolicy.BEST_EFFORT, durability=QoSDurabilityPolicy.VOLATILE,
                         history=QoSHistoryPolicy.KEEP_LAST, depth=1)
    node.create_subscription(PoseStamped, '/drone1/state/pose', on_truth, qos_profile_sensor_data)
    node.create_subscription(Odometry, '/visual_slam/tracking/odometry', on_est, 10)
    node.create_subscription(PoseStamped, '/skybeta/setpoint/pose', on_sp, 10)
    node.create_subscription(String, '/skybeta/state', on_state, 10)
    node.create_subscription(String, '/dronehacks_pathplan/status', on_mission, 10)
    node.create_subscription(VehicleStatus, fmu_topic('out', 'vehicle_status', 'VehicleStatus'), on_status, px4_qos)
    node.create_subscription(VisualSlamStatus, '/visual_slam/status', on_vo, 10)
    node.create_subscription(Image, '/drone1/left/color/image_raw', on_image, qos_profile_sensor_data)
    node.create_timer(1., lambda: [f.flush() for f in files.values()])
    event('recorder_start')
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, Exception):
        pass
    finally:
        event('recorder_stop', frames=last['frames'])
        for f in files.values():
            f.close()
        events.close()


if __name__ == '__main__':
    main()
