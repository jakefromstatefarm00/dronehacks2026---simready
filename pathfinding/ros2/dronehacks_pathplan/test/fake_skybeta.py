"""TEST ONLY - fake skybeta flight interface + vehicle for wiring tests (no simulator).

    python3 test/fake_skybeta.py /tmp/fake_skybeta.json &
    ros2 run dronehacks_pathplan pathplan_mission

Publishes /visual_slam/tracking/odometry (ENU odom, origin = start) and /skybeta/state
(offboard_active turns true after ``offboard_after_s``, as if the pilot armed then switched
to Position). Moves a point toward /skybeta/setpoint/pose at <= 1 m/s once Offboard is
active, clamps z to the 0.3 m envelope floor, follows the commanded yaw. Not a flight model.
"""
import json
import math
import sys

import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import String

OUT = sys.argv[1] if len(sys.argv) > 1 else '/tmp/fake_skybeta.json'
OFFBOARD_AFTER_S = 3.0
MIN_ALT = 0.3


class FakeSkybeta(Node):
    def __init__(self):
        super().__init__('fake_skybeta')
        self.pos, self.yaw, self.sp, self.sp_yaw = np.zeros(3), 0.0, None, None
        self.t0 = self.t()
        self.log = {'setpoints': 0, 'waiting_setpoints': 0, 'max_alt': 0.0, 'path': [], 'clamped': 0}
        self.odom_pub = self.create_publisher(Odometry, '/visual_slam/tracking/odometry', 10)
        self.state_pub = self.create_publisher(String, '/skybeta/state', 10)
        self.create_subscription(PoseStamped, '/skybeta/setpoint/pose', self.on_sp, 10)
        self.dt = 1 / 30
        self.create_timer(self.dt, self.step)
        self.create_timer(.2, self.publish_state)

    def t(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def offboard(self):
        return self.t() - self.t0 > OFFBOARD_AFTER_S

    def on_sp(self, msg):
        p, q = msg.pose.position, msg.pose.orientation
        z = float(p.z)
        if z < MIN_ALT:
            z, self.log['clamped'] = MIN_ALT, self.log['clamped'] + 1
        self.sp = np.array([p.x, p.y, z], float)
        self.sp_yaw = None if (q.x, q.y, q.z, q.w) == (0, 0, 0, 0) else 2 * math.atan2(q.z, q.w)
        self.log['setpoints' if self.offboard() else 'waiting_setpoints'] += 1

    def step(self):
        if self.offboard() and self.sp is not None:
            d = self.sp - self.pos
            n = np.linalg.norm(d)
            self.pos += d if n < self.dt else d / n * self.dt
            if self.sp_yaw is not None:
                self.yaw = self.sp_yaw
        self.log['max_alt'] = max(self.log['max_alt'], float(self.pos[2]))
        if int((self.t() - self.t0) / self.dt) % 15 == 0:
            self.log['path'].append([round(float(v), 3) for v in self.pos])
        o = Odometry()
        o.header.stamp = self.get_clock().now().to_msg()
        o.header.frame_id, o.child_frame_id = 'odom', 'base_link'
        o.pose.pose.position.x, o.pose.pose.position.y, o.pose.pose.position.z = map(float, self.pos)
        o.pose.pose.orientation.z, o.pose.pose.orientation.w = math.sin(self.yaw / 2), math.cos(self.yaw / 2)
        self.odom_pub.publish(o)

    def publish_state(self):
        self.state_pub.publish(String(data=json.dumps({'offboard_active': self.offboard(), 'clamped': []})))


def main():
    rclpy.init()
    node = FakeSkybeta()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.log['final'] = {'pos': node.pos.tolist(), 'yaw': node.yaw}
        json.dump(node.log, open(OUT, 'w'), indent=1)
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
