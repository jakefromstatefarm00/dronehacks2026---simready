#!/usr/bin/env python3
"""SIMULATION ONLY. Stand in for cuVSLAM: PX4's own odometry -> /visual_slam/tracking/odometry.

On the drone, cuVSLAM produces the odometry and px4_odom_bridge feeds it to PX4. In a simulator
without cuVSLAM, this node goes the other way: it republishes PX4's estimate (NED/FRD) as the
nav_msgs/Odometry (ENU/FLU, child frame base_link, body-frame twist) that flight_interface and
team_mission expect. Because it comes from PX4's own frame, flight_interface's plain ENU->NED
setpoint conversion is exactly right, so this is the frame-safe way to test the mission logic.

Frame note: the odom frame here is PX4's local frame turned to ENU (x = east, y = north of the
simulator world), NOT "nose direction at boot" as with cuVSLAM. For course checkpoints set
launch_yaw_deg = 0 and launch_x/launch_y = the spawn point in the world.

    ros2 run team_mission sim_odom_adapter --ros-args -p use_sim_time:=true
"""
import math

import numpy as np
import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

from team_mission.sim import refuse_on_drone

S = math.sqrt(0.5)
Q_ENU_FROM_NED = (S, S, 0.0, 0.0)        # (x, y, z, w): 180 deg about (1, 1, 0)/sqrt2
Q_FRD_FROM_FLU = (1.0, 0.0, 0.0, 0.0)    # 180 deg about x


def qmul(a, b):
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (aw * bx + ax * bw + ay * bz - az * by, aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw, aw * bw - ax * bx - ay * by - az * bz)


def rot(q, v):
    x, y, z, w = q
    u = np.array([x, y, z])
    v = np.asarray(v, float)
    return 2 * u.dot(v) * u + (w * w - u.dot(u)) * v + 2 * w * np.cross(u, v)


class SimOdomAdapter(Node):
    def __init__(self):
        super().__init__('sim_odom_adapter')
        self.declare_parameter('out_topic', '/visual_slam/tracking/odometry')
        self.declare_parameter('rate_hz', 30.0)
        self.declare_parameter('allow_on_drone', False)
        refuse_on_drone('sim_odom_adapter', bool(self.get_parameter('allow_on_drone').value))
        from px4_msgs.msg import VehicleOdometry
        try:
            from skybeta_flight_interface.topics import fmu_topic
            topic = fmu_topic('out', 'vehicle_odometry', 'VehicleOdometry')
        except Exception:
            topic = '/fmu/out/vehicle_odometry'
        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                         history=HistoryPolicy.KEEP_LAST, depth=5)
        self.pub = self.create_publisher(Odometry, self.get_parameter('out_topic').value, 10)
        self.create_subscription(VehicleOdometry, topic, self.on_px4, qos)
        self.min_dt = 1.0 / float(self.get_parameter('rate_hz').value)
        self.last = 0.0
        self.get_logger().warn(f'SIMULATION ONLY: {topic} -> {self.get_parameter("out_topic").value}')

    def on_px4(self, m):
        now = self.get_clock().now()
        t = now.nanoseconds * 1e-9
        if t - self.last < self.min_dt:
            return
        self.last = t
        p = np.array(m.position, float)
        if not np.all(np.isfinite(p)):
            return
        q_ned_frd = (float(m.q[1]), float(m.q[2]), float(m.q[3]), float(m.q[0]))   # px4 is w, x, y, z
        q_enu_flu = qmul(qmul(Q_ENU_FROM_NED, q_ned_frd), Q_FRD_FROM_FLU)
        v_ned = np.array(m.velocity, float)
        if m.velocity_frame == 3:      # VELOCITY_FRAME_BODY_FRD
            v_ned = rot(q_ned_frd, v_ned)
        v_enu = np.array([v_ned[1], v_ned[0], -v_ned[2]])
        q_inv = (-q_enu_flu[0], -q_enu_flu[1], -q_enu_flu[2], q_enu_flu[3])
        v_body = rot(q_inv, v_enu)
        o = Odometry()
        o.header.stamp = now.to_msg()
        o.header.frame_id, o.child_frame_id = 'odom', 'base_link'
        o.pose.pose.position.x, o.pose.pose.position.y, o.pose.pose.position.z = float(p[1]), float(p[0]), float(-p[2])
        oo = o.pose.pose.orientation
        oo.x, oo.y, oo.z, oo.w = (float(c) for c in q_enu_flu)
        o.twist.twist.linear.x, o.twist.twist.linear.y, o.twist.twist.linear.z = (float(c) for c in v_body)
        cov = [0.0] * 36
        for k in (0, 7, 14):
            cov[k] = 1e-4
        for k in (21, 28, 35):
            cov[k] = 1e-4
        o.pose.covariance = cov
        self.pub.publish(o)


def main(args=None):
    rclpy.init(args=args)
    n = SimOdomAdapter()
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    finally:
        n.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
