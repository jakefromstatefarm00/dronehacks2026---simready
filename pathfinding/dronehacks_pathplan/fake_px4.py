"""TEST ONLY: a fake PX4 + cuVSLAM, so the whole ROS pipeline runs with no simulator.

    ros2 launch dronehacks_pathplan selftest.launch.py        # uses this; never point it at a drone

Stands in for everything BELOW flight_interface: PX4 (via the topics the XRCE agent would
carry) and cuVSLAM. It publishes

    /fmu/out/vehicle_status(_vN), vehicle_land_detected, vehicle_local_position(_vN),
    vehicle_odometry          names resolved exactly as flight_interface resolves them
    /visual_slam/tracking/odometry   ENU, origin = where it "powered on" (the launch mark)
    /camera/depth/image_raw + camera_info   (depth:=true) ray-cast from the published course

and obeys /fmu/in/offboard_control_mode + trajectory_setpoint + vehicle_command like PX4:
Altitude/Position/Offboard mode switches (Offboard only with a live heartbeat), arming on the
ground, Land, Offboard loss -> Position, auto-disarm 2 s after landing and 10 s after arming
without taking off. The vehicle is a point with PX4-like position tracking (P gain, 1 m/s,
lag, 0.8 rad/s yaw). Not a flight model: it checks wiring, frames and the handshake.
"""
from __future__ import annotations

import json
import math
import time

import numpy as np
import rclpy
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.node import Node
from rclpy.qos import (QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy,
                       qos_profile_sensor_data)
from nav_msgs.msg import Odometry

ALTCTL, POSCTL, OFFBOARD, AUTO_LAND = 1, 2, 14, 18
DISARMED, ARMED = 1, 2


class FakePX4(Node):

    def __init__(self):
        super().__init__('fake_px4')
        d = ParameterDescriptor(dynamic_typing=True)
        self.declare_parameter('depth', False, d)
        self.declare_parameter('log_file', '/tmp/fake_px4.json', d)
        self.declare_parameter('odom_rate_hz', 30.0, d)
        import px4_msgs.msg as px4
        from skybeta_flight_interface.topics import resolve_endpoints
        self.px4, self.topics = px4, resolve_endpoints()
        self.pos, self.vel, self.yaw = np.zeros(3), np.zeros(3), 0.0      # ENU odom, facing +x
        self.arming, self.nav, self.landed = DISARMED, ALTCTL, True
        self.sp, self.last_heartbeat, self.armed_at, self.landed_since = None, -1e9, None, None
        self.log = {'commands': [], 'modes': [], 'max_alt': 0.0, 'path': []}
        out_qos = QoSProfile(reliability=QoSReliabilityPolicy.BEST_EFFORT,
                             durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
                             history=QoSHistoryPolicy.KEEP_LAST, depth=5)
        in_qos = QoSProfile(reliability=QoSReliabilityPolicy.BEST_EFFORT, durability=QoSDurabilityPolicy.VOLATILE,
                            history=QoSHistoryPolicy.KEEP_LAST, depth=10)
        T = self.topics
        self.pub = {
            'status': self.create_publisher(px4.VehicleStatus, T['vehicle_status'], out_qos),
            'land': self.create_publisher(px4.VehicleLandDetected, T['vehicle_land_detected'], out_qos),
            'local': self.create_publisher(px4.VehicleLocalPosition, T['vehicle_local_position'], out_qos),
            'odom': self.create_publisher(px4.VehicleOdometry, '/fmu/out/vehicle_odometry', out_qos),
        }
        self.vio_pub = self.create_publisher(Odometry, '/visual_slam/tracking/odometry', 10)
        self.create_subscription(px4.OffboardControlMode, T['offboard_control_mode'], self.on_heartbeat, in_qos)
        self.create_subscription(px4.TrajectorySetpoint, T['trajectory_setpoint'], self.on_setpoint, in_qos)
        self.create_subscription(px4.VehicleCommand, T['vehicle_command'], self.on_command, in_qos)
        self.dt = .02
        self.create_timer(self.dt, self.step)
        self.create_timer(1 / float(self.get_parameter('odom_rate_hz').value), self.publish_vio)
        self.slow = {'status': [.5, -1e9, None], 'land': [1.0, -1e9, None]}
        if str(self.get_parameter('depth').value).lower() in ('true', '1'):
            self.setup_depth()
        self.get_logger().warn('FAKE PX4 (test only): no simulator, no real autopilot.')

    # ------------------------------------------------------------------ PX4 inputs
    def on_heartbeat(self, msg):
        self.last_heartbeat = time.monotonic()
        self.heartbeat_position = bool(msg.position)

    def on_setpoint(self, msg):
        p = [float(v) for v in msg.position]
        if all(math.isfinite(v) for v in p):
            ned_yaw = float(msg.yaw)
            yaw = math.pi / 2 - ned_yaw if math.isfinite(ned_yaw) else None
            self.sp = (np.array([p[1], p[0], -p[2]]), yaw)               # NED -> ENU
        else:
            self.sp = None

    def on_command(self, msg):
        c, p1, p2 = int(msg.command), float(msg.param1), float(msg.param2)
        self.log['commands'].append({'t': round(time.monotonic(), 2), 'command': c, 'p1': p1, 'p2': p2,
                                     'nav': self.nav, 'arming': self.arming})
        if c == 176:
            if p2 == 2.0:
                self.set_nav(ALTCTL)
            elif p2 == 3.0:
                self.set_nav(POSCTL)
            elif p2 == 6.0 and time.monotonic() - self.last_heartbeat < .5:
                self.set_nav(OFFBOARD)
        elif c == 400:
            if p1 == 1.0 and self.landed and self.arming == DISARMED:
                self.arming, self.armed_at = ARMED, time.monotonic()
            elif p1 == 0.0 and self.landed:
                self.arming = DISARMED
        elif c == 21 and self.arming == ARMED:
            self.set_nav(AUTO_LAND)

    def set_nav(self, nav):
        if nav != self.nav:
            self.log['modes'].append({'t': round(time.monotonic(), 2), 'from': self.nav, 'to': nav})
            self.get_logger().info(f'nav_state {self.nav} -> {nav}')
        self.nav = nav

    # ------------------------------------------------------------------ the vehicle
    def step(self):
        now, dt = time.monotonic(), self.dt
        if self.nav == OFFBOARD and now - self.last_heartbeat > .5:
            self.set_nav(POSCTL)                                     # COM_OBL_RC_ACT: Position
        target_v = np.zeros(3)
        if self.arming == ARMED and self.nav == OFFBOARD and self.sp is not None:
            target_v = .95 * (self.sp[0] - self.pos)
            n = np.linalg.norm(target_v[:2])
            if n > 1.0:
                target_v[:2] /= n
            target_v[2] = float(np.clip(target_v[2], -.7, 1.0))
            if self.sp[1] is not None:
                turn = math.atan2(math.sin(self.sp[1] - self.yaw), math.cos(self.sp[1] - self.yaw))
                self.yaw += float(np.clip(turn, -.8 * dt, .8 * dt))
        elif self.arming == ARMED and self.nav == AUTO_LAND:
            target_v = np.array([0.0, 0.0, -.4])
        elif self.arming == ARMED and not self.landed:
            target_v = np.zeros(3)                                    # Position/Altitude: hover
        self.vel += (target_v - self.vel) * min(1.0, dt / .3)
        self.pos += self.vel * dt
        if self.pos[2] <= 0:
            self.pos[2], self.vel[2] = 0.0, max(0.0, self.vel[2])
        self.landed = bool(self.pos[2] < .02 and self.vel[2] <= .05)
        if self.arming == ARMED and self.landed:
            self.landed_since = self.landed_since or now
            if self.nav == AUTO_LAND and now - self.landed_since > 2.0:            # COM_DISARM_LAND
                self.arming = DISARMED
                self.set_nav(POSCTL)
            elif self.nav not in (OFFBOARD, AUTO_LAND) and now - (self.armed_at or now) > 10.0:   # COM_DISARM_PRFLT
                self.arming = DISARMED
        else:
            self.landed_since = None
        self.log['max_alt'] = max(self.log['max_alt'], float(self.pos[2]))
        if int(now / dt) % 25 == 0:
            self.log['path'].append([round(float(v), 3) for v in self.pos])
        self.publish_px4(now)

    def publish_px4(self, now):
        px4, us = self.px4, int(self.get_clock().now().nanoseconds / 1000)
        ned = [float(self.pos[1]), float(self.pos[0]), float(-self.pos[2])]
        yaw_ned = math.atan2(math.sin(math.pi / 2 - self.yaw), math.cos(math.pi / 2 - self.yaw))
        lp = px4.VehicleLocalPosition()
        lp.timestamp, lp.xy_valid, lp.z_valid = us, True, True
        lp.x, lp.y, lp.z, lp.heading = ned[0], ned[1], ned[2], yaw_ned
        self.pub['local'].publish(lp)
        o = px4.VehicleOdometry()
        o.timestamp = o.timestamp_sample = us
        o.pose_frame = 1
        o.position = ned
        o.q = [math.cos(yaw_ned / 2), 0.0, 0.0, math.sin(yaw_ned / 2)]
        self.pub['odom'].publish(o)
        if self.due('status', now, (self.arming, self.nav)):
            s = px4.VehicleStatus()
            s.timestamp, s.arming_state, s.nav_state = us, self.arming, self.nav
            self.pub['status'].publish(s)
        if self.due('land', now, self.landed):
            ld = px4.VehicleLandDetected()
            ld.timestamp, ld.landed = us, self.landed
            self.pub['land'].publish(ld)

    def due(self, key, now, value):
        period, last, old = self.slow[key]
        if now - last >= period or value != old:
            self.slow[key][1:] = [now, value]
            return True
        return False

    def publish_vio(self):
        o = Odometry()
        o.header.stamp = self.get_clock().now().to_msg()
        o.header.frame_id, o.child_frame_id = 'odom', 'base_link'
        o.pose.pose.position.x, o.pose.pose.position.y, o.pose.pose.position.z = (float(v) for v in self.pos)
        o.pose.pose.orientation.z, o.pose.pose.orientation.w = math.sin(self.yaw / 2), math.cos(self.yaw / 2)
        self.vio_pub.publish(o)

    # ------------------------------------------------------------------ fake depth camera
    K = (421.016, 0.0, 319.25, 0.0, 421.016, 200.66, 0.0, 0.0, 1.0)     # FlyX lab CameraInfo
    W, H = 640, 400

    def setup_depth(self):
        from sensor_msgs.msg import CameraInfo, Image
        from pathplan.ros_node import OdomFrame
        from pathplan.harness import load_published
        from pathplan.world import GridSpec, occupancy
        course, _ = load_published()
        self.spec = GridSpec()
        self.occ = occupancy(course, self.spec, altitude=.9, half_height=.05)
        self.frame = OdomFrame()                                     # powered on on the launch mark
        self.Image, self.CameraInfo = Image, CameraInfo
        self.depth_pub = self.create_publisher(Image, '/camera/depth/image_raw', qos_profile_sensor_data)
        self.info_pub = self.create_publisher(CameraInfo, '/camera/depth/camera_info', qos_profile_sensor_data)
        self.create_timer(1 / 13, self.publish_depth)

    def publish_depth(self):
        from dronehacks_pathplan.dry_run import render_depth
        stamp = self.get_clock().now().to_msg()                       # capture time
        xyz, yaw, _ = self.frame.odometry_to_course(self.pos, (0, 0, math.sin(self.yaw / 2), math.cos(self.yaw / 2)))
        depth = render_depth(self.occ, self.spec, xyz[:2], xyz[2], yaw, self.K, self.W, self.H)
        mm = np.where(np.isfinite(depth) & (depth < 10), depth * 1000, 0).astype('<u2')
        img = self.Image(height=self.H, width=self.W, encoding='16UC1', step=self.W * 2, data=mm.tobytes())
        img.header.stamp, img.header.frame_id = stamp, 'camera_depth_optical_frame'
        self.depth_pub.publish(img)
        info = self.CameraInfo(height=self.H, width=self.W, k=list(self.K))
        info.header = img.header
        self.info_pub.publish(info)


def main(args=None):
    rclpy.init(args=args)
    node = FakePX4()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.log['final'] = {'pos_enu': node.pos.tolist(), 'arming': node.arming, 'nav': node.nav,
                             'landed': node.landed}
        try:
            with open(str(node.get_parameter('log_file').value), 'w') as fh:
                json.dump(node.log, fh, indent=1)
        except OSError:
            pass
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
