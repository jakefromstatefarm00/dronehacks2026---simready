#!/usr/bin/env python3
"""team_mission: our replacement for competitor_example/square_mission.

Talks ONLY to the same topics the example does (plus two read-only health topics):

  IN   /skybeta/state                    std_msgs/String JSON  armed / offboard_active / landed (flight_interface)
  IN   /visual_slam/tracking/odometry    nav_msgs/Odometry     where we are (cuVSLAM, odom ENU)
  IN   /visual_slam/status               VisualSlamStatus      vo_state (1 = tracking)          [optional]
  IN   extra_health_topic                std_msgs/Bool         e.g. /team/localization/ok        [optional]
  OUT  /skybeta/setpoint/pose            geometry_msgs/PoseStamped   hold / takeoff / carrot setpoints
  OUT  /skybeta/setpoint/twist           geometry_msgs/TwistStamped  touchdown only (slow descent)
  OUT  /team_mission/status              std_msgs/String JSON  phase, checkpoint, health (2 Hz)

It never arms, never changes PX4 mode and never publishes to /fmu/in/*: flight_interface owns
all of that. The mission logic lives in team_mission/mission.py (ROS-free, unit-tested).
"""
from __future__ import annotations

import json
import math
import os

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped, TwistStamped
from nav_msgs.msg import Odometry
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.node import Node
from std_msgs.msg import Bool, String

from team_mission.control import Limits, load_policy
from team_mission.flight_log import FlightLogger
from team_mission.geometry import CourseFrame, quat_from_yaw, rotate_by_quat, yaw_from_quat
from team_mission.mission import DORMANT, Inputs, Mission, MissionParams
from team_mission.path import CheckpointSpec

try:
    from isaac_ros_visual_slam_interfaces.msg import VisualSlamStatus
except ImportError:  # laptop / simulator without Isaac ROS
    VisualSlamStatus = None


def share_dir() -> str:
    try:
        from ament_index_python.packages import get_package_share_directory
        return get_package_share_directory('team_mission')
    except Exception:
        return os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def resolve_file(name: str, subdir: str) -> str:
    """Absolute path as is; otherwise look in the package's share/<subdir>/."""
    if not name or os.path.isabs(name) or os.path.exists(name):
        return name
    cand = os.path.join(share_dir(), subdir, name)
    return cand if os.path.exists(cand) else name


class MissionNode(Node):
    def __init__(self):
        super().__init__('team_mission')
        dyn = ParameterDescriptor(dynamic_typing=True)
        P = lambda n, v: self.declare_parameter(n, v, dyn)  # noqa: E731
        P('checkpoints', 'test_line.yaml')
        P('policy', 'carrot')
        P('launch_x', 0.0)
        P('launch_y', 0.0)
        P('launch_z', 0.0)
        P('launch_yaw_deg', 0.0)
        P('odom_topic', '/visual_slam/tracking/odometry')
        P('status_topic', '/visual_slam/status')
        P('state_topic', '/skybeta/state')
        P('pose_topic', '/skybeta/setpoint/pose')
        P('twist_topic', '/skybeta/setpoint/twist')
        P('extra_health_topic', '')
        P('rate_hz', 20.0)
        P('log_dir', '')
        P('state_timeout_s', 1.0)
        for k, v in Limits().__dict__.items():
            P(k, v)
        for k, v in MissionParams().__dict__.items():
            P(k, v)
        g = lambda n: self.get_parameter(n).value  # noqa: E731

        for t in ('pose_topic', 'twist_topic'):
            if str(g(t)).startswith('/fmu/'):
                raise RuntimeError(f'{t}={g(t)}: this node must never publish to PX4 directly')

        limits = Limits(**{k: float(g(k)) for k in Limits().__dict__})
        mp = MissionParams(**{k: type(v)(g(k)) if not isinstance(v, bool) else
                              (g(k) if isinstance(g(k), bool) else str(g(k)).lower() == 'true')
                              for k, v in MissionParams().__dict__.items()})
        cp_file = resolve_file(str(g('checkpoints')), os.path.join('config', 'checkpoints'))
        spec = CheckpointSpec.load(cp_file)
        policy_spec = str(g('policy'))
        policy = load_policy(resolve_file(policy_spec, 'policies') if policy_spec not in ('carrot', 'baseline', '') else policy_spec)
        pl = getattr(policy, 'limits', None)
        if pl is not None and pl.__dict__ != limits.__dict__:
            self.get_logger().warn(f'policy was trained with limits {pl.__dict__}, flying with {limits.__dict__}')
        self.course_frame = CourseFrame(float(g('launch_x')), float(g('launch_y')), float(g('launch_z')),
                                        math.radians(float(g('launch_yaw_deg'))))
        logf = {'info': self.get_logger().info, 'warn': self.get_logger().warn, 'error': self.get_logger().error}
        self.mission = Mission(spec, policy, limits, mp, self.course_frame, lambda lv, m: logf[lv](m))
        self.meta = dict(checkpoints=cp_file, frame=spec.frame, policy=policy_spec,
                         policy_meta=getattr(policy, 'meta', {'name': 'carrot'}), limits=limits.__dict__,
                         mission=mp.__dict__, course_frame=self.course_frame.__dict__)
        self.logger = FlightLogger(str(g('log_dir')))
        self.state_timeout = float(g('state_timeout_s'))

        # latest inputs
        self.state, self.state_t = {}, None
        self.pos = self.vel = None
        self.yaw = None
        self.odom_t = None
        self.vo_state = None
        self.extra_ok = None

        self.pub_pose = self.create_publisher(PoseStamped, g('pose_topic'), 10)
        self.pub_twist = self.create_publisher(TwistStamped, g('twist_topic'), 10)
        self.pub_status = self.create_publisher(String, '/team_mission/status', 10)
        self.create_subscription(String, g('state_topic'), self.on_state, 10)
        self.create_subscription(Odometry, g('odom_topic'), self.on_odom, 10)
        if VisualSlamStatus is not None and g('status_topic'):
            self.create_subscription(VisualSlamStatus, g('status_topic'), self.on_status, 10)
        elif mp.require_vslam_status:
            self.get_logger().error('require_vslam_status is true but isaac_ros_visual_slam_interfaces is not '
                                    'installed: the mission will never consider tracking healthy. '
                                    'Set require_vslam_status:=false in simulation.')
        if g('extra_health_topic'):
            self.create_subscription(Bool, g('extra_health_topic'), self.on_extra, 10)
        self.create_timer(1.0 / float(g('rate_hz')), self.tick)
        self.create_timer(0.5, self.publish_status)
        self.last_out = None
        self.mission_events_snapshot = []
        self.get_logger().info(
            f'team_mission up: {len(spec.checkpoints)} checkpoints ({spec.frame} frame) from {cp_file}; '
            f'policy {policy_spec}; max {limits.max_xy_speed} m/s; takeoff {mp.takeoff_height} m. '
            'DORMANT until the pilot arms.')

    # ------------------------------------------------------------------ inputs
    def now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def on_state(self, msg):
        try:
            self.state = json.loads(msg.data)
            self.state_t = self.now()
        except ValueError:
            pass

    def on_odom(self, msg):
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        quat = (q.x, q.y, q.z, q.w)
        pos = np.array([p.x, p.y, p.z], float)
        if not np.all(np.isfinite(pos)):
            return
        lin = msg.twist.twist.linear
        self.pos = pos
        self.yaw = yaw_from_quat(quat)
        self.vel = rotate_by_quat(quat, (lin.x, lin.y, lin.z))   # body (child frame) -> world
        self.odom_t = self.now()

    def on_status(self, msg):
        self.vo_state = int(msg.vo_state)

    def on_extra(self, msg):
        self.extra_ok = bool(msg.data)

    # ------------------------------------------------------------------ loop
    def tick(self):
        now = self.now()
        st = self.state if (self.state_t is not None and now - self.state_t < self.state_timeout) else {}
        s = Inputs(now=now, armed=bool(st.get('armed', False)), offboard_active=bool(st.get('offboard_active', False)),
                   landed=st.get('landed'), nav_state_name=str(st.get('nav_state_name', '')),
                   pos=self.pos, vel=self.vel, yaw=self.yaw,
                   odom_age=1e9 if self.odom_t is None else now - self.odom_t,
                   vo_state=self.vo_state, extra_ok=self.extra_ok)
        out = self.mission.step(s)
        self.last_out = out
        if self.mission.phase != DORMANT and not self.logger.open:
            path = self.logger.start(now, self.meta)
            self.get_logger().info(f'logging this flight to {path}')
        if self.logger.open:
            if self.mission.path is not None and 'path_odom' not in self.logger.meta:
                P = self.mission.path
                self.logger.update_meta(path_odom=P.points, names=P.names, radius=P.radius, corridor=P.corridor,
                                        start=self.mission.start, start_yaw=self.mission.start_yaw)
            self.logger.row(now, s, out)
            if self.mission.phase == DORMANT:
                self.logger.update_meta(events=self.mission_events_snapshot)
                self.logger.close()
        self.mission_events_snapshot = list(self.mission.events)
        self.publish(out)

    def publish(self, out):
        stamp = self.get_clock().now().to_msg()
        if out.kind == 'pose' and out.position is not None:
            m = PoseStamped()
            m.header.stamp, m.header.frame_id = stamp, 'odom'
            m.pose.position.x, m.pose.position.y, m.pose.position.z = (float(v) for v in out.position)
            if out.yaw is not None:  # all-zero quaternion = "do not command yaw" (flight_interface)
                o = m.pose.orientation
                o.x, o.y, o.z, o.w = quat_from_yaw(float(out.yaw))
            else:
                o = m.pose.orientation
                o.x = o.y = o.z = o.w = 0.0
            self.pub_pose.publish(m)
        elif out.kind == 'twist' and out.velocity is not None:
            m = TwistStamped()
            m.header.stamp, m.header.frame_id = stamp, 'odom'
            m.twist.linear.x, m.twist.linear.y, m.twist.linear.z = (float(v) for v in out.velocity)
            m.twist.angular.z = float(out.yaw_rate)
            self.pub_twist.publish(m)

    def publish_status(self):
        m = self.mission
        d = dict(phase=m.phase, checkpoint=m.i, n_checkpoints=(m.path.n if m.path else None),
                 next=(m.path.names[m.i] if m.path and m.i <= m.path.n else None),
                 hold_reason=m.hold_reason if m.phase == 'HOLD' else '', holds=m.holds,
                 log=self.logger.path if self.logger.open else '')
        if m.path is not None and self.pos is not None:
            d['distance_to_next_m'] = round(float(np.linalg.norm(m.path.points[min(m.i, m.path.n)] - self.pos)), 3)
            d['position_course'] = [round(float(v), 3) for v in self.course_frame.odom_to_course(self.pos)]
        self.pub_status.publish(String(data=json.dumps(d)))


def main(args=None):
    rclpy.init(args=args)
    node = MissionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.logger.close()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
