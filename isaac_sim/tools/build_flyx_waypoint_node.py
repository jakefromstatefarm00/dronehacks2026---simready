"""Build a FlyX single-file node from machmoon/dronehacks-sim's waypoint controller.

    python tools/build_flyx_waypoint_node.py <machmoon-repo> <out.py> [--course mission/course-window-fast.json]

The generated node embeds the repo's mission/core.py (CourseController) and
mission/frames.py (CourseFrame, rotate_body_velocity) verbatim, plus the route JSON,
and speaks only the organizer skybeta contract:

    subscribe  /visual_slam/tracking/odometry  (cuVSLAM, ENU odom, twist in body FLU)
    subscribe  /skybeta/state                   (offboard_active)
    publish    /skybeta/setpoint/twist          (ENU odom velocity + yaw rate, 30 Hz)
    publish    /skybeta/setpoint/pose           (hold at the landing handoff / on a fault)

Where the Gazebo adapter (mission/run.py) read ground-truth odometry in the course
frame, this node maps cuVSLAM's odom into the course frame from the pose at the
moment Offboard starts: the drone is on the launch mark facing course +y (rule:
start facing up the first corridor), so course +X points at odom yaw - pi/2.
Never arms, never changes mode, never touches /fmu/*.
"""
import argparse
import json
from pathlib import Path

NODE = r'''
class WaypointMission(Node):
    def __init__(self):
        super().__init__('dronehacks_waypoints')
        p = self.declare_parameter
        p('speed_scale', 1.0)
        p('launch_heading_course_rad', math.pi / 2)
        p('rate_hz', 30.0)
        g = lambda n: self.get_parameter(n).value
        self.config = COURSE
        self.ctrl = core.CourseController(self.config, speed_scale=float(g('speed_scale')))
        self.launch_heading = float(g('launch_heading_course_rad'))
        self.frame = None
        self.odom = None
        self.odom_t = None
        self.state = {}
        self.state_t = None
        self.hold = None
        self.twist_pub = self.create_publisher(TwistStamped, '/skybeta/setpoint/twist', 10)
        self.pose_pub = self.create_publisher(PoseStamped, '/skybeta/setpoint/pose', 10)
        self.status_pub = self.create_publisher(String, '~/status', 10)
        self.create_subscription(Odometry, '/visual_slam/tracking/odometry', self.on_odom, 10)
        self.create_subscription(String, '/skybeta/state', self.on_state, 10)
        self.create_timer(1.0 / float(g('rate_hz')), self.tick)
        self.get_logger().info(f'dronehacks_waypoints ready: {len(self.ctrl.waypoints)} waypoints '
                               f'({self.config.get("name")}), speed scale {g("speed_scale")}. Waiting for Offboard...')

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def on_odom(self, msg):
        self.odom, self.odom_t = msg, self.now()

    def on_state(self, msg):
        try:
            self.state = json.loads(msg.data)
            self.state_t = self.now()
        except ValueError:
            pass

    def send_pose(self, p_odom):
        m = PoseStamped()
        m.header.stamp = self.get_clock().now().to_msg()
        m.header.frame_id = 'odom'
        m.pose.position.x, m.pose.position.y, m.pose.position.z = (float(v) for v in p_odom)
        m.pose.orientation.w = 0.0                      # all-zero quaternion: do not command yaw
        self.pose_pub.publish(m)

    def tick(self):
        if self.odom is None or self.state_t is None:
            return
        pp, q = self.odom.pose.pose.position, self.odom.pose.pose.orientation
        p_odom = (pp.x, pp.y, pp.z)
        yaw_odom = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        offboard = bool(self.state.get('offboard_active'))
        if self.hold is not None:
            self.send_pose(self.hold)
            return
        if self.frame is None:
            if not offboard:
                self.send_pose(p_odom)                  # stream "stay here" so Offboard can start
                return
            # course +X in odom = vehicle heading (course launch heading) rotated back
            self.frame = frames.CourseFrame(self.config['start'], p_odom, yaw_odom - self.launch_heading)
            self.get_logger().info(f'Offboard active: course frame anchored at odom {tuple(round(v, 3) for v in p_odom)}, '
                                   f'yaw offset {self.frame.yaw_offset:.3f} rad. Flying the route.')
        tw = self.odom.twist.twist.linear
        v_odom = frames.rotate_body_velocity((q.x, q.y, q.z, q.w), (tw.x, tw.y, tw.z))
        now = self.now()
        cmd = self.ctrl.update(self.frame.position_to_course(p_odom), self.frame.velocity_to_course(v_odom), now,
                               offboard=offboard, odom_age=now - self.odom_t, state_age=now - self.state_t,
                               yaw=core.wrap(yaw_odom - self.frame.yaw_offset))
        self.status_pub.publish(String(data=json.dumps({'phase': cmd.phase, 'done': cmd.done, 'fault': cmd.fault,
                                                        'waypoint': self.ctrl.waypoint_index,
                                                        'of': len(self.ctrl.waypoints)})))
        if cmd.fault:
            self.hold = p_odom
            self.get_logger().error(f'controller fault {cmd.fault} -- holding here.')
            return
        if cmd.done and math.dist(self.frame.position_to_course(p_odom), self.ctrl.handoff_target) <= self.ctrl.arrival_radius:
            self.hold = self.frame.position_to_odom(self.ctrl.handoff_target)
            self.get_logger().info('landing pad reached -- holding over it; the pilot lands.')
            return
        m = TwistStamped()
        m.header.stamp = self.get_clock().now().to_msg()
        m.header.frame_id = 'odom'
        m.twist.linear.x, m.twist.linear.y, m.twist.linear.z = self.frame.velocity_to_odom(cmd.velocity)
        m.twist.angular.z = float(cmd.yaw_rate)
        self.twist_pub.publish(m)


def main(args=None):
    rclpy.init(args=args)
    node = WaypointMission()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
'''


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('repo', type=Path)
    p.add_argument('output', type=Path)
    p.add_argument('--course', default='mission/course-window-fast.json')
    a = p.parse_args()
    import subprocess
    commit = subprocess.run(['git', '-C', str(a.repo), 'rev-parse', '--short', 'HEAD'], capture_output=True,
                            text=True).stdout.strip()
    core = (a.repo/'mission/core.py').read_text()
    frames = (a.repo/'mission/frames.py').read_text().replace('from .core import vector', 'vector = core.vector')
    course = json.loads((a.repo/a.course).read_text())
    head = f'''"""FlyX single file: machmoon/dronehacks-sim waypoint controller as a skybeta competitor node.

Generated by tools/build_flyx_waypoint_node.py from machmoon/dronehacks-sim@{commit}
({a.course}); mission/core.py and mission/frames.py are embedded verbatim. Do not edit;
rebuild. Needs only rclpy and the standard message packages.

    python3 waypoint_mission.py [--ros-args -p speed_scale:=1.0]

Start ON THE LAUNCH MARK facing up the first corridor. Never arms or changes mode;
ends holding over the landing pad (the pilot lands).
"""
import json
import math
import types

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, TwistStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import String

core = types.ModuleType('core')
exec(compile({core!r}, 'mission/core.py', 'exec'), core.__dict__)
frames = types.ModuleType('frames')
frames.core = core
exec(compile({frames!r}, 'mission/frames.py', 'exec'), frames.__dict__)
COURSE = json.loads({json.dumps(course)!r})
'''
    a.output.parent.mkdir(parents=True, exist_ok=True)
    a.output.write_text(head+NODE, encoding='utf-8', newline='\n')
    print('wrote', a.output, f'({len(course["waypoints"])} waypoints, {course["name"]})')


if __name__ == '__main__':
    main()
