"""ROS 2 node: depth + VIO odometry -> OnlinePlanner -> skybeta pose setpoints.

Follows the organisers' skybeta contract (github.com/Artash-N/skybeta,
docs/competitor_guide.md): it subscribes to ``/visual_slam/tracking/odometry``
and ``/skybeta/state`` and publishes ``/skybeta/setpoint/pose``.  It never touches
``/fmu/*``, never arms, never changes mode: ``skybeta_flight_interface`` owns all of
that and clamps every setpoint to its safety envelope.

NOT YET RUN on a drone or simulator.  Tested here without ROS: frame round trips,
command -> target, depth -> map on the real course prior.

    # alongside flight_interface (competitor.launch.py competitor:=false)
    python3 -m pathplan.ros_node --ros-args -p depth_topic:=<depth image> \
        -p camera_info_topic:=<its CameraInfo> -p launch_placement_reviewed:=true

Modes
-----
shadow (default, ``publish_setpoints:=false``)
    Maps and plans exactly as in flight; publishes ``~/path`` (frame ``odom``) and
    ``~/status`` only.  Safe beside any other mission.
setpoints (``publish_setpoints:=true``)
    Publishes ``/skybeta/setpoint/pose`` at ``setpoint_hz``: the current position
    while waiting (Offboard needs a recent team setpoint), then the plan once
    ``/skybeta/state`` reports ``offboard_active``; holds position if Offboard ends.

Depth: the stock skybeta camera config publishes NO depth (stereo images for
cuVSLAM only).  Point ``depth_topic`` at a depth source; with none the node plans on
the published map alone (equivalent to the benchmark's ``static`` planner).

Frames: ``odom`` is cuVSLAM's ENU frame -- origin and axes = the vehicle's pose when
SLAM started (x = forward, y = left, z = up).  The drone starts on the launch mark,
so course = launch_xy + Rz(launch_heading) @ odom_xy, z_course = z_odom + launch height.
"""
from __future__ import annotations

from dataclasses import dataclass
import json
from math import atan2, cos, sin
import sys
import threading
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]


# -------------------------------------------------------------- pure helpers
def _rz(a):
    return np.array([[cos(a), -sin(a), 0.0], [sin(a), cos(a), 0.0], [0.0, 0.0, 1.0]])


def quat_to_matrix(x, y, z, w):
    """Rotation matrix of a (x, y, z, w) quaternion (ROS order)."""
    n = (x * x + y * y + z * z + w * w) ** .5
    if n < 1e-9:
        raise ValueError("zero quaternion")
    x, y, z, w = x / n, y / n, z / n, w / n
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


@dataclass(frozen=True)
class OdomFrame:
    """cuVSLAM ``odom`` (ENU, origin = startup pose on the launch mark) <-> course frame."""
    launch_xy: tuple = (-2.515, .914)       # launch mark, course frame (m)
    launch_heading: float = np.pi / 2       # vehicle heading at startup, course frame (rad)
    launch_height: float = .05              # body origin above the floor at startup (m)

    def to_course(self, p_odom):
        p = np.asarray(p_odom, float)
        xy = np.asarray(self.launch_xy) + _rz(self.launch_heading)[:2, :2] @ p[:2]
        return np.array([xy[0], xy[1], p[2] + self.launch_height])

    def to_odom(self, p_course):
        p = np.asarray(p_course, float)
        xy = _rz(-self.launch_heading)[:2, :2] @ (p[:2] - np.asarray(self.launch_xy))
        return np.array([xy[0], xy[1], p[2] - self.launch_height])

    def yaw_to_course(self, yaw_odom):
        return float(np.arctan2(np.sin(yaw_odom + self.launch_heading), np.cos(yaw_odom + self.launch_heading)))

    def yaw_to_odom(self, yaw_course):
        return float(np.arctan2(np.sin(yaw_course - self.launch_heading), np.cos(yaw_course - self.launch_heading)))

    def odometry_to_course(self, position, quat_xyzw):
        """nav_msgs/Odometry pose -> (course xyz, course yaw, body-FLU -> course rotation)."""
        r_odom = quat_to_matrix(*quat_xyzw)
        rotation = _rz(self.launch_heading) @ r_odom
        yaw = atan2(rotation[1, 0], rotation[0, 0])
        return self.to_course(position), yaw, rotation


def target_from_command(cmd, pos, yaw, carrot_m):
    """Where to send the vehicle for one command: (xy, yaw) in the course frame.

    move: a point ``carrot_m`` along the path (pose setpoints are not speed-limited,
    so never send a far-away point); turn / done / failed: hold the current position,
    turning to cmd.yaw if given.
    """
    from .online import _point_along
    if cmd.mode == "move" and cmd.path is not None and len(cmd.path) >= 2:
        xy = _point_along(cmd.path, carrot_m)
        face = cmd.yaw if cmd.yaw is not None else atan2(xy[1] - pos[1], xy[0] - pos[0])
        return np.asarray(xy, float), float(face)
    return np.asarray(pos, float), float(cmd.yaw if cmd.yaw is not None else yaw)


def parse_state(text):
    """``/skybeta/state`` JSON -> dict (empty on bad input)."""
    try:
        state = json.loads(text)
    except ValueError:
        return {}
    return state if isinstance(state, dict) else {}


def build_planner(planner_name="theta", altitude=.9, chicane="window"):
    """Published-course prior + checkpoints -> OnlinePlanner (same setup as the benchmark)."""
    from .harness import load_published
    from .online import OnlinePlanner, PlannerConfig
    from .world import Envelope, GridSpec, checkpoints_from_route, occupancy
    course, route = load_published()
    spec, env = GridSpec(), Envelope()
    prior = occupancy(course, spec, altitude=altitude, half_height=env.half_height + env.tracking_margin)
    cps = checkpoints_from_route(route, course, chicane=chicane)
    return OnlinePlanner(prior, spec, env, cps, PlannerConfig(planner=planner_name)), spec, env, prior


# ---------------------------------------------------------------------- node
def main(argv=None):
    sys.path.insert(0, str(ROOT))
    import rclpy
    from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
    from rclpy.executors import MultiThreadedExecutor
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from geometry_msgs.msg import PoseStamped
    from nav_msgs.msg import Odometry, Path as PathMsg
    from sensor_msgs.msg import CameraInfo, Image
    from std_msgs.msg import String

    from .depth_projection import CameraMount, Intrinsics, decode_depth_image, project_depth

    class PlannerNode(Node):
        def __init__(self):
            super().__init__("pathplan_online")
            p = self.declare_parameter
            self.odom_topic = p("odometry_topic", "/visual_slam/tracking/odometry").value
            self.state_topic = p("state_topic", "/skybeta/state").value
            self.setpoint_topic = p("setpoint_topic", "/skybeta/setpoint/pose").value
            self.depth_topic = p("depth_topic", "").value               # "" = no depth: published map only
            self.info_topic = p("camera_info_topic", "").value
            self.publish_setpoints = p("publish_setpoints", False).value
            self.setpoint_hz = p("setpoint_hz", 10.0).value
            self.plan_hz = p("plan_hz", 5.0).value
            self.altitude = p("altitude_m", .9).value                  # course frame, above the floor
            self.carrot = p("carrot_m", .5).value
            self.reviewed = p("launch_placement_reviewed", False).value
            self.frame = OdomFrame(tuple(p("launch_xy_m", [-2.515, .914]).value),
                                   p("launch_heading_course_rad", float(np.pi / 2)).value,
                                   p("launch_height_m", .05).value)
            # camera pose on the airframe: CAMERA_X/Y/Z, CAMERA_PITCH in /etc/skybeta/drone.env
            self.mount = CameraMount(offset=tuple(p("camera_offset_flu_m", [.115, 0.0, .029]).value),
                                     pitch_down_deg=p("camera_pitch_down_deg", 0.0).value)
            self.stride = p("depth_stride", 8).value

            self.planner, self.spec, self.env, self.prior = build_planner(p("planner", "theta").value, self.altitude)
            self.lock = threading.Lock()
            self.intr = None
            self.pose = None                       # (course xyz, course yaw, rotation)
            self.target = None                     # (course xy, course yaw)
            self.offboard_active = False

            sensors, planning, output = (MutuallyExclusiveCallbackGroup() for _ in range(3))
            self.create_subscription(Odometry, self.odom_topic, self.on_odom, 10, callback_group=sensors)
            self.create_subscription(String, self.state_topic, self.on_state, 10, callback_group=sensors)
            if self.depth_topic:
                self.create_subscription(Image, self.depth_topic, self.on_depth, qos_profile_sensor_data,
                                         callback_group=sensors)
                self.create_subscription(CameraInfo, self.info_topic or self.depth_topic.rsplit("/", 1)[0]
                                         + "/camera_info", self.on_info, qos_profile_sensor_data,
                                         callback_group=sensors)
            else:
                self.get_logger().warn("no depth_topic: planning on the published map only (no obstacle sensing)")
            self.create_timer(1.0 / self.plan_hz, self.on_plan, callback_group=planning)
            self.path_pub = self.create_publisher(PathMsg, "~/path", 10)
            self.status_pub = self.create_publisher(String, "~/status", 10)
            if self.publish_setpoints:
                self.sp_pub = self.create_publisher(PoseStamped, self.setpoint_topic, 10)
                self.create_timer(1.0 / self.setpoint_hz, self.on_setpoint, callback_group=output)
            if not self.reviewed:
                self.get_logger().warn("launch_placement_reviewed is false: not planning until the drone's "
                                       "start pose on the launch mark is confirmed")
            self.get_logger().info(f"pathplan_online up ({'SETPOINTS' if self.publish_setpoints else 'shadow'} mode)")

        # ------------------------------------------------------------ inputs
        def on_odom(self, msg):
            if not self.reviewed:
                return
            p, q = msg.pose.pose.position, msg.pose.pose.orientation
            pose = self.frame.odometry_to_course((p.x, p.y, p.z), (q.x, q.y, q.z, q.w))
            with self.lock:
                self.pose = pose

        def on_state(self, msg):
            state = parse_state(msg.data)
            active = bool(state.get("offboard_active", False))
            with self.lock:
                if self.offboard_active and not active:
                    self.get_logger().warn("Offboard ended: holding position, not advancing")
                self.offboard_active = active

        def on_info(self, msg):
            if self.intr is None:
                self.intr = Intrinsics.from_k(msg.k, msg.width, msg.height)
                self.get_logger().info(f"depth intrinsics fx={self.intr.fx:.1f} {msg.width}x{msg.height}")

        def on_depth(self, msg):
            with self.lock:
                pose = self.pose
            if pose is None or self.intr is None:
                return
            depth = decode_depth_image(msg.data, msg.width, msg.height, msg.encoding, msg.step)
            xyz, _, rotation = pose
            cells = project_depth(depth, self.intr, xyz, rotation, self.spec, altitude=self.altitude,
                                  half_height=self.env.half_height + self.env.tracking_margin, mount=self.mount,
                                  stride=self.stride, expected=self.prior)
            with self.lock:
                self.planner.observe(*cells)

        # ---------------------------------------------------------- planning
        def on_plan(self):
            with self.lock:
                pose = self.pose
                if pose is None:
                    return
                xyz, yaw, _ = pose
                cmd = self.planner.update(xyz[:2], yaw)
                self.target = target_from_command(cmd, xyz[:2], yaw, self.carrot)
                checkpoint = self.planner.checkpoint
            self.status_pub.publish(String(data=json.dumps({
                "mode": cmd.mode, "reason": cmd.reason, "next": checkpoint.name if checkpoint else None,
                "cleared": [c["name"] for c in self.planner.cleared], "offboard_active": self.offboard_active})))
            if cmd.path is not None:
                out = PathMsg()
                out.header.frame_id = "odom"
                out.header.stamp = self.get_clock().now().to_msg()
                for x, y in cmd.path:
                    ps = PoseStamped()
                    ps.header = out.header
                    o = self.frame.to_odom((x, y, self.altitude))
                    ps.pose.position.x, ps.pose.position.y, ps.pose.position.z = map(float, o)
                    out.poses.append(ps)
                self.path_pub.publish(out)

        # ------------------------------------------------------------ output
        def on_setpoint(self):
            with self.lock:
                pose, target, active = self.pose, self.target, self.offboard_active
            if pose is None:
                return
            xyz, yaw, _ = pose
            if active and target is not None:
                goal, face = (target[0][0], target[0][1], self.altitude), target[1]
            else:
                # waiting (Offboard needs a recent team setpoint) or Offboard ended: hold where we are
                goal, face = tuple(xyz), yaw
            o = self.frame.to_odom(goal)
            msg = PoseStamped()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = "odom"
            msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = map(float, o)
            y_odom = self.frame.yaw_to_odom(face)
            msg.pose.orientation.z, msg.pose.orientation.w = sin(y_odom / 2), cos(y_odom / 2)
            self.sp_pub.publish(msg)

    rclpy.init(args=argv)
    node = PlannerNode()
    executor = MultiThreadedExecutor(num_threads=3)
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
