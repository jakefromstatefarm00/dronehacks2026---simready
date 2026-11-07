"""The skybeta competitor node with our online path planner instead of the square.

Forked from the organisers' ``competitor_example/square_mission.py``: same contract, same
safety behaviour, only the mission is different.

    subscribe  /skybeta/state                     what PX4 is doing
    subscribe  /visual_slam/tracking/odometry     where we are (ENU, odom)
    subscribe  <depth_topic> + CameraInfo         optional: obstacles the published map lacks
    publish    /skybeta/setpoint/pose             where we want to be (ENU, odom)
    publish    ~/status, ~/path                   for watching and logging only

It never talks to /fmu/*, never arms, never changes mode: ``skybeta_flight_interface``
owns that and clamps every setpoint to its envelope.

Mission (10 Hz state machine; the planner runs on its own thread):
    WAIT_FOR_OFFBOARD  command the current position (this stream lets Offboard start)
    TAKEOFF            climb straight up to ``flight_altitude_m`` above the floor
    NAVIGATE           pathplan.OnlinePlanner through the five checkpoints: a carrot
                       ``carrot_m`` ahead on the plan, facing the look-ahead point
    HOLD               over the landing pad at the envelope floor (there is no land command:
                       the pilot / sim operator lands)
    PAUSED             Offboard lost: hold here, stop advancing (as the template does)

Frames: ``odom`` is cuVSLAM's ENU frame whose origin and axes are the vehicle's pose when SLAM
started.  The drone must start ON THE LAUNCH MARK facing up the first corridor (course +y);
``launch_xy_m`` / ``launch_heading_course_rad`` describe that, and turn odom into the course
frame of the published map the planner uses.
"""
import json
import math
import os
from pathlib import Path
import sys
import threading

import rclpy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path as PathMsg
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import String


def _find_pathfinding(hint):
    """The repo's pathfinding/ folder (pathplan + published course): parameter, env, or this file's checkout."""
    candidates = [hint, os.environ.get('DRONEHACKS_PATHFINDING', '')]
    here = Path(os.path.realpath(__file__))
    candidates += [str(p) for p in here.parents] + [str(p / 'pathfinding') for p in here.parents]
    for c in candidates:
        if c and (Path(c) / 'pathplan' / 'online.py').exists():
            return str(Path(c).resolve())
    raise RuntimeError('cannot find pathfinding/pathplan: set the pathfinding_path parameter or '
                       'DRONEHACKS_PATHFINDING to <repo>/pathfinding')


class PathplanMission(Node):
    def __init__(self, **kwargs):
        super().__init__('dronehacks_pathplan', **kwargs)
        p = self.declare_parameter
        p('pathfinding_path', '')
        p('flight_altitude_m', 0.9)            # above the floor; walls are 1.73 m (rule 7.7)
        p('arrival_radius_m', 0.15)            # takeoff: how close to the climb target counts
        p('publish_rate_hz', 10.0)
        p('plan_rate_hz', 5.0)
        p('carrot_m', 0.5)                     # pose setpoints are not speed-limited: keep it short
        p('planner', 'theta')                  # theta | astar | static (published map only)
        p('depth_topic', '')                   # '' = no sensing: published map only
        p('camera_info_topic', '')             # default: <depth_topic parent>/camera_info
        p('camera_offset_flu_m', [0.115, 0.0, 0.029])   # Sky Beta CAD; drone.env CAMERA_X/Y/Z on hardware
        p('camera_pitch_down_deg', 0.0)
        p('launch_xy_m', [-2.515, 0.914])      # the launch mark, course frame
        p('launch_heading_course_rad', math.pi / 2)   # facing up the first corridor
        p('launch_height_m', 0.05)             # body origin above the floor at SLAM start
        p('navigate_timeout_s', 175.0)         # rule 7.9: 3-minute limit
        p('shadow', False)                     # True: plan and publish ~/path only, no setpoints
        p('min_altitude_m', 0.3)               # flight_interface's floor above the odom origin (hold height)
        g = lambda name: self.get_parameter(name).value

        sys.path.insert(0, _find_pathfinding(g('pathfinding_path')))
        from pathplan.depth_projection import CameraMount
        from pathplan.ros_node import OdomFrame, build_planner

        self.altitude = float(g('flight_altitude_m'))
        self.arrival_radius = float(g('arrival_radius_m'))
        self.carrot = float(g('carrot_m'))
        self.timeout = float(g('navigate_timeout_s'))
        self.shadow = bool(g('shadow'))
        self.min_altitude = float(g('min_altitude_m'))
        self.frame = OdomFrame(tuple(g('launch_xy_m')), float(g('launch_heading_course_rad')),
                               float(g('launch_height_m')))
        self.mount = CameraMount(offset=tuple(g('camera_offset_flu_m')),
                                 pitch_down_deg=float(g('camera_pitch_down_deg')))
        self.get_logger().info('building the course map from the published layout...')
        self.planner, self.spec, self.env, self.prior = build_planner(g('planner'), self.altitude)

        # --- the contract ----------------------------------------------
        self.setpoint_pub = self.create_publisher(PoseStamped, '/skybeta/setpoint/pose', 10)
        self.status_pub = self.create_publisher(String, '~/status', 10)
        self.path_pub = self.create_publisher(PathMsg, '~/path', 10)
        sensors, depth_group, planning, control = (MutuallyExclusiveCallbackGroup() for _ in range(4))
        self.create_subscription(Odometry, '/visual_slam/tracking/odometry', self.odometry_cb, 10,
                                 callback_group=sensors)
        self.create_subscription(String, '/skybeta/state', self.state_cb, 10, callback_group=sensors)
        depth = g('depth_topic')
        if depth:
            info = g('camera_info_topic') or depth.rsplit('/', 1)[0] + '/camera_info'
            self.create_subscription(Image, depth, self.depth_cb, qos_profile_sensor_data, callback_group=depth_group)
            self.create_subscription(CameraInfo, info, self.info_cb, qos_profile_sensor_data, callback_group=depth_group)
        else:
            self.get_logger().warn('no depth_topic: flying the published map only (no obstacle sensing)')

        # --- mission state ---------------------------------------------
        self.state = 'WAIT_FOR_OFFBOARD'
        self.position = None               # ENU odom
        self.pose_course = None            # (course xyz, course yaw, body->course rotation)
        self.start = None
        self.offboard_active = False
        self.was_offboard = False
        self.navigate_started_at = None
        self.target = None                 # (course xy, course yaw) from the planner thread
        self.plan_mode = 'none'
        self.intr = None
        self.lock = threading.Lock()       # planner: depth observe() vs update()
        self.log_once = set()

        self.create_timer(1.0 / float(g('publish_rate_hz')), self.tick, callback_group=control)
        self.create_timer(1.0 / float(g('plan_rate_hz')), self.plan_tick, callback_group=planning)
        self.get_logger().info(
            f'dronehacks_pathplan ready ({"SHADOW: no setpoints" if self.shadow else "flying"}): '
            f'planner {g("planner")}, altitude {self.altitude:.2f} m, depth {"on" if depth else "off"}. '
            'Start ON THE LAUNCH MARK facing up the first corridor. Waiting for Offboard...')

    # ------------------------------------------------------------------ inputs
    def odometry_cb(self, msg):
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        self.position = (float(p.x), float(p.y), float(p.z))
        self.pose_course = self.frame.odometry_to_course(self.position, (q.x, q.y, q.z, q.w))

    def state_cb(self, msg):
        try:
            state = json.loads(msg.data)
        except ValueError:
            return
        self.offboard_active = bool(state.get('offboard_active', False))
        if state.get('clamped'):
            self.info_once(f'clamped:{",".join(state["clamped"])}',
                           f'flight interface is CLAMPING our setpoints ({", ".join(state["clamped"])}), '
                           f'envelope {state.get("envelope")}')

    def info_cb(self, msg):
        if self.intr is None:
            from pathplan.depth_projection import Intrinsics
            self.intr = Intrinsics.from_k(msg.k, msg.width, msg.height)
            self.get_logger().info(f'depth camera: {msg.width}x{msg.height}, fx {self.intr.fx:.1f}')

    def depth_cb(self, msg):
        pose = self.pose_course
        if pose is None or self.intr is None:
            return
        from pathplan.depth_projection import decode_depth_image, project_depth
        depth = decode_depth_image(msg.data, msg.width, msg.height, msg.encoding, msg.step)
        xyz, _, rotation = pose
        cells = project_depth(depth, self.intr, xyz, rotation, self.spec, altitude=self.altitude,
                              half_height=self.env.half_height + self.env.tracking_margin,
                              mount=self.mount, expected=self.prior)
        with self.lock:
            self.planner.observe(*cells)

    def info_once(self, key, message):
        if key not in self.log_once:
            self.log_once.add(key)
            self.get_logger().info(message)

    # ------------------------------------------------------------------ outputs
    def send(self, x, y, z, yaw_enu=None):
        """One ENU setpoint in ``odom`` (all-zero quaternion = do not command yaw)."""
        if self.shadow:
            return
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'odom'
        msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = float(x), float(y), float(z)
        if yaw_enu is None:
            msg.pose.orientation.w = 0.0
        else:
            msg.pose.orientation.z = math.sin(float(yaw_enu) / 2.0)
            msg.pose.orientation.w = math.cos(float(yaw_enu) / 2.0)
        self.setpoint_pub.publish(msg)

    def send_course(self, xy, yaw_course, altitude=None):
        o = self.frame.to_odom((xy[0], xy[1], self.altitude if altitude is None else altitude))
        self.send(o[0], o[1], o[2], self.frame.yaw_to_odom(yaw_course))

    # ------------------------------------------------------------------ planner thread
    def plan_tick(self):
        if self.pose_course is None or self.state in ('HOLD', 'PAUSED'):
            return
        from pathplan.ros_node import target_from_command
        xyz, yaw, _ = self.pose_course
        with self.lock:
            cmd = self.planner.update(xyz[:2], yaw)
        self.target = target_from_command(cmd, xyz[:2], yaw, self.carrot)
        self.plan_mode = cmd.mode
        if cmd.path is not None:
            out = PathMsg()
            out.header.frame_id = 'odom'
            out.header.stamp = self.get_clock().now().to_msg()
            for x, y in cmd.path:
                ps = PoseStamped()
                ps.header = out.header
                o = self.frame.to_odom((x, y, self.altitude))
                ps.pose.position.x, ps.pose.position.y, ps.pose.position.z = map(float, o)
                out.poses.append(ps)
            self.path_pub.publish(out)

    # ------------------------------------------------------------------ the 10 Hz state machine
    def tick(self):
        if self.position is None:
            self.info_once('no_odom', 'no /visual_slam/tracking/odometry yet -- is cuVSLAM up and tracking? '
                                      'Nothing will be commanded until it is.')
            return
        if self.was_offboard and not self.offboard_active and self.state not in ('PAUSED', 'HOLD'):
            self.get_logger().warn('Offboard lost -- the pilot has control. Mission paused '
                                   '(Offboard returns only after disarm + re-arm).')
            self.state = 'PAUSED'
            self.paused_at = self.position
        if self.offboard_active:
            self.was_offboard = True
        getattr(self, f'_state_{self.state.lower()}')()
        self.status_pub.publish(String(data=json.dumps({
            'state': self.state, 'planner': self.plan_mode,
            'next': getattr(self.planner.checkpoint, 'name', None),
            'cleared': [c['name'] for c in self.planner.cleared], 'offboard_active': self.offboard_active})))

    def _state_wait_for_offboard(self):
        if not self.offboard_active:
            self.info_once('waiting', 'holding at the current position until the pilot arms, then switches '
                                      'to Position.')
            self.send(*self.position)          # this stream is what lets Offboard start
            return
        self.start = self.position
        self.state = 'TAKEOFF'
        self.get_logger().info(f'Offboard active; climbing to {self.altitude:.2f} m above the floor.')

    def _state_takeoff(self):
        xyz, yaw, _ = self.pose_course
        climb = self.frame.to_odom((*self.frame.to_course(self.start)[:2], self.altitude))
        self.send(climb[0], climb[1], climb[2], self.frame.yaw_to_odom(yaw))
        if math.dist(self.position, climb) <= self.arrival_radius:
            self.state = 'NAVIGATE'
            self.navigate_started_at = self.now_s()
            self.get_logger().info('at altitude -- navigating the course with the online planner.')

    def _state_navigate(self):
        if self.plan_mode == 'done':
            self.state = 'HOLD'
            self.get_logger().info('landing pad reached -- holding low over it; the pilot / operator lands.')
            return
        if self.plan_mode == 'failed' or self.now_s() - self.navigate_started_at > self.timeout:
            why = 'planner failed: ' + (self.planner.events[-1]['event'] if self.planner.events else '?') \
                if self.plan_mode == 'failed' else 'time limit'
            self.get_logger().error(f'{why} -- holding here.')
            self.state = 'HOLD'
            self.hold_here = True
            return
        if self.target is None:                # first plan not in yet: hold at altitude
            xyz, yaw, _ = self.pose_course
            self.send_course(xyz[:2], yaw)
            return
        self.send_course(*self.target)

    def _state_hold(self):
        if getattr(self, 'hold_here', False) or self.planner.checkpoint is not None:
            xyz, yaw, _ = self.pose_course
            self.send_course(xyz[:2], yaw)
            return
        # over the pad, exactly at flight_interface's floor (min_altitude_m above the odom origin),
        # so the setpoint is not clamped (and logged) every cycle
        pad = self.planner.checkpoints[-1].centre
        floor = self.frame.to_course((0.0, 0.0, self.min_altitude))[2]
        self.send_course(pad, self.pose_course[1], altitude=floor)

    def _state_paused(self):
        self.send(*self.paused_at)

    def now_s(self):
        return self.get_clock().now().nanoseconds * 1e-9


def main(args=None):
    rclpy.init(args=args)
    node = PathplanMission()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except Exception:                      # a signal shut the context down mid-wait: a normal stop
        if rclpy.ok():
            raise
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
