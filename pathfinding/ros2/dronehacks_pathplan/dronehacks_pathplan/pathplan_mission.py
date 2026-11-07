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
                       ``carrot_m`` ahead on the plan, facing the look-ahead point; the commanded
                       heading turns at most ``yaw_rate_max_rad_s`` (VIO is lost on fast turns)
    HOLD               centre over the landing pad to ``pad_centre_m``, then descend to the envelope
                       floor (there is no land command: the pilot / sim operator lands)
    PAUSED             Offboard lost: hold here, stop advancing (as the template does)

The planner and depth projection run in a separate low-priority process (pathplan.planner_process,
as in flyx/): sharing this process's GIL they stalled the odometry callbacks.  Depth frames are
placed with the odometry nearest their capture time, and dropped while turning faster than 0.5 rad/s.

Frames: ``odom`` is cuVSLAM's ENU frame whose origin and axes are the vehicle's pose when SLAM
started.  The drone must start ON THE LAUNCH MARK facing up the first corridor (course +y);
``launch_xy_m`` / ``launch_heading_course_rad`` describe that, and turn odom into the course
frame of the published map the planner uses.
"""
from collections import deque
import json
import math
import os
from pathlib import Path
import sys
import threading
import time

import numpy as np
import rclpy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path as PathMsg
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import String

PLAN_ABOVE_M = 0.5          # no mapping/planning below this: on the ground the camera sits 5 cm up
ODOM_PAUSE_S = 0.3          # odometry older than this: hold the last setpoint (cuVSLAM pauses ~0.7 s)
PAD_CENTRE_TIMEOUT_S = 8.0  # give up centring over the pad after this and descend anyway


def _find_pathfinding(hint):
    """The repo's pathfinding/ folder (pathplan + published course): env, or this file's checkout."""
    candidates = [hint, os.environ.get('DRONEHACKS_PATHFINDING', '')]
    here = Path(os.path.realpath(__file__))
    candidates += [str(p) for p in here.parents] + [str(p / 'pathfinding') for p in here.parents]
    for c in candidates:
        if c and (Path(c) / 'pathplan' / 'online.py').exists():
            return str(Path(c).resolve())
    raise RuntimeError('cannot find pathfinding/pathplan: set DRONEHACKS_PATHFINDING (the launch '
                       'argument pathfinding_path does) to <repo>/pathfinding')


def _wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


class PathplanMission(Node):
    def __init__(self, planner, **kwargs):
        super().__init__('dronehacks_pathplan', **kwargs)
        p = self.declare_parameter
        p('flight_altitude_m', 0.9)            # above the floor; walls are 1.73 m (rule 7.7)
        p('arrival_radius_m', 0.15)            # takeoff: how close to the climb target counts
        p('publish_rate_hz', 10.0)
        p('plan_rate_hz', 5.0)
        p('carrot_m', 0.5)                     # pose setpoints are not speed-limited: keep it short
        p('yaw_rate_max_rad_s', 0.25)          # cap on the commanded heading's rate (VIO loss on fast turns)
        p('pad_centre_m', 0.08)                # centre over the pad to this before descending
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

        from pathplan.depth_projection import CameraMount
        from pathplan.ros_node import OdomFrame

        self.altitude = float(g('flight_altitude_m'))
        self.arrival_radius = float(g('arrival_radius_m'))
        self.carrot = float(g('carrot_m'))
        self.yaw_rate_max = float(g('yaw_rate_max_rad_s'))
        self.pad_centre = float(g('pad_centre_m'))
        self.timeout = float(g('navigate_timeout_s'))
        self.shadow = bool(g('shadow'))
        self.min_altitude = float(g('min_altitude_m'))
        self.frame = OdomFrame(tuple(g('launch_xy_m')), float(g('launch_heading_course_rad')),
                               float(g('launch_height_m')))
        self.get_logger().info('building the course map from the published layout (planner process)...')
        self.planner = planner
        planner.configure(g('planner'), self.altitude, tuple(g('camera_offset_flu_m')),
                          float(g('camera_pitch_down_deg')), self.carrot)
        self.pad_xy = np.asarray(planner.pad_xy, float)

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
        self.odom_t = None                 # monotonic arrival of the latest odometry
        self.poses = deque(maxlen=300)     # (stamp s, ENU position, ENU quaternion xyzw): ~3 s of odometry
        self.poses_lock = threading.Lock() # odometry thread appends, depth thread reads
        self.start = None
        self.offboard_active = False
        self.was_offboard = False
        self.navigate_started_at = None
        self.target = None                 # (course xy, course yaw) from the planner thread
        self.plan_mode = 'none'
        self.turn_xy = None                # where a look-around began: held there while it turns
        self.last_sp = None                # last NAVIGATE setpoint (course xy, yaw): held while odometry pauses
        self.sp_yaw = None                 # last commanded course yaw (rate limiter state)
        self.sp_yaw_t = None
        self.hold_here = False
        self.pad_since = None
        self.pad_centred = False
        self.intr = None                   # CameraInfo K
        self.depth_stats = {'frames': 0, 'matched': 0, 'fallback': 0, 'turning': 0}
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
        position, quat = (float(p.x), float(p.y), float(p.z)), (q.x, q.y, q.z, q.w)
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        with self.poses_lock:
            self.poses.append((stamp, position, quat))
        self.odom_t = time.monotonic()         # before position/pose: the control thread reads them in this order
        self.pose_course = self.frame.odometry_to_course(position, quat)
        self.position = position

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
            self.intr = list(msg.k)
            self.get_logger().info(f'depth camera: {msg.width}x{msg.height}, fx {msg.k[0]:.1f}')

    def pose_at(self, stamp):
        """(course pose when the image was taken, yaw rate then): the odometry nearest its header
        stamp. Projecting with the latest pose smears walls during turns (FlyX run 4: it sealed
        the bend after checkpoint 1)."""
        with self.poses_lock:
            poses = list(self.poses)
        if stamp > 0 and poses:
            t, pos, q = min(poses, key=lambda s: abs(s[0] - stamp))
            if abs(t - stamp) <= .05:
                self.depth_stats['matched'] += 1
                t0, pos0, q0 = min(poses, key=lambda s: abs(s[0] - (stamp - .15)))
                pose = self.frame.odometry_to_course(pos, q)
                dyaw = _wrap(pose[1] - self.frame.odometry_to_course(pos0, q0)[1])
                return pose, abs(dyaw) / max(t - t0, .05)
        self.depth_stats['fallback'] += 1          # unstamped / other clock: the latest pose
        return self.pose_course, 0.0

    def depth_cb(self, msg):
        if self.intr is None or not self.planning_allowed():
            return
        from pathplan.planner_process import MAX_TURN_RATE
        pose, rate = self.pose_at(msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9)
        if pose is None:
            return
        self.depth_stats['frames'] += 1
        if rate > MAX_TURN_RATE:                   # pose error while turning: skip the frame
            self.depth_stats['turning'] += 1
            return
        xyz, _, rotation = pose
        # the planner process takes the latest frame at its next step (<= plan_rate_hz)
        self.planner.give_frame((bytes(msg.data), msg.width, msg.height, msg.encoding, msg.step, self.intr,
                                 xyz, rotation))

    def planning_allowed(self):
        """Map and plan only in flight (or in shadow): on the ground each "look around" call
        spent turning budget the vehicle never used."""
        if self.pose_course is None:
            return False
        if self.shadow or self.state == 'NAVIGATE':
            return True
        return self.state == 'TAKEOFF' and self.pose_course[0][2] > PLAN_ABOVE_M

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

    def limit_yaw(self, yaw):
        """Rate-limit the commanded course heading to yaw_rate_max_rad_s (starts from the vehicle's yaw).
        cuVSLAM loses tracking on fast turns: 30 of 40 losses in Suraj's flyx_course runs were above
        0.35 rad/s, 3/3 Gazebo completions at 0.25."""
        now = time.monotonic()
        if self.sp_yaw is None:
            self.sp_yaw, self.sp_yaw_t = self.pose_course[1], now
        step = self.yaw_rate_max * max(0.0, min(now - self.sp_yaw_t, .2))
        self.sp_yaw = _wrap(self.sp_yaw + float(np.clip(_wrap(yaw - self.sp_yaw), -step, step)))
        self.sp_yaw_t = now
        return self.sp_yaw

    # ------------------------------------------------------------------ planner thread
    def plan_tick(self):
        if not self.planning_allowed():
            return
        from pathplan.ros_node import target_from_command
        xyz, yaw, _ = self.pose_course
        cmd = self.planner.update(xyz[:2], yaw)    # blocks this thread only, not the odometry callbacks
        target = target_from_command(cmd, xyz[:2], yaw, self.carrot)
        if cmd.mode == 'turn':
            # hold the spot where the turning began: re-holding "here" every step let the vehicle
            # drift 11 cm during a look-around in FlyX run 6, toward a wall
            if self.turn_xy is None:
                self.turn_xy = np.asarray(xyz[:2], float).copy()
            target = (self.turn_xy, target[1])
        else:
            self.turn_xy = None
        self.target, self.plan_mode = target, cmd.mode
        if cmd.path is not None:                   # [here, carrot]: the planner process shortens the plan
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
        # ready_to_land: the pilot's cue (sim: team_mission sim_pilot). True once centred over the pad,
        # when holding after a failure, or when Offboard was lost; not while still centring.
        ready = (self.state == 'PAUSED' or
                 (self.state == 'HOLD' and (self.hold_here or self.pad_centred)))
        self.status_pub.publish(String(data=json.dumps({
            'state': self.state, 'planner': self.plan_mode, 'ready_to_land': ready,
            'cleared': [c['name'] for c in self.planner.cleared], 'offboard_active': self.offboard_active,
            'depth': self.depth_stats, 'depth_quality': self.planner.depth_quality})))

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
            self.get_logger().info('landing pad reached -- centring over it; the pilot / operator lands.')
            return
        if self.plan_mode == 'failed' or self.now_s() - self.navigate_started_at > self.timeout:
            why = 'planner failed: ' + (self.planner.events[-1]['event'] if self.planner.events else '?') \
                if self.plan_mode == 'failed' else 'time limit'
            self.get_logger().error(f'{why} -- holding here.')
            self.state = 'HOLD'
            self.hold_here = True
            return
        if time.monotonic() - self.odom_t > ODOM_PAUSE_S and self.last_sp is not None:
            # cuVSLAM pauses odometry for up to ~0.7 s: keep sending the last setpoint (the flight
            # interface keeps flying on its own estimate) rather than steering on a stale pose
            self.info_once('odom_pause', 'odometry paused: holding the last setpoint')
            self.send_course(*self.last_sp)
            return
        if self.target is None:                # first plan not in yet: hold at altitude
            xyz, yaw, _ = self.pose_course
            xy, face = xyz[:2], yaw
        else:
            xy, face = self.target
        self.last_sp = (xy, self.limit_yaw(face))
        self.send_course(*self.last_sp)

    def _state_hold(self):
        xyz, yaw, _ = self.pose_course
        if self.hold_here:
            self.send_course(xyz[:2], yaw)
            return
        # centre over the pad at cruise height first (landing where "reached" fired put the drone
        # 0.25 m off centre in Gazebo), then go to flight_interface's floor (min_altitude_m above
        # the odom origin) so the setpoint is not clamped (and logged) every cycle
        now = time.monotonic()
        self.pad_since = self.pad_since if self.pad_since is not None else now
        off = float(np.hypot(*(np.asarray(xyz[:2]) - self.pad_xy)))
        if not self.pad_centred and (off <= self.pad_centre or now - self.pad_since > PAD_CENTRE_TIMEOUT_S):
            self.pad_centred = True
            self.get_logger().info(f'over the pad ({off:.2f} m from its centre) -- descending to the floor.')
        if not self.pad_centred:
            self.send_course(self.pad_xy, yaw)
            return
        floor = self.frame.to_course((0.0, 0.0, self.min_altitude))[2]
        self.send_course(self.pad_xy, yaw, altitude=floor)

    def _state_paused(self):
        self.send(*self.paused_at)

    def now_s(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def summary(self):
        """What happened, for the terminal after the run."""
        q = self.planner.depth_quality or {}
        lines = ['', '=' * 72, f'RESULT: {self.state} (planner {self.plan_mode})',
                 f'checkpoints cleared: {", ".join(c["name"] for c in self.planner.cleared) or "none"}'
                 f'  ({len(self.planner.cleared)}/5)']
        if q.get('hits'):
            lines.append(f'depth: {self.depth_stats["frames"]} frames ({self.depth_stats["turning"]} dropped '
                         f'turning); {q["hits_on_published_walls"] / q["hits"]:.0%} of hits on published walls '
                         f'(low = wrong camera mount); new obstacles {q["hits_new_confirmed"]}')
        elif self.depth_stats['frames'] == 0:
            lines.append('depth: no frames used (check depth_topic / camera_info_topic)')
        if self.planner.events:
            lines.append('planner events (last 5):')
            lines += [f'  {e}' for e in self.planner.events[-5:]]
        lines.append('=' * 72)
        print('\n'.join(lines), flush=True)


def main(args=None):
    sys.path.insert(0, _find_pathfinding(''))
    from pathplan.planner_process import PlannerProcess
    from pathplan.ros_node import build_planner
    planner = PlannerProcess(build_planner)    # first: it forks, which is only safe before ROS threads exist
    rclpy.init(args=args)
    node = PathplanMission(planner)
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
        node.summary()
        executor.shutdown()
        planner.close()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
