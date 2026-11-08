"""path_player ROS 2 node: Replays recorded paths through /skybeta/setpoint/pose.

Contract:
---------
1. Subscribes to /visual_slam/tracking/odometry (where we are, in ENU odom)
2. Subscribes to /skybeta/state (PX4 state via flight_interface)
3. Publishes to /skybeta/setpoint/pose (where we want to be, in ENU odom)
4. Publishes ~/status (JSON at 2 Hz for telemetry / monitoring)

Safety:
-------
- Streams current pose while waiting for Offboard so flight_interface can transition cleanly.
- Starts trajectory playback only when /skybeta/state indicates offboard_active == True.
- Stops advancing trajectory if Offboard is lost (safety pilot takes control).
- Anchors trajectory on the arming pose (p0, yaw0) on the ground, exactly matching path_recorder.
"""
from __future__ import annotations

import json
import math
from pathlib import Path
import time
from typing import Optional

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
import numpy as np
import rclpy
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import String

from path_player.replay import (
    TrajectoryPlayer,
    arm_to_odom,
    find_latest_recording,
    load_recording,
    quaternion_from_yaw,
    runs_dir,
    wrap,
    yaw_of,
)

PARAMS = {
    'recording_path': ('', 'path to recording folder; empty = latest in runs dir'),
    'publish_rate_hz': (20.0, 'setpoint publishing rate in Hz'),
    'mode': ('adaptive', 'playback mode: adaptive (time with lag cap), time, or waypoint'),
    'speed_factor': (1.0, 'speed multiplier (e.g. 0.8 for 80% speed)'),
    'arrival_radius_m': (0.25, 'arrival radius for waypoint mode in metres'),
    'max_lag_m': (0.5, 'maximum allowed lag in adaptive mode before pausing time'),
    'command_yaw': (True, 'whether to command yaw orientation from recording'),
    'anchor_mode': ('armed', 'anchor source: armed (at arming), offboard, recorded, or fixed'),
    'anchor_xyz': ([0.0, 0.0, 0.0], 'anchor position for fixed mode [x, y, z]'),
    'anchor_yaw': (0.0, 'anchor yaw in radians for fixed mode'),
    'hold_at_end': (True, 'hold final position when trajectory ends'),
    'odometry_topic': ('/visual_slam/tracking/odometry', 'cuVSLAM odometry topic (ENU odom)'),
    'state_topic': ('/skybeta/state', 'flight_interface state JSON topic'),
    'setpoint_topic': ('/skybeta/setpoint/pose', 'setpoint pose topic'),
}


class PathPlayer(Node):

    def __init__(self):
        super().__init__('path_player')
        for name, (default, help_) in PARAMS.items():
            self.declare_parameter(name, default, ParameterDescriptor(description=help_, dynamic_typing=True))

        g = lambda n: self.get_parameter(n).value
        self.recording_path_param = str(g('recording_path'))
        self.rate_hz = max(1.0, float(g('publish_rate_hz')))
        self.mode = str(g('mode')).lower()
        self.speed_factor = float(g('speed_factor'))
        self.arrival_radius = float(g('arrival_radius_m'))
        self.max_lag = float(g('max_lag_m'))
        self.command_yaw = bool(g('command_yaw'))
        self.anchor_mode = str(g('anchor_mode')).lower()
        self.fixed_anchor_xyz = [float(v) for v in g('anchor_xyz')]
        self.fixed_anchor_yaw = float(g('anchor_yaw'))
        self.hold_at_end = bool(g('hold_at_end'))

        self.odom_topic = str(g('odometry_topic'))
        self.state_topic = str(g('state_topic'))
        self.setpoint_topic = str(g('setpoint_topic'))

        # Load recording
        self.folder: Optional[Path] = None
        self.meta = {}
        self.points = []
        self._load_path()

        # Telemetry & state
        self.state = 'WAIT_FOR_ODOM'
        self.odom_pos: Optional[np.ndarray] = None
        self.odom_q: Optional[tuple] = None
        self.odom_yaw: Optional[float] = None
        self.odom_t: Optional[float] = None

        self.px4_state = {}
        self.px4_state_t: Optional[float] = None
        self.armed_prev: Optional[bool] = None
        self.offboard_active = False
        self.was_offboard = False

        # Anchor pose
        self.anchor_pos: Optional[np.ndarray] = None
        self.anchor_yaw: Optional[float] = None
        self.anchor_latched = False

        # Trajectory player engine
        self.player: Optional[TrajectoryPlayer] = None
        self._init_player()

        # Target pose
        self.target_pos = np.zeros(3, dtype=float)
        self.target_yaw = 0.0
        self.player_info = {}

        # ROS interfaces
        cb_sensors = MutuallyExclusiveCallbackGroup()
        cb_timer = MutuallyExclusiveCallbackGroup()
        cb_slow = MutuallyExclusiveCallbackGroup()

        self.create_subscription(Odometry, self.odom_topic, self.on_odom, qos_profile_sensor_data, callback_group=cb_sensors)
        self.create_subscription(String, self.state_topic, self.on_state, 10, callback_group=cb_sensors)

        self.setpoint_pub = self.create_publisher(PoseStamped, self.setpoint_topic, 10)
        self.status_pub = self.create_publisher(String, '~/status', 10)

        self.create_timer(1.0 / self.rate_hz, self.on_tick, callback_group=cb_timer)
        self.create_timer(0.5, self.on_status, callback_group=cb_slow)

        self.get_logger().info(
            f'path_player ready: mode={self.mode}, speed={self.speed_factor}x, anchor_mode={self.anchor_mode}. '
            f'Loaded {len(self.points)} points from {self.folder.name if self.folder else "none"}. '
            f'Waiting for pilot to arm and switch to Position (Offboard)...'
        )

    def _load_path(self):
        if self.recording_path_param:
            target = Path(self.recording_path_param)
        else:
            target = find_latest_recording()

        if not target or not target.exists():
            self.get_logger().error(f'No valid recording found at {self.recording_path_param or runs_dir()}')
            return

        self.folder = target
        try:
            self.meta, self.points = load_recording(target)
            self.get_logger().info(f'Loaded path {target.name}: {len(self.points)} points, '
                                   f'{self.meta.get("length_m", 0.0):.2f}m, '
                                   f'{self.meta.get("duration_s", 0.0):.1f}s')
        except Exception as e:
            self.get_logger().error(f'Failed to load recording from {target}: {e}')

    def _init_player(self):
        if not self.points:
            return

        p0 = np.array(self.fixed_anchor_xyz) if self.anchor_mode == 'fixed' else np.zeros(3)
        yaw0 = self.fixed_anchor_yaw if self.anchor_mode == 'fixed' else 0.0

        if self.anchor_mode == 'recorded' and 'anchor' in self.meta:
            rec_anchor = self.meta['anchor']
            p0 = np.array(rec_anchor.get('odom_position', [0.0, 0.0, 0.0]))
            yaw0 = float(rec_anchor.get('odom_yaw', 0.0))
            self.anchor_pos, self.anchor_yaw = p0, yaw0
            self.anchor_latched = True

        self.player = TrajectoryPlayer(
            points=self.points,
            meta=self.meta,
            anchor_pos=p0,
            anchor_yaw=yaw0,
            speed_factor=self.speed_factor,
            arrival_radius_m=self.arrival_radius,
            mode=self.mode,
            max_lag_m=self.max_lag,
            command_yaw=self.command_yaw,
        )

    # ------------------------------------------------------------------ Callbacks
    def on_odom(self, msg: Odometry):
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        pos = np.array([p.x, p.y, p.z], dtype=float)
        quat = (float(q.x), float(q.y), float(q.z), float(q.w))

        if not np.all(np.isfinite(pos)) or not all(math.isfinite(v) for v in quat) or sum(v * v for v in quat) < 0.5:
            return

        self.odom_pos = pos
        self.odom_q = quat
        self.odom_yaw = yaw_of(*quat)
        self.odom_t = time.monotonic()

        if self.state == 'WAIT_FOR_ODOM':
            self.state = 'WAIT_FOR_OFFBOARD'
            self.get_logger().info(f'Odometry received at ENU ({pos[0]:.2f}, {pos[1]:.2f}, {pos[2]:.2f}). '
                                   'Ready for arming.')

    def on_state(self, msg: String):
        try:
            s = json.loads(msg.data)
        except ValueError:
            return
        if not isinstance(s, dict):
            return

        self.px4_state = s
        self.px4_state_t = time.monotonic()
        armed = bool(s.get('armed', False))
        self.offboard_active = bool(s.get('offboard_active', False))

        # Handle arming state transitions
        if self.armed_prev is None:
            if armed:
                self._handle_armed()
        elif armed and not self.armed_prev:
            self._handle_armed()
        elif not armed and self.armed_prev:
            self._handle_disarmed()

        self.armed_prev = armed

    def _handle_armed(self):
        if self.anchor_mode == 'armed' and not self.anchor_latched:
            self._latch_anchor('ARMED')

    def _handle_disarmed(self):
        self.get_logger().info('Vehicle DISARMED.')
        if self.state in ('PLAYING', 'PAUSED'):
            self.state = 'HOLD_END'
        self.anchor_latched = False

    def _latch_anchor(self, trigger_name: str):
        if self.odom_pos is None:
            self.get_logger().warn(f'{trigger_name} but no odometry yet; anchor will be default (0, 0, 0)')
            self.anchor_pos = np.zeros(3)
            self.anchor_yaw = 0.0
        else:
            self.anchor_pos = self.odom_pos.copy()
            self.anchor_yaw = self.odom_yaw

        self.anchor_latched = True
        if self.player is not None:
            self.player.set_anchor(self.anchor_pos, self.anchor_yaw)

        self.get_logger().info(
            f'Anchor latched on {trigger_name}: ENU position ({self.anchor_pos[0]:.2f}, '
            f'{self.anchor_pos[1]:.2f}, {self.anchor_pos[2]:.2f}), yaw {math.degrees(self.anchor_yaw):.1f}°'
        )

    # ------------------------------------------------------------------ Control loop
    def on_tick(self):
        if self.state == 'WAIT_FOR_ODOM' or self.odom_pos is None or self.player is None:
            return

        now = time.monotonic()

        # Handle Offboard lost
        if self.was_offboard and not self.offboard_active:
            self.get_logger().warn('Offboard lost! Safety pilot took control. Replay paused.')
            self.was_offboard = False
            if self.state == 'PLAYING':
                self.state = 'PAUSED'
                self.player.pause()

        if self.offboard_active and not self.was_offboard:
            self.was_offboard = True
            if self.state == 'PAUSED':
                self.get_logger().info('Offboard restored. Resuming playback.')
                self.state = 'PLAYING'
                self.player.resume(now)

        # State machine
        if self.state == 'WAIT_FOR_OFFBOARD':
            if not self.offboard_active:
                # Stream current position so flight_interface can safely switch to Offboard
                self.send_pose(self.odom_pos, self.odom_yaw)
                return

            # Offboard became active!
            if self.anchor_mode == 'offboard' and not self.anchor_latched:
                self._latch_anchor('OFFBOARD')
            elif not self.anchor_latched:
                self._latch_anchor('OFFBOARD_FALLBACK')

            self.state = 'PLAYING'
            self.player.start(now)
            self.get_logger().info(f'Offboard active! Commencing replay of {len(self.points)} points.')

        elif self.state == 'PLAYING':
            pos, yaw, info = self.player.update(now, self.odom_pos)
            self.target_pos = pos
            self.target_yaw = yaw
            self.player_info = info

            self.send_pose(pos, yaw)

            if self.player.is_finished():
                self.get_logger().info('Trajectory playback complete! Holding final waypoint.')
                self.state = 'HOLD_END'

        elif self.state == 'PAUSED':
            # While paused, command current position so vehicle holds in place
            self.send_pose(self.odom_pos, self.odom_yaw)

        elif self.state == 'HOLD_END':
            if self.hold_at_end:
                self.send_pose(self.target_pos, self.target_yaw)

    def send_pose(self, pos: np.ndarray | tuple, yaw: float):
        """Publish one PoseStamped setpoint in the ENU 'odom' frame."""
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'odom'
        msg.pose.position.x = float(pos[0])
        msg.pose.position.y = float(pos[1])
        msg.pose.position.z = float(pos[2])

        if self.command_yaw:
            qx, qy, qz, qw = quaternion_from_yaw(yaw)
            msg.pose.orientation.x = float(qx)
            msg.pose.orientation.y = float(qy)
            msg.pose.orientation.z = float(qz)
            msg.pose.orientation.w = float(qw)
        else:
            # All-zero quaternion tells flight_interface to preserve current heading
            msg.pose.orientation.x = 0.0
            msg.pose.orientation.y = 0.0
            msg.pose.orientation.z = 0.0
            msg.pose.orientation.w = 0.0

        self.setpoint_pub.publish(msg)

    # ------------------------------------------------------------------ Status telemetry
    def on_status(self):
        now = time.monotonic()
        p = self.player

        status_data = {
            'state': self.state,
            'folder': str(self.folder) if self.folder else None,
            'total_points': len(self.points),
            'current_idx': p.current_idx if p else 0,
            'progress_pct': round(p.progress_pct(), 1) if p else 0.0,
            'rem_distance_m': round(p.remaining_distance_m(self.odom_pos), 2) if p else 0.0,
            'lag_m': round(p.active_lag_m, 3) if p else 0.0,
            'speed_factor': self.speed_factor,
            'offboard_active': self.offboard_active,
            'armed': bool(self.px4_state.get('armed', False)),
            'mode': self.px4_state.get('nav_state', 'UNKNOWN'),
            'anchor_latched': self.anchor_latched,
            'anchor_pos': [round(float(v), 3) for v in self.anchor_pos] if self.anchor_pos is not None else None,
            'anchor_yaw_deg': round(math.degrees(self.anchor_yaw), 1) if self.anchor_yaw is not None else None,
            'odom_age_s': round(now - self.odom_t, 2) if self.odom_t else None,
            'target_pos': [round(float(v), 3) for v in self.target_pos],
            'target_yaw_deg': round(math.degrees(self.target_yaw), 1),
            'drone_pos': [round(float(v), 3) for v in self.odom_pos] if self.odom_pos is not None else None,
        }

        self.status_pub.publish(String(data=json.dumps(status_data)))


def main(args=None):
    rclpy.init(args=args)
    node = PathPlayer()
    ex = MultiThreadedExecutor(num_threads=3)
    ex.add_node(node)
    try:
        ex.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        try:
            ex.shutdown()
            node.destroy_node()
        except Exception:
            pass
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
