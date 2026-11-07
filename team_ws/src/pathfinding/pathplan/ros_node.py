"""Helpers shared by the ROS node and the tests (no ROS import here).

The node itself is ``dronehacks_pathplan/node.py`` (the Sky Beta competitor node). This module
holds the pure pieces: the odom <-> course frame, command -> setpoint target, the
``/skybeta/state`` parser, and ``build_planner()`` (published course -> OnlinePlanner).

Frames: ``odom`` is cuVSLAM's ENU frame -- origin and axes = the vehicle's pose when SLAM
started (x = forward, y = left, z = up). ``OdomFrame`` maps it onto the course frame of the
published map. By default it assumes SLAM started on the launch mark facing up the first
corridor; ``OdomFrame.latched`` instead takes the odom pose at the moment the drone sits on
the launch mark (the node latches it when the pilot arms), which also covers a drone that was
powered on elsewhere and carried to the mark.
"""
from __future__ import annotations

from dataclasses import dataclass
import json
from math import atan2, cos, sin
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


def yaw_of_quat(x, y, z, w):
    """Heading (rad, about +z, CCW) of a ROS (x, y, z, w) quaternion."""
    return atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))


@dataclass(frozen=True)
class OdomFrame:
    """cuVSLAM ``odom`` (ENU) <-> course frame.

    The drone sits on the launch mark (``launch_xy``, heading ``launch_heading``, body origin
    ``launch_height`` above the floor) at odom pose ``origin`` / ``origin_yaw``. The defaults
    (origin 0, yaw 0) mean "SLAM started on the mark"."""
    launch_xy: tuple = (-2.515, .914)       # launch mark, course frame (m)
    launch_heading: float = np.pi / 2       # vehicle heading on the mark, course frame (rad)
    launch_height: float = .05              # body origin above the floor on the mark (m)
    origin: tuple = (0.0, 0.0, 0.0)         # odom position while on the mark
    origin_yaw: float = 0.0                 # odom heading while on the mark

    @classmethod
    def latched(cls, position, quat_xyzw, launch_xy=(-2.515, .914), launch_heading=np.pi / 2,
                launch_height=.05):
        """The frame for a drone sitting on the launch mark right now at this odom pose."""
        return cls(tuple(float(v) for v in launch_xy), float(launch_heading), float(launch_height),
                   tuple(float(v) for v in position), float(yaw_of_quat(*quat_xyzw)))

    @property
    def theta(self):
        """Rotation odom -> course about +z."""
        return self.launch_heading - self.origin_yaw

    def to_course(self, p_odom):
        p = np.asarray(p_odom, float) - np.asarray(self.origin, float)
        xy = np.asarray(self.launch_xy) + _rz(self.theta)[:2, :2] @ p[:2]
        return np.array([xy[0], xy[1], p[2] + self.launch_height])

    def to_odom(self, p_course):
        p = np.asarray(p_course, float)
        xy = _rz(-self.theta)[:2, :2] @ (p[:2] - np.asarray(self.launch_xy))
        return np.array([xy[0], xy[1], p[2] - self.launch_height]) + np.asarray(self.origin, float)

    def yaw_to_course(self, yaw_odom):
        a = yaw_odom + self.theta
        return float(np.arctan2(np.sin(a), np.cos(a)))

    def yaw_to_odom(self, yaw_course):
        a = yaw_course - self.theta
        return float(np.arctan2(np.sin(a), np.cos(a)))

    def odometry_to_course(self, position, quat_xyzw):
        """nav_msgs/Odometry pose -> (course xyz, course yaw, body-FLU -> course rotation)."""
        r_odom = quat_to_matrix(*quat_xyzw)
        rotation = _rz(self.theta) @ r_odom
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
