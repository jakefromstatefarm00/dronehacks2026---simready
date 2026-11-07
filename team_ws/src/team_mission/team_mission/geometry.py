"""Small frame/rotation helpers shared by the drone node and the training environment.

Conventions (the same as the skybeta stack):
  * World vectors are ENU in cuVSLAM's ``odom`` frame: x = the drone's nose direction at boot,
    y = left of that, z = up. Origin = where base_link (the Pixhawk) was at boot.
  * Quaternions are ROS order (x, y, z, w).
  * Yaw is counter-clockwise from +x, radians.
  * The "heading frame" H is the world frame rotated by the drone's yaw: x = forward (level),
    y = left, z = up. Observations and actions live in H so a policy does not care which way
    the course happens to point.
"""
from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np


def wrap_pi(a: float) -> float:
    return (a + math.pi) % (2.0 * math.pi) - math.pi


def yaw_from_quat(q) -> float:
    x, y, z, w = (float(v) for v in q)
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def quat_from_yaw(yaw: float):
    return (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))


def rotate_by_quat(q, v) -> np.ndarray:
    """Rotate vector v by unit quaternion q = (x, y, z, w)."""
    x, y, z, w = (float(c) for c in q)
    u = np.array([x, y, z])
    v = np.asarray(v, float)
    return 2.0 * np.dot(u, v) * u + (w * w - np.dot(u, u)) * v + 2.0 * w * np.cross(u, v)


def rot_z(yaw: float) -> np.ndarray:
    c, s = math.cos(yaw), math.sin(yaw)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def world_to_heading(v, yaw: float) -> np.ndarray:
    """World (ENU) vector -> heading frame H (x forward, y left, z up)."""
    return rot_z(-yaw) @ np.asarray(v, float)


def heading_to_world(v, yaw: float) -> np.ndarray:
    return rot_z(yaw) @ np.asarray(v, float)


def clip_norm(v, max_norm: float) -> np.ndarray:
    v = np.asarray(v, float)
    n = float(np.linalg.norm(v))
    return v if n <= max_norm or n == 0.0 else v * (max_norm / n)


@dataclass
class CourseFrame:
    """Where cuVSLAM's odom origin sits in the published-course frame.

    x, y, z: base_link position at boot, in course metres. yaw: nose direction at boot, in the
    course frame (radians, CCW from course +x). All zeros means course == odom.
    """
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0
    yaw: float = 0.0

    def course_to_odom(self, p) -> np.ndarray:
        p = np.asarray(p, float)
        return rot_z(-self.yaw) @ (p - np.array([self.x, self.y, self.z]))

    def odom_to_course(self, p) -> np.ndarray:
        return rot_z(self.yaw) @ np.asarray(p, float) + np.array([self.x, self.y, self.z])

    def yaw_course_to_odom(self, yaw: float) -> float:
        return wrap_pi(yaw - self.yaw)
