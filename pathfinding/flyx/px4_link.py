"""PX4 frames and commands for the FlyX lab (pure Python; no ROS import).

Frames
------
PX4 local frame: NED (x north, y east, z down); attitude q = FRD body -> NED, (w, x, y, z).
Course frame:    the published map (x, y on the floor, z up, origin at the floor centre).
The link between them is ``CourseFrame`` below (from Suraj's ``dronebench.teammate_policy``), built
once from the drone's pose on the launch mark (reviewed placement + launch heading).
"""
from __future__ import annotations

from dataclasses import dataclass
from math import atan2, cos, sin

import numpy as np

def rotation_wxyz(q):
    """Rotation matrix of a unit FRD->NED quaternion (w, x, y, z)."""
    q = np.asarray(q, dtype=float)
    if q.shape != (4,) or not np.isfinite(q).all() or abs(np.linalg.norm(q) - 1) > 1e-4:
        raise ValueError("finite unit FRD-to-NED quaternion required")
    w, x, y, z = q
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


@dataclass
class CourseFrame:
    """PX4 local NED <-> course frame (x east-ish, y up the first corridor, z up), fixed at launch."""
    ned_to_course: np.ndarray
    origin_in_course: np.ndarray
    north_heading_in_course: float

    @classmethod
    def from_launch(cls, *, position_ned, heading_ned, launch_xy, body_agl,
                    physical_heading_in_course, placement_reviewed=False):
        values = np.asarray([*position_ned, heading_ned, *launch_xy, body_agl, physical_heading_in_course])
        if not placement_reviewed or values.shape != (8,) or not np.isfinite(values).all() or body_agl <= 0:
            raise ValueError("reviewed physical launch alignment and actual pose/range required")
        # NED yaw is clockwise viewed from above; course yaw is CCW/Z-up: course_heading = north_heading - NED_heading
        north = physical_heading_in_course + heading_ned
        matrix = np.array([[cos(north), sin(north), 0.], [sin(north), -cos(north), 0.], [0., 0., -1.]])
        origin = np.array([*launch_xy, body_agl]) - matrix @ np.asarray(position_ned)
        return cls(matrix, origin, north)

    def to_course(self, position_ned):
        return self.ned_to_course @ np.asarray(position_ned) + self.origin_in_course

    def to_ned(self, position_course):
        return self.ned_to_course.T @ (np.asarray(position_course) - self.origin_in_course)


# px4_msgs constants (PX4 v1.16), kept here so the logic is testable without px4_msgs
CMD_DO_SET_MODE = 176
CMD_ARM_DISARM = 400
CMD_NAV_LAND = 21
CMD_RESULT_ACCEPTED = 0
ARMING_STATE_DISARMED, ARMING_STATE_ARMED = 1, 2
NAV_STATE_OFFBOARD = 14
NAV_STATE_AUTO_LAND = 18
POSE_FRAME_NED = 1


def yaw_from_q_ned(q):
    """Heading (rad, NED: clockwise from north) of an FRD->NED quaternion (w, x, y, z)."""
    w, x, y, z = q
    return atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))


def odom_to_course(frame, position_ned, q_ned_frd):
    """PX4 odometry -> (course xyz, course yaw, body-FLU -> course rotation)."""
    xyz = frame.to_course(position_ned)
    yaw = frame.north_heading_in_course - yaw_from_q_ned(q_ned_frd)
    rotation = frame.ned_to_course @ rotation_wxyz(q_ned_frd) @ np.diag([1.0, -1.0, -1.0])
    return xyz, float(np.arctan2(np.sin(yaw), np.cos(yaw))), rotation


def course_to_ned_setpoint(frame, xy, altitude, yaw_course):
    """Course-frame target -> (NED position, NED yaw) for TrajectorySetpoint."""
    position = frame.to_ned(np.array([xy[0], xy[1], altitude]))
    yaw_ned = frame.north_heading_in_course - yaw_course
    return position, float(np.arctan2(np.sin(yaw_ned), np.cos(yaw_ned)))


@dataclass(frozen=True)
class Command:
    """One VehicleCommand to send (fields as in px4_msgs/VehicleCommand)."""
    command: int
    param1: float = 0.0
    param2: float = 0.0

    @staticmethod
    def offboard():
        return Command(CMD_DO_SET_MODE, 1.0, 6.0)          # custom mode, PX4 main mode OFFBOARD

    @staticmethod
    def arm():
        return Command(CMD_ARM_DISARM, 1.0)

    @staticmethod
    def land():
        return Command(CMD_NAV_LAND)


