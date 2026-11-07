"""Frame math and command handling of the ROS node (no ROS needed)."""
from math import cos, sin

import numpy as np
import pytest

from pathplan.online import Command
from pathplan.ros_node import OdomFrame, parse_state, quat_to_matrix, target_from_command

FRAME = OdomFrame(launch_xy=(-2.515, .914), launch_heading=np.pi / 2, launch_height=.05)


def q_yaw(yaw):
    """ROS (x, y, z, w) quaternion for a pure yaw."""
    return (0.0, 0.0, sin(yaw / 2), cos(yaw / 2))


def test_startup_pose_is_the_launch_mark_facing_up_the_corridor():
    xyz, yaw, rot = FRAME.odometry_to_course((0.0, 0.0, 0.0), q_yaw(0.0))
    assert np.allclose(xyz, [-2.515, .914, .05])
    assert np.isclose(yaw, np.pi / 2)
    assert np.allclose(rot[:, 0], [0, 1, 0], atol=1e-12)        # body forward = course +y


def test_odom_forward_is_course_heading():
    # 1 m along odom +x (the vehicle's forward at startup) = 1 m up the first corridor
    xyz, _, _ = FRAME.odometry_to_course((1.0, 0.0, .85), q_yaw(0.0))
    assert np.allclose(xyz, [-2.515, 1.914, .9])
    # odom +y is the vehicle's left at startup = course -x when facing +y
    xyz, _, _ = FRAME.odometry_to_course((0.0, 1.0, 0.0), q_yaw(0.0))
    assert np.allclose(xyz[:2], [-3.515, .914])


@pytest.mark.parametrize("p", [(0.0, 0.0, .9), (1.2, -2.0, .5), (-2.0, 2.5, 1.4)])
def test_course_odom_round_trip(p):
    assert np.allclose(FRAME.to_course(FRAME.to_odom(p)), p)


@pytest.mark.parametrize("yaw", [0.0, 1.0, -2.5, 3.0])
def test_yaw_round_trip_and_rotation_agree(yaw):
    assert np.isclose(np.cos(FRAME.yaw_to_course(FRAME.yaw_to_odom(yaw)) - yaw), 1.0)
    _, y, rot = FRAME.odometry_to_course((0, 0, 0), q_yaw(FRAME.yaw_to_odom(yaw)))
    assert np.isclose(np.cos(y - yaw), 1.0)
    assert np.allclose(rot[:2, 0], [cos(yaw), sin(yaw)])


def test_quaternion_matrix_is_a_rotation():
    r = quat_to_matrix(.1, -.2, .3, .9)
    assert np.allclose(r @ r.T, np.eye(3)) and np.isclose(np.linalg.det(r), 1.0)
    with pytest.raises(ValueError):
        quat_to_matrix(0, 0, 0, 0)


def test_target_from_command():
    pos = np.array([0.0, 0.0])
    path = np.array([[0.0, 0.0], [1.0, 0.0], [1.0, 1.0]])
    xy, yaw = target_from_command(Command("move", path, .3), pos, 0.0, .5)
    assert np.allclose(xy, [.5, 0]) and yaw == .3
    xy, yaw = target_from_command(Command("move", path, None), pos, 0.0, 1.5)
    assert np.allclose(xy, [1.0, .5]) and np.isclose(yaw, np.arctan2(.5, 1.0))
    xy, yaw = target_from_command(Command("turn", yaw=1.0), pos, 0.0, .5)
    assert np.allclose(xy, pos) and yaw == 1.0               # turning holds position
    xy, yaw = target_from_command(Command("failed"), pos, .2, .5)
    assert np.allclose(xy, pos) and yaw == .2                # failure holds position and heading


def test_parse_state():
    assert parse_state('{"offboard_active": true, "clamped": []}')["offboard_active"] is True
    assert parse_state("not json") == {} and parse_state("[1, 2]") == {}


def test_node_wiring_published_course_depth_to_command():
    """build_planner + project_depth + OnlinePlanner, as the node chains them, on the real course prior."""
    from pathplan.depth_projection import CameraMount, Intrinsics, project_depth
    from pathplan.ros_node import build_planner
    planner, spec, env, prior = build_planner()
    intr = Intrinsics(300., 300., 160., 100., 320, 200)
    # startup pose in odom, lifted to flight height; an unmapped "wall" 1.2 m ahead
    xyz, yaw, rot = FRAME.odometry_to_course((0.0, 0.0, .85), q_yaw(0.0))
    depth = np.full((200, 320), 1.2)
    planner.observe(*project_depth(depth, intr, xyz, rot, spec, expected=prior))
    cmd = planner.update(xyz[:2], yaw)
    assert cmd.mode in ("move", "turn")
    forward = CameraMount().offset[0]                     # camera sits this far ahead of the body origin
    assert planner.belief.occ[tuple(spec.to_cell([-2.515, .914 + forward + 1.2]))]   # the wall is in the map
