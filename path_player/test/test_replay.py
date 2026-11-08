"""Unit tests for path_player core replay logic (no ROS required)."""
import math
import sys
from pathlib import Path

import numpy as np
import pytest

# Ensure path_player and path_recorder are in sys.path
WORKSPACE = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(WORKSPACE / "path_player"))
sys.path.insert(0, str(WORKSPACE / "path_recorder"))

from path_player.replay import (
    TrajectoryPlayer,
    TrajectoryPoint,
    arm_to_odom,
    load_recording,
    odom_to_arm,
    quaternion_from_yaw,
    wrap,
    yaw_of,
)
from path_recorder.recording import Recording


def make_test_recording(folder: Path, poses, anchor_pos=(1.0, 2.0, 0.1), anchor_yaw=math.pi / 2):
    """Helper to generate a real path_recorder dataset using Recording class."""
    half = anchor_yaw / 2.0
    q0 = (0.0, 0.0, math.sin(half), math.cos(half))
    rec = Recording(folder, anchor_pos, q0, spacing_m=0.15, turn_deg=15.0)

    t = 0.0
    for pos, yaw in poses:
        qh = (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))
        rec.odometry(t, pos, qh, landed=False, nav_state=2, clear=(1.0, 2.0, 3.0), depth_age=0.05)
        t += 0.1
    rec.close("disarm")
    return rec


def test_arm_to_odom_is_exact_inverse_of_path_recorder(tmp_path):
    """Verify arm_to_odom produces identical results to path_recorder Recording.to_odom()."""
    anchor_pos = (1.5, 3.2, 0.25)
    anchor_yaw = math.radians(45.0)

    rec = Recording(tmp_path / "rec", anchor_pos, quaternion_from_yaw(anchor_yaw))

    test_rel_points = [
        ([0.0, 0.0, 0.0], 0.0),
        ([1.0, 0.0, 0.5], math.radians(30.0)),
        ([0.0, 1.0, 1.2], math.radians(-45.0)),
        ([2.5, -1.2, 0.8], math.radians(180.0)),
    ]

    for rel_xyz, rel_yaw in test_rel_points:
        # Expected from path_recorder
        expected_pos, expected_yaw = rec.to_odom(rel_xyz, rel_yaw)

        # Computed by path_player
        calc_pos, calc_yaw = arm_to_odom(anchor_pos, anchor_yaw, rel_xyz, rel_yaw)

        assert np.allclose(calc_pos, expected_pos, atol=1e-6)
        assert calc_yaw == pytest.approx(expected_yaw, abs=1e-6)

        # Inverse check: odom_to_arm
        back_rel, back_yaw = odom_to_arm(anchor_pos, anchor_yaw, calc_pos, calc_yaw)
        assert np.allclose(back_rel, rel_xyz, atol=1e-6)
        assert back_yaw == pytest.approx(wrap(rel_yaw), abs=1e-6)


def test_quaternion_and_yaw_conversions():
    test_angles = [0.0, math.pi / 4, -math.pi / 3, math.pi, -math.pi + 0.01]
    for yaw in test_angles:
        q = quaternion_from_yaw(yaw)
        assert q[0] == 0.0 and q[1] == 0.0
        reconstructed_yaw = yaw_of(*q)
        assert reconstructed_yaw == pytest.approx(wrap(yaw), abs=1e-6)


def test_load_recording(tmp_path):
    poses = [
        ((1.0, 2.0, 0.1), math.pi / 2),
        ((1.0, 3.0, 0.6), math.pi / 2),
        ((1.0, 4.0, 1.1), math.pi / 2),
    ]
    rec_folder = tmp_path / "test_rec"
    make_test_recording(rec_folder, poses, anchor_pos=(1.0, 2.0, 0.1), anchor_yaw=math.pi / 2)

    meta, points = load_recording(rec_folder)
    assert meta["status"] == "complete"
    assert len(points) >= 3

    p0 = points[0]
    assert p0.idx == 0
    assert p0.x == pytest.approx(0.0, abs=1e-3)
    assert p0.y == pytest.approx(0.0, abs=1e-3)
    assert p0.z == pytest.approx(0.0, abs=1e-3)

    p_last = points[-1]
    # At (1.0, 4.0, 1.1) with anchor facing +y (90 deg), relative x (forward) is 2.0m, y is 0, z is 1.0m
    assert p_last.x == pytest.approx(2.0, abs=1e-2)
    assert p_last.y == pytest.approx(0.0, abs=1e-2)
    assert p_last.z == pytest.approx(1.0, abs=1e-2)


def test_trajectory_player_time_interpolation():
    points = [
        TrajectoryPoint(idx=0, t=0.0, x=0.0, y=0.0, z=1.0, yaw=0.0),
        TrajectoryPoint(idx=1, t=2.0, x=2.0, y=0.0, z=1.0, yaw=math.pi / 2),
    ]

    # Replaying with anchor at origin facing east (0 rad)
    player = TrajectoryPlayer(points, anchor_pos=(0, 0, 0), anchor_yaw=0.0, mode="time", speed_factor=1.0)
    player.start(now_s=100.0)

    # At t = 1.0s (midpoint of segment)
    pos, yaw, info = player.update(now_s=101.0)
    assert np.allclose(pos, [1.0, 0.0, 1.0], atol=1e-3)
    assert yaw == pytest.approx(math.pi / 4, abs=1e-3)
    assert not player.is_finished()

    # At t = 2.0s (end of segment)
    pos, yaw, info = player.update(now_s=102.0)
    assert np.allclose(pos, [2.0, 0.0, 1.0], atol=1e-3)
    assert yaw == pytest.approx(math.pi / 2, abs=1e-3)
    assert player.is_finished()


def test_trajectory_player_speed_factor():
    points = [
        TrajectoryPoint(idx=0, t=0.0, x=0.0, y=0.0, z=1.0, yaw=0.0),
        TrajectoryPoint(idx=1, t=2.0, x=2.0, y=0.0, z=1.0, yaw=0.0),
    ]

    # Speed factor = 2.0: should reach end in 1.0 wall-clock second instead of 2.0s
    player = TrajectoryPlayer(points, anchor_pos=(0, 0, 0), anchor_yaw=0.0, mode="time", speed_factor=2.0)
    player.start(now_s=10.0)

    pos, yaw, info = player.update(now_s=11.0)
    assert np.allclose(pos, [2.0, 0.0, 1.0], atol=1e-3)
    assert player.is_finished()


def test_trajectory_player_adaptive_lag_cap():
    points = [
        TrajectoryPoint(idx=0, t=0.0, x=0.0, y=0.0, z=1.0, yaw=0.0),
        TrajectoryPoint(idx=1, t=10.0, x=10.0, y=0.0, z=1.0, yaw=0.0),
    ]

    # Max lag = 0.5m
    player = TrajectoryPlayer(points, anchor_pos=(0, 0, 0), anchor_yaw=0.0, mode="adaptive", max_lag_m=0.5)
    player.start(now_s=0.0)

    # Step 1: Drone starts at (0.0, 0.0, 1.0), within lag limit of start
    pos1, _, info1 = player.update(now_s=0.2, current_pos_odom=(0.15, 0.0, 1.0))
    assert np.allclose(pos1, [0.2, 0.0, 1.0], atol=1e-2)
    assert info1["status"] == "PLAYING"

    # Step 2: Drone advances smoothly
    pos2, _, info2 = player.update(now_s=0.5, current_pos_odom=(0.4, 0.0, 1.0))
    assert np.allclose(pos2, [0.5, 0.0, 1.0], atol=1e-2)
    assert info2["status"] == "PLAYING"

    # Step 3: Drone gets stuck at (0.5, 0.0, 1.0) while time advances to 1.5s
    # When lag exceeds 0.5m, time freezes
    pos3, _, info3 = player.update(now_s=1.5, current_pos_odom=(0.5, 0.0, 1.0))
    assert info3["status"] == "LAG_PAUSED"

    # Step 4: Drone catches up to (0.9, 0.0, 1.0); playback resumes
    pos4, _, info4 = player.update(now_s=1.6, current_pos_odom=(0.9, 0.0, 1.0))
    assert info4["status"] == "PLAYING"


def test_trajectory_player_anchor_shift():
    """Verify changing anchor shifts all setpoints correctly."""
    points = [
        TrajectoryPoint(idx=0, t=0.0, x=1.0, y=0.0, z=0.5, yaw=0.0),
    ]

    # Replay with anchor at (10, 20, 1), yaw = 90 deg (pi/2)
    player = TrajectoryPlayer(points, anchor_pos=(10.0, 20.0, 1.0), anchor_yaw=math.pi / 2, mode="waypoint")
    player.start(now_s=0.0)

    # In arm frame, (1, 0, 0.5) with anchor at (10, 20, 1) facing +y should be at odom (10, 21, 1.5), yaw = pi/2
    pos, yaw, _ = player.update(now_s=0.1)
    assert np.allclose(pos, [10.0, 21.0, 1.5], atol=1e-3)
    assert yaw == pytest.approx(math.pi / 2, abs=1e-3)

    # Shift anchor to (5, 5, 0), yaw = 0
    player.set_anchor((5.0, 5.0, 0.0), 0.0)
    pos2, yaw2, _ = player.update(now_s=0.2)
    assert np.allclose(pos2, [6.0, 5.0, 0.5], atol=1e-3)
    assert yaw2 == pytest.approx(0.0, abs=1e-3)


def test_find_latest_recording(tmp_path):
    d1 = tmp_path / "path-20261108-100000-001"
    d2 = tmp_path / "path-20261108-110000-002"
    make_test_recording(d1, [((0, 0, 0), 0)])
    make_test_recording(d2, [((0, 0, 0), 0)])

    from path_player.replay import find_latest_recording
    latest = find_latest_recording(tmp_path)
    assert latest == d2


def test_trajectory_player_waypoint_mode():
    points = [
        TrajectoryPoint(idx=0, t=0.0, x=0.0, y=0.0, z=1.0, yaw=0.0),
        TrajectoryPoint(idx=1, t=1.0, x=1.0, y=0.0, z=1.0, yaw=0.0),
        TrajectoryPoint(idx=2, t=2.0, x=2.0, y=0.0, z=1.0, yaw=0.0),
    ]

    player = TrajectoryPlayer(points, anchor_pos=(0, 0, 0), anchor_yaw=0.0, mode="waypoint", arrival_radius_m=0.2)
    player.start(now_s=0.0)

    # Far from waypoint 0: should stay on waypoint 0
    pos, _, info = player.update(now_s=0.1, current_pos_odom=(10.0, 0.0, 1.0))
    assert np.allclose(pos, [0.0, 0.0, 1.0])
    assert info["idx"] == 0

    # Arrive at waypoint 0: should advance to waypoint 1
    pos, _, info = player.update(now_s=0.2, current_pos_odom=(0.05, 0.0, 1.0))
    assert np.allclose(pos, [1.0, 0.0, 1.0])
    assert info["idx"] == 1

    # Arrive at waypoint 1: should advance to waypoint 2
    pos, _, info = player.update(now_s=0.3, current_pos_odom=(1.05, 0.0, 1.0))
    assert np.allclose(pos, [2.0, 0.0, 1.0])
    assert info["idx"] == 2

    # Arrive at final waypoint: finished
    pos, _, info = player.update(now_s=0.4, current_pos_odom=(2.02, 0.0, 1.0))
    assert player.is_finished()
