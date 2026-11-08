"""Closed loop, no ROS: the real Mission + real planner fly the published course through a model of
flight_interface + PX4, with the pilot's steps scripted (see dronehacks_pathplan/dry_run.py).
~4 s per flight."""
import pytest

from dronehacks_pathplan.dry_run import run


def flown_ok(log):
    assert log["final_state"] == "DONE", log["result"]
    assert len(log["cleared"]) == 5, log["cleared"]
    assert log["pad_error_m"] < .10                         # the pilot's Land, over the pad centre
    assert log["max_sp_yaw_rate"] <= .25 + 1e-6             # heading limit for VIO
    assert log["min_clearance"] >= .30                      # centre never within 0.30 m of a published wall
    assert log["setpoints_while_not_ready"] == 0
    assert log["flight_s"] < 175


@pytest.mark.parametrize("depth", [True, False])
def test_published_course(depth):
    flown_ok(run(depth=depth, verbose=False))


def test_short_odometry_dropout_is_ridden_out():
    flown_ok(run(dropout=(30, 1.0), verbose=False))


def test_drone_carried_to_the_mark_after_slam_started():
    flown_ok(run(carry=(.6, -.3, 25), verbose=False))


def test_long_odometry_loss_faults_and_asks_for_hold():
    log = run(dropout=(30, 4.0), verbose=False)
    assert "FAULT" in log["states"] and log["hold_requests"] == 1
    assert log["final_state"] == "DONE" and log["result"].startswith("FAULT")


def test_pilot_takeover_stops_the_mission():
    log = run(takeover_s=20, verbose=False)
    assert "TAKEN_OVER" in log["states"] and "PX4 left Offboard" in log["result"]
    assert len(log["cleared"]) < 5
