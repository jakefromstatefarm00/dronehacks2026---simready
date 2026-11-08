"""path_recorder core, no ROS: frames, spacing, turn points, flags, files, clearance."""
import json
import math
import sys
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from path_recorder.recording import Recording, clearance, decode_depth, load  # noqa: E402


def q(yaw):
    return (0.0, 0.0, math.sin(yaw / 2), math.cos(yaw / 2))


def fly(rec, poses, dt=1 / 30, landed=False, **kw):
    t = 0.0
    for pos, yaw in poses:
        rec.odometry(t, pos, q(yaw), landed=landed, nav_state=2, clear=(1.0, 2.0, 3.0), depth_age=.05, **kw)
        t += dt
    return t


def test_arm_frame_is_relative_to_the_arming_pose(tmp_path):
    # armed at odom (1, 2, 0.1) facing 90 deg (odom +y): 1 m along that heading = x +1 in the arm frame
    rec = Recording(tmp_path / "r", (1.0, 2.0, .1), q(math.pi / 2))
    rel, ryaw = rec.to_rel((1.0, 3.0, .6), q(math.pi))
    assert np.allclose(rel, [1.0, 0.0, .5]) and ryaw == pytest.approx(math.pi / 2)
    back, yaw = rec.to_odom(rel, ryaw)
    assert np.allclose(back, [1.0, 3.0, .6]) and yaw == pytest.approx(math.pi)


def test_points_every_15_cm_plus_turns_and_an_end_point(tmp_path):
    rec = Recording(tmp_path / "r", (0, 0, 0), q(0))
    straight = [((0.01 * i, 0.0, .9), 0.0) for i in range(101)]            # 1 m forward, 1 cm steps
    turn = [((1.0, 0.0, .9), math.radians(2 * i)) for i in range(1, 46)]   # 90 deg turn in place
    fly(rec, straight + turn)
    rec.close("disarm")
    meta, rows = load(tmp_path / "r")
    reasons = [r["reason"] for r in rows]
    assert reasons[0] == "start" and reasons[-1] == "end"
    assert reasons.count("distance") == 6                                   # 0.15 .. 0.90 m
    assert reasons.count("turn") >= 5                                        # every 15 deg of the turn
    assert rows[-1]["yaw"] == pytest.approx(math.pi / 2, abs=1e-3)           # final heading kept (end point)
    steps = [math.dist((a["x"], a["y"]), (b["x"], b["y"])) for a, b in zip(rows, rows[1:]) if b["reason"] == "distance"]
    assert all(.15 <= s < .17 for s in steps)
    assert meta["status"] == "complete" and meta["points"] == len(rows)
    assert meta["length_m"] == pytest.approx(1.0, abs=.02)


def test_each_row_has_what_skybeta_setpoint_pose_needs(tmp_path):
    rec = Recording(tmp_path / "r", (0, 0, 0), q(0))
    fly(rec, [((0.0, 0.0, 0.0), 0.0), ((0.3, 0.2, .5), math.radians(40))])
    rec.close()
    _, rows = load(tmp_path / "r")
    r = rows[1]
    assert (r["x"], r["y"], r["z"]) == (0.3, 0.2, 0.5)
    assert math.atan2(2 * r["qw"] * r["qz"], 1 - 2 * r["qz"] ** 2) == pytest.approx(math.radians(40), abs=1e-4)
    assert r["qx"] == 0.0 and r["qy"] == 0.0
    assert (r["clear_left"], r["clear_center"], r["clear_right"]) == (1.0, 2.0, 3.0)


def test_flags_gap_jump_depth_and_px4(tmp_path):
    rec = Recording(tmp_path / "r", (0, 0, 0), q(0))
    rec.odometry(0.0, (0, 0, 0), q(0), landed=True, nav_state=2, depth_age=None)
    rec.odometry(1.0, (.5, 0, 0), q(0), landed=False, nav_state=2, depth_age=.1, gap=True, jump=True, local_valid=False)
    rec.close()
    meta, rows = load(tmp_path / "r")
    assert "depth_stale" in rows[0]["flags"]
    assert set(rows[1]["flags"].split(";")) == {"odom_gap", "odom_jump", "px4_no_position"}
    assert meta["flags"]["odom_jump"] == 1


def test_files_are_usable_while_still_recording(tmp_path):
    rec = Recording(tmp_path / "r", (0, 0, 0), q(0))
    fly(rec, [((0.01 * i, 0, .9), 0.0) for i in range(200)], dt=.1)        # 20 s: meta refreshed on the way
    meta, rows = load(tmp_path / "r")                                       # no close(): a crash mid-flight
    assert meta["status"] == "recording" and len(rows) >= 12 and meta["points"] >= 10


def test_short_flight_is_marked(tmp_path):
    rec = Recording(tmp_path / "r", (0, 0, 0), q(0))
    fly(rec, [((0, 0, 0), 0.0)] * 30, landed=True)
    rec.close()
    meta, _ = load(tmp_path / "r")
    assert meta["aborted_or_short"] is True


def test_clearance_from_a_depth_image():
    d = np.full((400, 640), 4.0, np.float32)
    d[:, :200] = 1.2                                     # something close on the left
    d[:, 300:340] = 0.0                                   # no data stripe in the middle (ignored)
    d[300:, :] = 0.5                                      # floor below the band (ignored)
    left, centre, right = clearance(d)
    assert left == pytest.approx(1.2) and centre == pytest.approx(4.0) and right == pytest.approx(4.0)
    assert clearance(np.zeros((400, 640), np.float32)) == (None, None, None)


def test_decode_depth_16uc1_millimetres_and_32fc1_metres():
    mm = (np.arange(12, dtype="<u2").reshape(3, 4) * 1000)
    assert np.allclose(decode_depth(mm.tobytes(), 4, 3, "16UC1", 8), mm / 1000.0)
    m = np.arange(12, dtype="<f4").reshape(3, 4)
    assert np.allclose(decode_depth(m.tobytes(), 4, 3, "32FC1", 16), m)
    with pytest.raises(ValueError):
        decode_depth(b"", 1, 1, "rgb8", 3)
