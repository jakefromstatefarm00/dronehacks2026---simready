"""Geometry, checkpoints and the observation/action contract. Pure numpy: runs anywhere."""
import json
import math
import os

import numpy as np
import pytest

from team_mission.control import (ACT_DIM, OBS_DIM, OBS_VERSION, ActionMapper, CarrotPolicy, Limits, MLPPolicy,
                                  PolicyContext, build_observation, speed_scale)
from team_mission.geometry import CourseFrame, quat_from_yaw, rotate_by_quat, yaw_from_quat
from team_mission.path import CheckpointSpec, Path, resolve_path

PKG = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
CP = os.path.join(PKG, 'config', 'checkpoints')


def test_all_checkpoint_files_load():
    for f in os.listdir(CP):
        spec = CheckpointSpec.load(os.path.join(CP, f))
        assert spec.checkpoints and spec.frame in ('course', 'start')


def test_course_frame_round_trip():
    cf = CourseFrame(-2.515, 0.914, 0.05, math.radians(90))
    p = np.array([1.0, -2.0, 0.8])
    assert np.allclose(cf.odom_to_course(cf.course_to_odom(p)), p)
    # 1 m along the boot nose direction (odom +x) is course +y when the nose points up course +y
    assert np.allclose(cf.odom_to_course([1.0, 0, 0]), [-2.515, 1.914, 0.05])


def test_start_frame_follows_takeoff_heading():
    spec = CheckpointSpec.from_dict({'frame': 'start', 'checkpoints': [{'xyz': [1, 0, 1]}]})
    P = resolve_path(spec, [2.0, 3.0, 0.1], math.radians(90), 1.0)
    assert np.allclose(P.points[0], [2.0, 3.0, 1.1])          # takeoff point
    assert np.allclose(P.points[1], [2.0, 4.0, 1.1])          # 1 m "forward" = +y here


def test_path_geometry_and_reached():
    P = Path(np.array([[0, 0, 1], [2, 0, 1], [2, 2, 1.0]]), np.array([0, 0.2, 0.2]), np.array([0.3, 0.3, 0.3]))
    assert P.length == pytest.approx(4.0)
    assert P.progress([1.0, 0.5, 1.0], 1) == pytest.approx(1.0)
    assert np.allclose(P.lateral([1.0, 0.5, 1.0], 1), [0, 0.5, 0])
    assert P.reached([1.85, 0.05, 1.0], 1)
    assert not P.reached([1.5, 0.0, 1.0], 1)
    assert P.reached([2.1, 0.1, 1.0], 1)                       # slightly past it, close sideways
    d, c = P.off_path([1.9, 0.0, 1.0], 2)                      # just reached the corner: still near the old line
    assert d == pytest.approx(0.0, abs=1e-9)


def test_observation_is_heading_relative():
    P = Path(np.array([[0, 0, 1], [1, 0, 1], [1, 1, 1.0]]), np.array([0, 0.2, 0.2]), np.array([0.3, 0.3, 0.3]))
    L = Limits()
    o1 = build_observation([0, 0, 1], [0.2, 0, 0], 0.0, P, 1, np.zeros(4), L)
    # rotate the whole world by 90 deg: same observation
    R = np.array([[0, -1, 0], [1, 0, 0], [0, 0, 1.0]])
    P2 = Path(P.points @ R.T, P.radius, P.corridor)
    o2 = build_observation([0, 0, 1], R @ [0.2, 0, 0], math.pi / 2, P2, 1, np.zeros(4), L)
    assert o1.shape == (OBS_DIM,) and np.allclose(o1, o2, atol=1e-6)
    assert o1[0] == pytest.approx(0.5)                          # 1 m ahead / 2
    assert o1[18] == 0.0 and build_observation([1, 0.9, 1], [0, 0, 0], 0, P, 2, np.zeros(4), L)[18] == 1.0


def test_action_mapping_is_slow_and_bounded():
    L = Limits()
    m = ActionMapper(L)
    cmd = m.command([1, 1, 1, 1], [0, 0, 1], math.pi / 2)
    v = cmd.velocity
    assert np.linalg.norm(v[:2]) <= L.max_xy_speed + 1e-9
    assert np.linalg.norm(cmd.carrot - [0, 0, 1]) <= L.max_carrot_m + 1e-9
    fwd = m.command([1, 0, 0, 0], [0, 0, 1], math.pi / 2).velocity     # facing +y: forward = +y
    assert np.allclose(fwd, [0, L.max_xy_speed, 0], atol=1e-9)
    assert np.all(np.isfinite(m.command([np.nan] * 4, [0, 0, 1], 0).carrot))
    P = Path(np.array([[0, 0, 1], [1, 0, 1.0]]), np.array([0, 0.2]), np.array([0.18, 0.18]))
    assert speed_scale(P, 1) == pytest.approx(0.6)


def test_carrot_policy_heads_for_the_checkpoint():
    P = Path(np.array([[0, 0, 1], [2, 0, 1.0], [2, 2, 1.0]]), np.array([0, 0.2, 0.2]), np.array([0.3] * 3))
    a = CarrotPolicy().act(None, PolicyContext(np.array([0, 0, 1.0]), np.zeros(3), 0.0, P, 1, Limits()))
    assert a.shape == (ACT_DIM,) and a[0] > 0.5 and abs(a[1]) < 0.1


def test_mlp_policy_numpy_inference(tmp_path):
    rng = np.random.default_rng(0)
    W = [rng.normal(0, 0.3, (8, OBS_DIM)), rng.normal(0, 0.3, (ACT_DIM, 8))]
    b = [np.zeros(8), np.zeros(ACT_DIM)]
    meta = dict(obs_version=OBS_VERSION, obs_dim=OBS_DIM, act_dim=ACT_DIM, n_layers=2)
    f = tmp_path / 'p.npz'
    np.savez(f, meta=np.array(json.dumps(meta)), W0=W[0], b0=b[0], W1=W[1], b1=b[1])
    p = MLPPolicy(str(f))
    x = rng.uniform(-1, 1, OBS_DIM)
    assert np.allclose(p.network(x), W[1] @ np.tanh(W[0] @ x))
    bad = dict(meta, obs_version=OBS_VERSION - 1)
    np.savez(f, meta=np.array(json.dumps(bad)), W0=W[0], b0=b[0], W1=W[1], b1=b[1])
    with pytest.raises(ValueError):
        MLPPolicy(str(f))


def test_shipped_policies_load():
    pol_dir = os.path.join(PKG, 'policies')
    for f in os.listdir(pol_dir) if os.path.isdir(pol_dir) else []:
        if f.endswith('.npz'):
            p = MLPPolicy(os.path.join(pol_dir, f))
            assert p.network(np.zeros(OBS_DIM)).shape == (ACT_DIM,)


def test_quaternion_helpers():
    q = quat_from_yaw(0.7)
    assert yaw_from_quat(q) == pytest.approx(0.7)
    assert np.allclose(rotate_by_quat(q, [1, 0, 0]), [math.cos(0.7), math.sin(0.7), 0])
