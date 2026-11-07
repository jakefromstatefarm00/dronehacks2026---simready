"""Residual: bounds, zero-on-stale, --no-residual == base, generated files carry the original.

    python -m pytest -q residual/tests
"""
import importlib.util
import json
import math
import pickle
import sys
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest

REPO = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO))

from residual import runtime as rt          # noqa: E402
from residual import sim                    # noqa: E402

JC = REPO / "jolie_course"


def _load(path, name):
    spec = importlib.util.spec_from_file_location(name, path)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


def _random_policy(rng, scale=3.0):
    sizes = [rt.OBS_DIM, 16, 3]
    n = sum(i * o + o for i, o in zip(sizes[:-1], sizes[1:]))
    pol = rt.MLPPolicy.from_flat(rng.normal(0, scale, n), sizes)
    return {"format": rt.RESIDUAL_FORMAT, "obs_dim": rt.OBS_DIM, "layers": [(w.tolist(), b.tolist()) for w, b in pol.layers]}


def test_action_mapping_bounded_and_zero_is_base():
    assert rt.action_to_residual([0, 0, 0]) == rt.ZERO
    for a in np.random.default_rng(0).normal(0, 10, (2000, 3)):
        s, lat, y = rt.action_to_residual(a)
        assert rt.CARROT_SCALE_MIN - 1e-12 <= s <= rt.CARROT_SCALE_MAX + 1e-12
        assert abs(lat) <= rt.LATERAL_MAX_M + 1e-12
        assert rt.YAW_SCALE_MIN - 1e-12 <= y <= 1.0


class _Planner:
    cleared, events, pad_xy = [], [], (0.0, 0.0)


def _mission(policy):
    b = sim.base()
    cls = rt.make_residual_mission(b.Mission, policy)
    m = cls(_Planner(), b.MissionConfig(execute=True))
    m.state = "NAVIGATE"
    return m


def _cmd(xy, end, ext=None, lat_ok=(True, True)):
    Command = sys.modules["pathplan.online"].Command
    cmd = Command("move", np.array([xy, end]), math.atan2(end[1] - xy[1], end[0] - xy[0]))
    cmd.residual_info = {"cp_index": 1, "cp_dist": 1.5, "ext": ext, "lat_ok": lat_ok, "probes": [1.2, .8, .9, .5, .6]}
    return cmd


def test_residual_target_stays_within_bounds():
    rng = np.random.default_rng(1)
    for k in range(300):
        m = _mission(_random_policy(rng))
        m._r_now = m._r_odom_t = 10.0
        xy = rng.uniform(-2, 2, 2)
        ang = rng.uniform(-np.pi, np.pi)
        chord = rng.uniform(.05, .5)
        end = xy + chord * np.array([np.cos(ang), np.sin(ang)])
        ext = (end + .15 * np.array([np.cos(ang + .3), np.sin(ang + .3)])).tolist() if k % 2 else None
        cmd = _cmd(xy, end, ext)
        yaw = ang + rng.uniform(-1, 1)
        c0 = m._carrot_for(cmd, yaw)
        base_t = sys.modules["pathplan.ros_node"].target_from_command(cmd, xy, yaw, c0)
        tgt = m._residual_target(cmd, np.r_[xy, .9], yaw, c0, base_t)
        s, lat, y = m.residual
        assert rt.CARROT_SCALE_MIN <= s <= rt.CARROT_SCALE_MAX and abs(lat) <= rt.LATERAL_MAX_M and rt.YAW_SCALE_MIN <= y <= 1
        d = np.linalg.norm(tgt[0] - xy)
        # never farther than the longest carrot plus the lateral offset; heading untouched
        assert d <= c0 * rt.CARROT_SCALE_MAX + rt.LATERAL_MAX_M + 1e-9
        assert tgt[1] == base_t[1]
        if ext is None:                       # no checked extension: never longer than the base
            assert np.linalg.norm(tgt[0] - base_t[0]) <= rt.LATERAL_MAX_M + 1e-9 or s <= 1
            assert d <= np.linalg.norm(base_t[0] - xy) + rt.LATERAL_MAX_M + 1e-9


@pytest.mark.parametrize("why", ["stale", "stall", "no_info", "not_nav", "nan"])
def test_zero_correction_on_stale_or_invalid_inputs(why):
    m = _mission(_random_policy(np.random.default_rng(2)))
    m._r_now, m._r_odom_t = 10.0, 10.0
    xy, end = np.array([0.0, 0.0]), np.array([.5, 0.0])
    cmd = _cmd(xy, end, ext=[.65, 0.0])
    if why == "stale":
        m._r_odom_t = 9.5
    elif why == "stall":
        m._r_last_stall = 9.8
    elif why == "no_info":
        cmd.residual_info = None
    elif why == "not_nav":
        m.state = "TAKEOFF"
    elif why == "nan":
        cmd.residual_info["probes"][0] = float("nan")
    base_t = (end.copy(), 0.0)
    assert m._residual_target(cmd, np.r_[xy, .9], 0.0, .5, base_t) is base_t
    assert m.residual == rt.ZERO


def test_stale_odometry_in_tick_resets_yaw_cap():
    m = _mission(_random_policy(np.random.default_rng(3)))
    m.residual = (1.2, .05, .6)
    b = sim.base()
    m.tick(b.Telemetry(now=5.0, odom_t=4.0))
    assert m.residual == rt.ZERO


def test_planner_info_survives_the_process_pipe():
    Command = sys.modules["pathplan.online"].Command
    cmd = _cmd(np.zeros(2), np.array([.5, 0]))
    back = pickle.loads(pickle.dumps(cmd))
    assert back.residual_info == cmd.residual_info


def test_wrapped_planner_returns_identical_commands():
    """The planner-side wrapper only reads; commands match the bare planner step for step."""
    b = sim.base()
    bare = b._build_planner_embedded()[0]
    wrapped = rt.wrap_build_planner(b._build_planner_embedded, .5)()[0]
    xy, yaw = np.array([-2.515, .914]), math.pi / 2
    for _ in range(40):
        c1, c2 = bare.update(xy, yaw), wrapped.update(xy, yaw)
        assert c1.mode == c2.mode and c1.yaw == c2.yaw
        assert (c1.path is None and c2.path is None) or np.array_equal(c1.path, c2.path)
        if c1.mode == "move":
            xy = sys.modules["pathplan.online"]._point_along(c1.path, .1)
            yaw = c1.yaw
        else:
            yaw = c1.yaw
    assert c2.residual_info is not None and len(c2.residual_info["probes"]) == 5


def test_zero_policy_flies_like_the_base():
    """Closed loop: a residual whose output layer is zero reproduces the base flight."""
    rng = np.random.default_rng(4)
    p = _random_policy(rng)
    p["layers"][-1] = (np.zeros((3, 16)).tolist(), np.zeros(3).tolist())
    a = sim.run_episode("L0", 5, None, residual=False, record=True, max_s=40)
    r = sim.run_episode("L0", 5, p, residual=True, record=True, max_s=40)
    ta, tr = np.array([x[1:5] for x in a.traj], float), np.array([x[1:5] for x in r.traj], float)
    assert len(ta) == len(tr) and np.abs(ta - tr).max() < 1e-6
    assert r.applied > 0


GENERATED = [("upload_to_flyx_residual.py", "upload_to_flyx.py"),
             ("upload_to_flyx_residual_skybeta.py", "upload_to_flyx_skybeta.py")]


@pytest.mark.parametrize("gen, orig", GENERATED)
def test_generated_file_no_residual_is_the_original(gen, orig):
    if not (JC / gen).exists():
        pytest.skip("run jolie_course/build_residual_copy.py first")
    g_text, o_text = (JC / gen).read_text(encoding="utf-8"), (JC / orig).read_text(encoding="utf-8")
    k0, k1 = o_text.index("_SOURCES = {"), o_text.index("\ndef _install_embedded():")
    assert o_text[k0:k1] in g_text                                      # embedded modules byte for byte
    g = _load(JC / gen, "gen_" + gen[:-3])
    o = _load(JC / orig, "orig_" + orig[:-3])
    assert g._SOURCES == o._SOURCES and g._PRIOR == o._PRIOR and g._CHECKPOINTS == o._CHECKPOINTS
    a = g.parse_args(["--no-residual"])
    assert a.no_residual
    cls, build = g._residual_choice(a.no_residual, a.carrot)
    assert cls is g.Mission and build is g._build_planner_embedded      # her objects, untouched
    assert cls is sys.modules["flyx.mission"].Mission
    ref = vars(o.parse_args([]))
    got = vars(g.parse_args(["--no-residual"]))
    got.pop("no_residual")
    ref.pop("output"), got.pop("output")
    assert got == ref                                                   # every other option unchanged
    cls_r, _ = g._residual_choice(False, .5)
    assert issubclass(cls_r, g.Mission) and cls_r is not g.Mission
    policy = json.loads((REPO / "residual" / "policy.json").read_text())
    assert g._RESIDUAL_POLICY["layers"] == policy["layers"]


@pytest.mark.parametrize("gen, orig", GENERATED)
def test_generated_file_differs_only_by_the_insertions(gen, orig):
    if not (JC / gen).exists():
        pytest.skip("run jolie_course/build_residual_copy.py first")
    import difflib
    g = (JC / gen).read_text(encoding="utf-8").splitlines()
    o = (JC / orig).read_text(encoding="utf-8").splitlines()
    g_set = set(g)
    gone = [l[2:] for l in difflib.ndiff(o, g) if l.startswith("- ") and l[2:] not in g_set]
    # the docstring's opening line (its text is kept inside the new header) and exactly two code lines
    assert sorted(x.strip() for x in gone if not x.startswith('"""')) == [
        "build_planner = _build_planner_embedded", "mission = Mission(planner, MissionConfig("]
    assert len(gone) <= 3
