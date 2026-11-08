"""Speed-gated turning ("turns"): the deterministic yaw-rate x speed clamp, default-off equivalence,
the corner view rule port, and the generated turns files."""
import importlib.util
import math
from pathlib import Path
import re

import numpy as np
import pytest

from flyx.mission import Mission, MissionConfig, Telemetry, gated_yaw_cap
from flyx.px4_link import ARMING_STATE_ARMED, NAV_STATE_OFFBOARD, yaw_from_q_ned
from pathplan.online import Command as PlanCommand

ROOT = Path(__file__).resolve().parents[1]
REPO = ROOT.parent
TURNS = dict(turn_gate=True, yaw_speed_max=.10, yaw_rate_slow_max=.50, yaw_rate_in_place_max=.35)


# ------------------------------------------------------------------ the clamp itself
def test_off_is_the_v3_cap():
    c = MissionConfig()
    assert not c.turn_gate
    for v in (None, 0.0, .1, .5, 2.0):
        assert gated_yaw_cap(v, False, c) == c.yaw_rate_max == .25
        assert gated_yaw_cap(v, True, c) == .25


@pytest.mark.parametrize("in_place", [False, True])
def test_cap_times_speed_never_exceeds_the_bound(in_place):
    c = MissionConfig(**TURNS)
    rng = np.random.default_rng(0)
    speeds = np.r_[0.0, 1e-9, .01, .05, .1, .15, .2, .25, .4, .5, .6, 1.0, 3.0, rng.uniform(0, 2, 2000)]
    for v in speeds:
        cap = gated_yaw_cap(float(v), in_place, c)
        assert cap * v <= .10 + 1e-12
        assert 0.0 < cap <= (.35 if in_place else .50) + 1e-12
    assert gated_yaw_cap(None, False, c) == pytest.approx(.10 / .60)          # unknown speed: assume 0.6 m/s
    assert gated_yaw_cap(float("nan"), False, c) == pytest.approx(.10 / .60)


def test_measured_speed_only_lowers_the_ceiling():
    c = MissionConfig(**TURNS)
    for v in np.linspace(0, 1, 41):
        for m in np.linspace(0, 1, 41):
            assert gated_yaw_cap(v, False, c, m) <= gated_yaw_cap(v, False, c) + 1e-12
            assert gated_yaw_cap(v, False, c, m) * v <= .10 + 1e-12
    assert gated_yaw_cap(.14, False, c, .02) == pytest.approx(.35)     # predicted 0.14 m/s, really stopped


def test_cap_shape_higher_when_slow_never_higher_than_v3_when_fast():
    c, base = MissionConfig(**TURNS), .25
    assert gated_yaw_cap(0.0, False, c) == pytest.approx(.35)                 # nearly stopped: moderate
    assert gated_yaw_cap(.15, False, c) == pytest.approx(.50)                 # slow: sharper
    assert gated_yaw_cap(.20, False, c) == pytest.approx(.50)
    for v in (.4, .45, .5, .6):                                               # fast: at most v3, often less
        assert gated_yaw_cap(v, False, c) <= base + 1e-12
    caps = [gated_yaw_cap(v, False, c) for v in np.linspace(.15, 2, 50)]
    assert all(a >= b - 1e-12 for a, b in zip(caps, caps[1:]))                # non-increasing above 0.15 m/s


# ------------------------------------------------------------------ closed loop in the Mission
class TurnPlanner:
    """Route due east (course frame); the vehicle starts heading north, so the nose must turn 90 deg."""
    events = []

    def update(self, xy, yaw):
        return PlanCommand("move", np.array([xy, [xy[0] + 2.0, xy[1]]]), 0.0)

    def observe(self, *cells):
        pass


def _tel(t, xy_ned, yaw_ned, sim=None):
    q = (math.cos(yaw_ned / 2), 0.0, 0.0, math.sin(yaw_ned / 2))
    return Telemetry(now=t, sim_now=sim, odom_t=t, position_ned=(xy_ned[0], xy_ned[1], -.85), q_ned_frd=q,
                     status_t=t, arming_state=ARMING_STATE_ARMED, nav_state=NAV_STATE_OFFBOARD, landed_t=t,
                     landed=False, local_t=t, xy_valid=True, z_valid=True)


def _navigating(cfg):
    m = Mission(TurnPlanner(), MissionConfig(execute=True, **cfg))
    from flyx.px4_link import CourseFrame
    m.frame = CourseFrame.from_launch(position_ned=[0.0, 0.0, 0.0], heading_ned=0.0, launch_xy=[-2.515, .914],
                                      body_agl=.05, physical_heading_in_course=math.pi / 2, placement_reviewed=True)
    m.state, m._t_state, m._t_takeoff, m._sim_takeoff = "NAVIGATE", 0.0, 0.0, 0.0
    return m


def _fly(cfg, speed, seconds=4.0, rtf=1.0, follow=True):
    """Vehicle translating north (NED x) at ``speed`` (sim m/s) while the mission turns the nose; the
    heading follows the setpoint (follow=True) or lags it (follow=False). Returns per-tick
    (commanded rate in sim rad/s, true speed)."""
    m = _navigating(cfg)
    dt = .05
    x, yaw, out = 0.0, 0.0, []
    prev_sp = None
    for k in range(int(seconds / dt)):
        t = k * dt
        m.tick(_tel(t, (x, 0.0), yaw, sim=t * rtf))
        if k % 4 == 0:
            m.plan()
        o = m.tick(_tel(t + 1e-3, (x, 0.0), yaw, sim=(t + 1e-3) * rtf))
        sp_yaw = o.setpoint[1]
        if prev_sp is not None:
            rate = abs(math.atan2(math.sin(sp_yaw - prev_sp), math.cos(sp_yaw - prev_sp))) / (dt * rtf)
            out.append((rate, speed, m.yaw_rate_cmd, m.speed_for_cap))
        prev_sp = sp_yaw
        yaw = sp_yaw if follow else yaw + .3 * math.atan2(math.sin(sp_yaw - yaw), math.cos(sp_yaw - yaw))
        x += speed * dt * rtf
    return m, out


@pytest.mark.parametrize("speed", [0.0, .05, .1, .2, .3, .4, .5, .7])
@pytest.mark.parametrize("rtf", [1.0, .8])
def test_closed_loop_commanded_rate_times_speed_within_bound(speed, rtf):
    m, out = _fly(TURNS, speed, rtf=rtf)
    for rate, v, cmd_rate, v_cap in out[10:]:                # after the speed estimate has history
        assert rate * v <= .10 + 2e-3, (rate, v)
        assert cmd_rate * v_cap <= .10 + 1e-9
        assert rate <= .50 + 2e-3


def test_closed_loop_turns_faster_when_slow_not_when_fast():
    _, slow_on = _fly(TURNS, .03)
    _, slow_off = _fly({}, .03)
    _, fast_on = _fly(TURNS, .45)
    _, fast_off = _fly({}, .45)
    peak = lambda o: max(r for r, *_ in o[10:])
    assert peak(slow_on) > 1.3 * peak(slow_off)              # slow: sharper than v3's 0.25
    assert peak(fast_on) <= peak(fast_off) + 1e-9            # fast: never sharper than v3


def test_lead_over_the_vehicle_heading_is_bounded():
    """The setpoint never runs more than cap x yaw_lead_s (+ one tick) ahead of the measured heading,
    so PX4 cannot keep turning fast on a backlog after the cap drops."""
    m2 = _navigating(TURNS)
    yaw = 0.0
    for k in range(80):                                      # vehicle heading frozen at north
        t = k * .05
        m2.tick(_tel(t, (0.0, 0.0), yaw))
        if k % 4 == 0:
            m2.plan()
        m2.tick(_tel(t + 1e-3, (0.0, 0.0), yaw))
    lead = abs(math.atan2(math.sin(m2._sp_yaw - yaw), math.cos(m2._sp_yaw - yaw)))
    assert lead <= m2.yaw_cap * m2.cfg.yaw_lead_s + m2.yaw_cap * .05 + 1e-6


def test_default_config_flies_exactly_like_v3():
    """turn_gate off (the default) must give the v3 setpoints tick for tick."""
    _, a = _fly({}, .2, follow=True)
    _, b = _fly(dict(turn_gate=False, yaw_rate_slow_max=.9, explore_look_deg=0.0, corner_carrot_scale=1.0), .2)
    assert a == b
    assert max(r for r, *_ in a) == pytest.approx(.25, abs=1e-6)


# ------------------------------------------------------------------ corner rule port + generated files
def test_corner_rule_matches_first_corner_vio():
    from pathplan import planner_process as pp
    for name in ("route_turn_ahead", "view_distance", "view_heading", "corner_rule"):
        assert callable(getattr(pp, name))
    assert (pp.CORNER_TURN_DEG, pp.CORNER_WITHIN_M, pp.CORNER_TURNING_DEG, pp.VIEW_ENOUGH_M, pp.VIEW_MAX_DEV_DEG,
            pp.VIEW_STEP_DEG) == (45.0, 1.2, 20.0, 1.5, 40, 10)


def test_short_view_caps_the_rate_at_v3():
    m = _navigating(TURNS)
    m.view_m = .6
    assert m.short_view_cap() == pytest.approx(.25)
    m.view_m = 1.5
    assert m.short_view_cap() == float("inf")
    m.wall_m = .4                                               # a wall beside the drone, view ahead long
    assert m.short_view_cap() == pytest.approx(.25)
    m.wall_m = .9
    assert m.short_view_cap() == float("inf")
    off = _navigating({})
    off.view_m = .2
    assert off.short_view_cap() == float("inf")                 # default off: v3


def test_long_view_sign_picks_the_open_side():
    from pathplan.planner_process import long_view_sign
    from pathplan.world import GridSpec
    spec = GridSpec()
    occ = np.zeros((spec.n, spec.n), bool)
    rc = spec.to_cell(np.array([0.0, .4]))          # a wall 0.4 m to the left (+y) of a drone facing +x
    occ[rc[0] - 1:rc[0] + 2, :] = True
    assert long_view_sign(occ, spec, np.array([0.0, 0.0]), 0.0) == -1.0
    assert long_view_sign(np.zeros_like(occ), spec, np.array([0.0, 0.0]), 0.0) is None


def test_planner_look_around_extent_default_is_a_full_turn():
    from pathplan.online import OnlinePlanner
    import inspect
    assert "self.look_max_rad = 2 * np.pi" in inspect.getsource(OnlinePlanner.__init__)


class _Conn:
    def __init__(self, inbox, outbox):
        self.inbox, self.outbox = inbox, outbox

    def send(self, x):
        self.outbox.put(x)

    def recv(self):
        return self.inbox.get(timeout=60)


# best-v5: this build's planner options (the crawl breaker / short_view_guard of turns-explore are not in it)
@pytest.mark.parametrize("options", [None, {"view_m": True}, {"corner_view": True, "view_m": True},
                                     {"look_guard": True, "forget_snap": True, "forget_votes": True, "view_m": True},
                                     {"solid_relax": True, "plan_cache": False}, {"solid_off": True, "soft_margin": .15},
                                     {"plan_cache": True, "look_guard": True, "corner_view": True, "forget_votes": True,
                                      "forget_snap": True, "solid_relax": True, "view_m": True}])
def test_planner_child_loop_runs_with_every_turns_option(options, monkeypatch):
    """The real planner_process loop on a thread with each option set (7ea3480 crashed the planner
    process on its first command: _wall_distance(.8) passed the window as a position)."""
    import queue
    import threading
    from pathplan import planner_process as pp
    from pathplan.ros_node import build_planner
    monkeypatch.setattr(pp, "signal", type("S", (), {"signal": staticmethod(lambda *a: None), "SIGINT": 2, "SIG_IGN": None}))
    monkeypatch.setattr(pp.os, "nice", lambda n: 0, raising=False)
    a, b = queue.Queue(), queue.Queue()
    th = threading.Thread(target=pp.planner_process, args=(_Conn(a, b), build_planner), daemon=True)
    th.start()
    a.put(("theta", .9, (.115, 0.0, .029), 0.0, .5) + ((options,) if options else ()))
    assert b.get(timeout=60)[0] == "ready"
    xy = np.array([-2.515, .914])
    for k in range(30):
        a.put((xy + [0.0, .02 * k], np.pi / 2, None))
        cmd, *_ = b.get(timeout=60)
        assert cmd.mode in ("move", "turn")
        if options and options.get("view_m"):
            assert 0.0 <= cmd.view_m <= 2.0 and 0.0 <= cmd.wall_m <= .8
    a.put(None)
    th.join(5)
    assert not th.is_alive()
