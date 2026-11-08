"""best-v5 switches on top of v4c: each is off in OnlinePlanner (= v4c) and set by planner_process."""
import numpy as np

from pathplan.world import Checkpoint, Envelope, GridSpec

SPEC = GridSpec(half_size=1.5, resolution=.05)
ENV = Envelope(radius=.15, tracking_margin=.05, soft_margin=.1)
GOAL = [Checkpoint("north line", np.array([0.0, 1.0]), np.array([0.0, 1.0]), 1.2, 7)]
START = np.array([0.0, -1.1])
NONE = np.empty((0, 2), int)


def arena():
    occ = np.zeros((SPEC.n, SPEC.n), bool)
    occ[0, :] = occ[-1, :] = occ[:, 0] = occ[:, -1] = True
    return occ


def wall_cells(y0=.2, y1=.3):
    xy = SPEC.to_xy(np.argwhere(np.ones((SPEC.n, SPEC.n), bool)))
    m = (xy[:, 1] >= y0) & (xy[:, 1] <= y1)
    return np.argwhere(np.ones((SPEC.n, SPEC.n), bool))[m]


def _stuck_planner(cache):
    from pathplan.online import OnlinePlanner, PlannerConfig
    p = OnlinePlanner(arena(), SPEC, ENV, GOAL, PlannerConfig(lookahead_m=0))
    p.plan_cache = cache
    p.map_resets = 2                                  # no forgets left: straight to looking around
    p.observe(NONE, wall_cells(), NONE)               # a depth wall sealing the arena
    return p


def test_plan_cache_skips_repeated_failed_search_with_identical_commands():
    out = {}
    for cache in (False, True):
        p, yaw, cmds = _stuck_planner(cache), np.pi / 2, []
        for _ in range(12):
            cmd = p.update(START, yaw)
            cmds.append((cmd.mode, cmd.reason, None if cmd.yaw is None else round(cmd.yaw, 9)))
            if cmd.mode == "turn":
                yaw = cmd.yaw                         # turned there; no new depth (map unchanged)
        out[cache] = (cmds, p.stats.get("cached_failures", 0), p.stats["plan_calls"])
    assert out[True][0] == out[False][0]              # same behaviour
    assert out[True][1] > 0 and out[True][2] < out[False][2]   # fewer searches
    assert out[False][1] == 0


def test_plan_cache_searches_again_when_the_map_changes():
    p = _stuck_planner(True)
    p.update(START, np.pi / 2)
    calls = p.stats["plan_calls"]
    p.observe(wall_cells(), NONE, NONE)               # the camera now sees the phantom wall free
    cmd = p.update(START, np.pi / 2)
    assert p.stats["plan_calls"] == calls + 1 and cmd.mode == "move", (cmd, p.events[-2:])


def test_look_guard_bounds_the_look_around():
    from pathplan.online import _wrap
    p = _stuck_planner(True)
    p.look_max_rad = np.pi
    yaw, turned = np.pi / 2, 0.0
    for _ in range(300):
        cmd = p.update(START, yaw)
        if cmd.mode != "turn":
            break
        turned += abs(_wrap(cmd.yaw - yaw))
        yaw = cmd.yaw
        p.observe(NONE, NONE, NONE)                   # a frame at this heading
    assert turned <= np.pi + np.deg2rad(16), turned
    assert any(e["event"] == "doubt unconfirmed map" for e in p.events)


def test_look_guard_turns_toward_the_long_view():
    from pathplan.planner_process import long_view_sign
    occ = arena()
    occ[:, SPEC.n // 2 + 6:] = True                   # a wall 0.3 m to the right (+x) of the centre line
    # facing north (+y): the left (-x) side sees farther
    assert long_view_sign(occ, SPEC, np.array([0.0, 0.0]), np.pi / 2) == 1.0


def test_corner_view_leads_into_the_turn_at_most_40_deg():
    from types import SimpleNamespace
    from pathplan.online import Command, _wrap
    from pathplan.planner_process import VIEW_MAX_DEV_DEG, corner_rule
    occ = arena()
    occ[:SPEC.n // 2 + 2, SPEC.n // 2 + 8:] = True    # corner: north, then right; a near wall straight ahead
    pl = SimpleNamespace(belief=SimpleNamespace(occ=occ), spec=SPEC)
    path = np.array([[0.0, -.6], [0.0, .3], [1.2, .3]])
    cmd = Command("move", path, np.pi / 2)
    corner_rule(pl, cmd, path[0], np.pi / 2)
    dev = _wrap(cmd.yaw - np.pi / 2)
    assert -np.deg2rad(VIEW_MAX_DEV_DEG) - 1e-9 <= dev <= 0.0     # into the (right) turn, bounded
    near = Command("move", path, np.pi / 2, "near miss")
    corner_rule(pl, near, path[0], np.pi / 2)
    assert near.yaw == np.pi / 2                       # never on a near miss / back out


def test_no_mapping_or_planning_during_takeoff_by_default():
    """[v5] Gazebo v909: depth taken while climbing made phantoms in checkpoint 1's goal pocket ->
    forget + look around at the launch mark. v5 maps and plans only from NAVIGATE; v4c also in TAKEOFF."""
    from flyx.mission import Mission, MissionConfig
    for cfg, expect in ((MissionConfig(), False), (MissionConfig(plan_in_takeoff=True), True)):
        m = Mission(planner=None, cfg=cfg)
        m.state, m.pose_course = "TAKEOFF", (np.array([-2.5, .9, .7]), np.pi / 2, np.eye(3))
        assert bool(m.planning_allowed()) is expect
        m.state = "NAVIGATE"
        assert m.planning_allowed()


def test_planner_process_configure_passes_options_like_fly_pathplan():
    """Gazebo v926: fly_pathplan passes a 6th element (planner options) and configure() refused it
    (TypeError at start-up). The message must be the 5-tuple without options, the 6-tuple with."""
    from pathplan.planner_process import PlannerProcess

    class Conn:
        def __init__(self):
            self.sent = []

        def send(self, m):
            self.sent.append(m)

        def recv(self):
            return "ready", [0.0, 0.0]
    for options, n in ((None, 5), ({"view_m": True}, 6)):
        p = object.__new__(PlannerProcess)
        p.conn = Conn()
        p.configure("theta", .9, (.115, 0.0, .029), 0.0, .5, options)
        assert len(p.conn.sent[0]) == n and p.pad_xy == [0.0, 0.0]
    # and the exact call fly_pathplan.main() makes
    import inspect
    import flyx.fly_pathplan as fp
    src = inspect.getsource(fp.main)
    assert "a.carrot, planner_options(a)))" in src


def test_v5_options_default_on_and_known():
    from pathplan.planner_process import V5_OPTIONS
    assert set(V5_OPTIONS) == {"plan_cache", "look_guard", "corner_view", "forget_votes", "forget_snap",
                               "solid_relax", "solid_off", "soft_margin", "view_m"}
    assert V5_OPTIONS["plan_cache"] is True
