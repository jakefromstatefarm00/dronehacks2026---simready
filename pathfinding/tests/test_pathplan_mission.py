"""Online-planning loop on small synthetic maps (no course files, no simulator)."""
from math import inf

import numpy as np
import pytest

from pathplan.belief import BeliefMap
from pathplan.mission import MissionConfig, run_mission
from pathplan.sensor import DepthCamera
from pathplan.world import Checkpoint, Envelope, GridSpec, cost_grid

SPEC = GridSpec(half_size=1.5, resolution=.05)       # 60 x 60 cells, 3 m square
ENV = Envelope(radius=.15, tracking_margin=.05, soft_margin=.1)


def box(occ, x0, x1, y0, y1):
    """Mark the cells whose centres lie in [x0, x1] x [y0, y1] (metres) occupied."""
    xy = SPEC.to_xy(np.argwhere(np.ones_like(occ)))
    m = (xy[:, 0] >= x0) & (xy[:, 0] <= x1) & (xy[:, 1] >= y0) & (xy[:, 1] <= y1)
    rc = np.argwhere(np.ones_like(occ))[m]
    occ[rc[:, 0], rc[:, 1]] = True
    return occ


def arena():
    occ = np.zeros((SPEC.n, SPEC.n), bool)
    occ[0, :] = occ[-1, :] = occ[:, 0] = occ[:, -1] = True
    return occ


GOAL = [Checkpoint("north line", np.array([0.0, 1.0]), np.array([0.0, 1.0]), 1.2, 7)]
START = np.array([0.0, -1.1])


def test_unmapped_block_static_crashes_replanners_detour():
    prior = arena()
    truth = box(arena(), -.35, .35, -.1, .1)        # an obstacle the map does not know about
    out = {}
    for planner in ("static", "theta", "astar", "dstar"):
        out[planner] = run_mission(prior, truth, SPEC, ENV, GOAL, START, np.pi / 2,
                                   MissionConfig(planner=planner), np.random.default_rng(0))
    assert out["static"].outcome == "crashed"
    for planner in ("theta", "astar", "dstar"):
        r = out[planner]
        assert r.outcome == "finished", (planner, r.events)
        assert r.min_true_clearance >= ENV.radius
        assert r.replans > 0


@pytest.mark.parametrize("planner", ["theta", "dstar"])
def test_known_map_needs_no_replans(planner):
    prior = box(arena(), -.35, .35, -.1, .1)
    r = run_mission(prior, prior.copy(), SPEC, ENV, GOAL, START, np.pi / 2,
                    MissionConfig(planner=planner, replan_every_m=inf), np.random.default_rng(0))
    assert r.outcome == "finished" and r.replans == 0 and r.keepout_violations == 0
    assert r.plan_calls == 1                       # nothing changed, so it planned exactly once


def test_camera_stops_at_first_hit_and_skips_min_range():
    truth = box(arena(), -1.5, 1.5, .5, .6)          # wall across, 0.5 m north of the origin
    cam = DepthCamera(fov_deg=10, rays=5, min_range=.2, max_range=2.0)
    free, hit, behind = cam.observe(truth, SPEC, np.zeros(2), np.pi / 2)
    fy = SPEC.to_xy(free)[:, 1]
    assert fy.min() >= .2 - SPEC.resolution and fy.max() < .5
    assert np.all(truth[hit[:, 0], hit[:, 1]])
    assert len(behind) and SPEC.to_xy(behind)[:, 1].min() > .5   # suspected beyond the face
    # an expected (already mapped) wall produces no suspicion
    _, _, behind2 = cam.observe(truth, SPEC, np.zeros(2), np.pi / 2, expected=truth)
    assert len(behind2) == 0


def test_belief_map_reports_only_changed_costs():
    prior = arena()
    bm = BeliefMap(prior, SPEC, ENV)
    assert bm.integrate(np.empty((0, 2), int), np.empty((0, 2), int), np.empty((0, 2), int)) is None
    hit = SPEC.to_cell(np.array([[0.0, 0.0]]))
    changed = bm.integrate(np.empty((0, 2), int), hit, np.empty((0, 2), int))
    assert changed is not None and bm.occ[tuple(hit[0])]
    assert np.isinf(bm.cost[tuple(hit[0])])
    # seeing the same thing again changes nothing
    assert bm.integrate(np.empty((0, 2), int), hit, np.empty((0, 2), int)) is None
    # free observations clear it after clear_after views (here 3); a hit is believed at once
    bm = BeliefMap(prior, SPEC, ENV, clear_after=3)
    bm.integrate(np.empty((0, 2), int), hit, np.empty((0, 2), int))
    none = np.empty((0, 2), int)
    assert bm.integrate(hit, none, none) is None and bm.occ[tuple(hit[0])]
    assert bm.integrate(hit, none, none) is None and bm.occ[tuple(hit[0])]
    assert bm.integrate(hit, none, none) is not None
    assert not bm.occ[tuple(hit[0])] and np.isfinite(bm.cost[tuple(hit[0])])


def test_one_hit_resets_free_votes():
    bm = BeliefMap(arena(), SPEC, ENV, clear_after=3)
    cell = SPEC.to_cell(np.array([[0.0, 0.0]]))
    none = np.empty((0, 2), int)
    bm.integrate(none, cell, none)                       # seen occupied
    bm.integrate(cell, none, none)
    bm.integrate(cell, none, none)                       # two free views...
    bm.integrate(none, cell, none)                       # ...then seen occupied again
    bm.integrate(cell, none, none)
    bm.integrate(cell, none, none)
    assert bm.occ[tuple(cell[0])]                        # votes restarted: still occupied


def test_suspect_cells_raise_cost_but_do_not_block():
    bm = BeliefMap(arena(), SPEC, ENV, suspect_penalty=5.0)
    base = bm.cost.copy()
    behind = SPEC.to_cell(np.array([[0.0, .5]]))
    bm.integrate(np.empty((0, 2), int), np.empty((0, 2), int), behind)
    cell = tuple(behind[0])
    assert np.isfinite(bm.cost[cell]) and bm.cost[cell] == base[cell] + 5.0


@pytest.mark.parametrize("planner", ["theta", "astar"])
def test_mapped_wall_that_is_gone_is_cleared_by_looking(planner):
    """The published map has a wall across the arena (no path); in truth it is gone.
    The drone must look, clear it, and finish rather than give up."""
    prior = box(arena(), -1.5, 1.5, .2, .3)
    r = run_mission(prior, arena(), SPEC, ENV, GOAL, START, -np.pi / 2,     # facing away from it
                    MissionConfig(planner=planner), np.random.default_rng(0))
    assert r.outcome == "finished", r.events
    assert any(e["event"] == "look around" for e in r.events)


def test_start_inside_goal_cell_does_not_stall():
    goal = [Checkpoint("pad", np.array([0.0, 0.0]), np.array([0.0, 1.0]), .2, 7, kind="pad")]
    r = run_mission(arena(), arena(), SPEC, ENV, goal, np.array([.01, .01]), 0.0,
                    MissionConfig(planner="astar", max_steps=50), np.random.default_rng(0))
    assert r.outcome == "finished"


def _side_of_pillar(poly):
    """-1 if the route passes left of x = 0 at the pillar's height, +1 if right."""
    mid = [p for p in poly if -.3 < p[1] < .3]
    return int(np.sign(np.mean([p[0] for p in mid]))) if mid else 0


@pytest.mark.parametrize("margin, expect_flips", [(0.0, True), (.10, False)])
def test_path_commitment_keeps_the_route_through_near_ties(margin, expect_flips):
    """Pillar dead centre: left and right nearly tie, and a few cm of pose wobble tips the
    choice each replan. Without commitment the route flips sides; with it, it stays."""
    from pathplan.online import OnlinePlanner, PlannerConfig
    occ = box(arena(), -.15, .15, -.15, .15)
    planner = OnlinePlanner(occ, SPEC, ENV, GOAL, PlannerConfig(commit_margin=margin, lookahead_m=0))
    sides = []
    for i in range(12):
        wobble = .06 if i % 2 else -.06
        cmd = planner.update(np.array([wobble, -1.1]), np.pi / 2)
        assert cmd.mode == "move"
        sides.append(_side_of_pillar(cmd.path))
        planner.dirty = True                                  # force a replan every step
    assert (len(set(sides)) > 1) == expect_flips, sides


def test_path_commitment_switches_when_the_route_is_blocked():
    from pathplan.online import OnlinePlanner, PlannerConfig
    occ = box(arena(), -.15, .15, -.15, .15)
    planner = OnlinePlanner(occ, SPEC, ENV, GOAL, PlannerConfig(lookahead_m=0))
    pos = np.array([0.0, -1.1])
    side = _side_of_pillar(planner.update(pos, np.pi / 2).path)
    # a wall appears on that side, from the pillar to the arena edge
    wall = box(np.zeros_like(occ), .15, 1.5, -.05, .05) if side > 0 else box(np.zeros_like(occ), -1.5, -.15, -.05, .05)
    planner.observe(np.empty((0, 2), int), np.argwhere(wall), np.empty((0, 2), int))
    assert _side_of_pillar(planner.update(pos, np.pi / 2).path) == -side


@pytest.mark.parametrize("bad", ["fly", ""])
def test_unknown_planner_rejected(bad):
    with pytest.raises(ValueError):
        run_mission(arena(), arena(), SPEC, ENV, GOAL, START, 0.0, MissionConfig(planner=bad))


def test_goal_pocket_is_entered_only_across_the_line():
    """Front of the line blocked (phantom walls in the first corridor), a way round behind it
    open: the planner must not slip into the goal pocket from behind ("goal without crossing")."""
    from pathplan.online import OnlinePlanner, PlannerConfig
    occ = arena()
    occ = box(occ, -1.5, -.6, -.05, .05)            # partition with an opening at x > -0.6 ...
    occ = box(occ, -.6, .9, .45, .55)               # ... and a wall across the front of the line
    line = [Checkpoint("line", np.array([-1.0, .1]), np.array([0.0, 1.0]), .35, 1)]
    planner = OnlinePlanner(occ, SPEC, ENV, line, PlannerConfig(lookahead_m=0))
    cmd = planner.update(np.array([-1.0, -.8]), np.pi / 2)
    # the only route that reaches the pocket goes round the partition's end and in from behind
    assert cmd.mode == "turn" and cmd.reason == "look around", cmd


def test_look_around_turns_toward_the_goal_side_first():
    from pathplan.online import OnlinePlanner, PlannerConfig
    occ = box(arena(), -1.5, 1.5, .2, .3)                      # wall across: no path north
    for start_yaw, expect in ((np.pi / 2 + .5, -1), (np.pi / 2 - .5, +1)):
        p = OnlinePlanner(occ, SPEC, ENV, GOAL, PlannerConfig(lookahead_m=0))
        cmd = p.update(START, start_yaw)
        assert cmd.mode == "turn" and np.sign(_wrap_test(cmd.yaw - start_yaw)) == expect, (start_yaw, cmd)


def _wrap_test(a):
    return float(np.arctan2(np.sin(a), np.cos(a)))
