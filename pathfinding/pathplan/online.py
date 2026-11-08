"""OnlinePlanner: the drone-side brain, independent of ROS and of the simulator.

    planner = OnlinePlanner(prior_occ, spec, env, checkpoints)
    loop:
        planner.observe(free, hit, behind)     # after each depth frame (map update)
        cmd = planner.update(xy, yaw)          # after each pose update
        cmd.mode == "move"  -> follow cmd.path (course-frame xy polyline), face cmd.yaw
        cmd.mode == "turn"  -> yaw in place toward cmd.yaw, do not translate
        cmd.mode == "done" / "failed"

The same object runs in the offline benchmark (``mission.run_mission``) and in
the ROS 2 node (``ros_node.py``), so what is benchmarked is what flies.

Behaviour (each item fixed a failure seen in the benchmark):
* checkpoints in order; the chicane accepts the window (+5) or another opening (Gate B);
* replan when a map change lands near the remaining path, and every ``replan_every_m``;
* look before you go: face the point ``lookahead_m`` ahead, turn in place if far off;
* inside a newly seen keep-out: back out to the nearest planable cell;
* no path: look around once, then doubt unconfirmed published walls (checkpoint
  structure is always trusted).
"""
from __future__ import annotations

from dataclasses import dataclass, field
from math import atan2, inf
import time

import numpy as np

from .belief import BeliefMap
from .grid_search import astar, theta_star

MAX_MAP_RESETS = 2      # times per flight the planner may forget depth obstacles before giving up
LOOK_CLEAR_M = .50      # look around (turn in place) only this far from anything; else back out first
NEAR_MISS_M = .03       # inside the keep-out, this much beyond the drone's radius still counts as a near miss


@dataclass
class PlannerConfig:
    planner: str = "theta"          # theta | astar | static (no sensing) | dstar (experimental)
    goal_beyond: float = .10        # goal sits this far past (radius beyond) each line
    suspect_penalty: float = 8.0    # see BeliefMap; 0 disables
    lookahead_m: float = .7         # 0 = no yaw command (caller faces the motion)
    turn_in_place_deg: float = 30.
    turn_step_deg: float = 15.
    replan_every_m: float = .5
    commit_margin: float = .10      # keep the current route unless the new one is >10 % cheaper; 0 = off

    PLANNERS = ("theta", "astar", "static", "dstar")

    def validate(self):
        if self.planner not in self.PLANNERS:
            raise ValueError(f"planner must be one of {self.PLANNERS}, got {self.planner!r}")
        if self.turn_step_deg <= 0:
            raise ValueError("turn_step_deg must be positive")
        return self


@dataclass
class Command:
    mode: str                       # move | turn | done | failed
    path: np.ndarray | None = None  # move: (K, 2) course xy from the current position
    yaw: float | None = None        # desired heading, course frame (None = face the motion)
    reason: str = ""


class Router:
    """One interface over the search algorithms: ``plan(cost, here, goal) -> vertex cells``."""

    def __init__(self, kind: str):
        self.kind = kind
        self.dstar = None

    def reset(self):
        """Forget incremental state (new goal, or after an escape)."""
        self.dstar = None

    def map_changed(self, cells, cost):
        if self.dstar is not None:
            self.dstar.update_costs(cells, cost[cells[:, 0], cells[:, 1]])

    def plan(self, cost, here, goal):
        if self.kind in ("theta", "static"):
            cells, _, ex = theta_star(cost, here, goal)
            return cells, ex
        if self.kind == "astar":
            cells, _, ex = astar(cost, here, goal)
            return cells, ex
        from .dstar_lite import DStarLite        # experimental, imported only when used
        if self.dstar is None or self.dstar.goal != goal:
            self.dstar = DStarLite(cost, here, goal)
        else:
            self.dstar.move_start(here)
        before = self.dstar.expansions
        self.dstar.compute()
        return self.dstar.path(), self.dstar.expansions - before


class OnlinePlanner:
    def __init__(self, prior_occ, spec, env, checkpoints, cfg: PlannerConfig | None = None):
        self.cfg = (cfg or PlannerConfig()).validate()
        self.spec, self.env = spec, env
        self.checkpoints = list(checkpoints)
        self.prior = prior_occ
        self.belief = BeliefMap(prior_occ, spec, env, self.cfg.suspect_penalty)
        self.prior_cost = self.belief.cost.copy()           # the published map's cost (look-around direction)
        self.router = Router(self.cfg.planner)
        self.sensing = self.cfg.planner != "static"
        self.cleared, self.events, self.plan_times = [], [], []
        self.stats = dict(expansions=0, replans=0, plan_calls=0, escapes=0)
        self.pos = self.yaw = None
        self._near_miss = False
        self.map_resets = 0
        self._trail, self._backing = [], 0
        self.observations = 0                      # depth frames integrated
        self.status = "active"                     # active | done | failed
        self.index = 0
        self._near = env.hard + env.soft_margin
        self._grid_xy = spec.to_xy(np.argwhere(np.ones(prior_occ.shape, bool))).reshape(prior_occ.shape + (2,))
        self._start_checkpoint()

    # ------------------------------------------------------------ inputs
    def observe(self, free, hit, behind):
        """Integrate one depth observation (cells from DepthCamera or depth_projection)."""
        if not self.sensing:
            return
        self.observations += 1
        changed = self.belief.integrate(free, hit, behind)
        if changed is None:
            return
        self.router.map_changed(changed, self.belief.cost)      # D* must see every change
        if not self.dirty and _near_path(self.spec.to_xy(changed), self.poly, self._near):
            self.dirty = True
            self.stats["replans"] += 1

    def update(self, xy, yaw) -> Command:
        """New pose (course frame). Returns what to do next."""
        xy = np.asarray(xy, float)
        prev, self.pos, self.yaw = self.pos, xy, float(yaw)
        if not self._trail or np.linalg.norm(xy - self._trail[-1]) >= .15:
            self._trail = (self._trail + [xy.copy()])[-80:]             # where we have been (backing out)
        if prev is not None and self.status == "active":
            self.since_plan += float(np.linalg.norm(xy - prev))
            self.poly = _trim(self.poly, xy)
            self._check_progress(prev, xy)
        if self.status == "done":
            return Command("done")
        if self.status == "failed":
            return Command("failed", reason=self.events[-1]["event"] if self.events else "")
        return self._decide()

    @property
    def checkpoint(self):
        return self.checkpoints[self.index] if self.index < len(self.checkpoints) else None

    # --------------------------------------------------------- internals
    def _start_checkpoint(self):
        self.router.reset()
        self.poly, self.dirty, self.looked, self.optimistic, self.since_plan = None, True, 0.0, False, 0.0
        self._look_prev, self._look_calls, self._look_target, self._look_mark = None, 0, None, None
        self._look_dir = None
        cp = self.checkpoint
        if cp is None:
            self.status = "done"
            return
        self.trusted = self._near_line(cp, .5)               # checkpoint structure is never doubted
        self.fence = self._fence(cp)
        self.goal = self._snap_goal(self.belief.cost, cp)        # None: _decide looks for one

    def _fail(self, why):
        self.status = "failed"
        self.events.append({"checkpoint": getattr(self.checkpoint, "name", ""), "event": why,
                            "at": None if self.pos is None else self.pos.round(3).tolist()})

    def _event(self, what):
        self.events.append({"checkpoint": self.checkpoint.name, "event": what, "at": self.pos.round(3).tolist()})

    def _check_progress(self, a, b):
        cp = self.checkpoint
        via = None
        if cp.reached(a, b, self.env.radius):
            via = "line"
        elif self.goal is not None and np.linalg.norm(b - self.spec.to_xy(np.array(self.goal))) < .025:
            # at the goal (past the line) without crossing it: it came in by another opening
            if not cp.other_route:
                self._fail("goal without crossing")
                return
            via = "other"
            self._event("entered by another opening")
        if via:
            self.cleared.append({"name": cp.name, "points": cp.points, "via": via})
            self.index += 1
            self._start_checkpoint()

    def _decide(self) -> Command:
        cfg, spec, cost = self.cfg, self.spec, self.belief.cost
        relaxed = self.belief.optimistic_cost(self.trusted) if self.optimistic else None
        here = tuple(int(v) for v in spec.to_cell(self.pos))
        if not np.isfinite((relaxed if self.optimistic else cost)[here]):
            free = np.argwhere(np.isfinite(cost))
            if not len(free):
                self._fail("no free cell")
                return Command("failed", reason="no free cell")
            nearest = free[np.argmin(np.linalg.norm(spec.to_xy(free) - self.pos, axis=1))]
            if self._wall_distance() >= self.env.radius + NEAR_MISS_M:
                # Inside the keep-out (it includes 8 cm for tracking error) but not touching: keep
                # flying the route, planning from the nearest free cell. Escaping here jittered the
                # drone back and forth 0.34 m from the wall past maze wall M1, turning in place, and
                # visual odometry died (FlyX runs 6 and 9).
                if not self._near_miss:
                    self._event("keep-out near miss")
                self._near_miss = True
                here = tuple(int(v) for v in nearest)
                if self.poly is None or len(self.poly) < 2:
                    self.dirty = True
            else:
                # a newly seen obstacle (or a real slip) puts us really close: back out
                self._near_miss = False
                target = spec.to_xy(nearest)
                self.stats["escapes"] += 1
                self._event("escape")
                self.router.reset()
                self.poly, self.dirty = None, True
                return Command("move", np.array([self.pos, target]), self.yaw, "escape")
        else:
            self._near_miss = False

        if self.goal is None or not np.isfinite(cost[self.goal]):
            goal = self._snap_goal(cost, self.checkpoint)
            if goal is None and self.optimistic:
                goal = self._snap_goal(relaxed, self.checkpoint)
            if goal is None:
                return self._stuck("no goal cell")
            self.goal = goal
            self.router.reset()
            self.dirty = True
        if self.sensing and self.since_plan >= cfg.replan_every_m:
            self.dirty = True

        if self.dirty or self.poly is None or len(self.poly) < 2:
            self.since_plan, self.dirty = 0.0, False
            t = time.perf_counter()
            cost, relaxed = self._fenced(cost, here), self._fenced(relaxed, here)
            cells, ex = self.router.plan(cost, here, self.goal)
            if cells:
                self.optimistic = False
            elif self.optimistic:                        # fallback search on the relaxed map
                self.router.reset()
                cells, _, ex = theta_star(relaxed, here, self.goal)
            self.plan_times.append(time.perf_counter() - t)
            self.stats["expansions"] += ex
            self.stats["plan_calls"] += 1
            if not cells:
                return self._stuck("no path")
            self.looked, self._look_prev, self._look_calls, self._look_target = 0.0, None, 0, None
            self._look_dir = None
            self._backing = 0
            # start at the true position; a one-cell plan (already in the goal cell) still
            # flies to the cell centre instead of stalling
            new_poly = np.vstack([self.pos, spec.to_xy(np.array(cells[1:] or cells[-1:]))])
            self.poly = self._commit(cost, new_poly)

        if cfg.lookahead_m <= 0:
            return Command("move", self.poly, None)
        ahead = _point_along(self.poly, cfg.lookahead_m)
        if np.linalg.norm(ahead - self.pos) < 1e-6:
            return Command("move", self.poly, self.yaw)
        want = atan2(ahead[1] - self.pos[1], ahead[0] - self.pos[0])
        off = _wrap(want - self.yaw)
        if abs(off) > np.deg2rad(cfg.turn_in_place_deg):
            turn = np.sign(off) * min(abs(off), np.deg2rad(cfg.turn_step_deg))
            return Command("turn", yaw=self.yaw + turn, reason="look before moving")
        return Command("move", self.poly, want)

    def _stuck(self, why):
        """No route or no free goal cell: 1) forget depth obstacles (phantoms from misplaced or noisy
        depth - by far the commonest cause in FlyX; no movement needed); 2) look around once, sensing
        - but first back out to open space if near walls: turning in place next to a wall lost visual
        odometry in the real sim (M1 channel; tunnel mouth, run 10); 3) doubt unconfirmed published
        walls; 4) give up."""
        if self.map_resets < MAX_MAP_RESETS and self._look_calls == 0 and not self._backing:
            self.map_resets += 1
            self._event("forget depth map")
            self.events[-1]["why"] = why
            self.belief = BeliefMap(self.prior, self.spec, self.env, self.cfg.suspect_penalty)
            self.router.reset()
            self.optimistic, self.dirty = False, True
            self.looked, self._look_prev, self._look_calls, self._look_target, self._look_dir = 0.0, None, 0, None, None
            return self._decide()
        if self._look_calls == 0 and self._backing < 60 and self._wall_distance() < LOOK_CLEAR_M:
            target = next((p for p in reversed(self._trail[:-1])
                           if self._wall_distance(p) >= LOOK_CLEAR_M and np.linalg.norm(p - self.pos) > .1), None)
            if target is not None:
                if not self._backing:
                    self._event("back out to look")
                self._backing += 1
                self.dirty = True
                return Command("move", np.array([self.pos, target]), self.yaw, "back out to look")
        # count the heading change actually seen, not the calls: a real vehicle turns slower
        # than we ask (5 calls/s at 15 deg each would "finish" before PX4 had turned 90 deg)
        if self._look_prev is not None:
            self.looked += abs(_wrap(self.yaw - self._look_prev))
        self._look_prev, self._look_calls = self.yaw, self._look_calls + 1
        if self.looked < 2 * np.pi and self._look_calls <= 200:      # cap: in case it cannot turn
            if self._look_calls == 1:
                self._event("look around")
                self.events[-1]["why"] = why
            self.dirty = True
            # step, hold until a depth frame has been taken at this heading, step again: frames
            # taken mid-turn are the ones a slightly late pose misplaces (the node drops them)
            step = np.deg2rad(self.cfg.turn_step_deg) * self._look_sign()
            if self._look_target is None:
                self._look_target, self._look_mark = self.yaw + step, None
            elif abs(_wrap(self._look_target - self.yaw)) < np.deg2rad(4):
                if self._look_mark is None:
                    self._look_mark = self.observations
                elif self.observations > self._look_mark or not self.sensing:
                    self._look_target, self._look_mark = self._look_target + step, None
            return Command("turn", yaw=self._look_target, reason="look around")
        if not self.optimistic:
            # 3) doubt published walls the camera has not confirmed (a wall that moved)
            self.optimistic, self.dirty = True, True
            self._event("doubt unconfirmed map")
            return self._decide()
        self._fail(why)
        return Command("failed", reason=why)

    def _look_sign(self):
        """Turn toward where the PUBLISHED route goes first. The straight-line bearing to the next
        checkpoint (used before) is often the wrong side in a maze: the real sim kept looking left
        while the course turns right. Falls back to that bearing if the published map has no route."""
        if getattr(self, "_look_dir", None) is None:
            goal = self.spec.to_xy(np.array(self.goal)) if self.goal is not None else self.checkpoint.centre
            towards = goal
            here = tuple(int(v) for v in self.spec.to_cell(self.pos))
            goal_rc = tuple(int(v) for v in self.spec.to_cell(goal))
            if np.isfinite(self.prior_cost[here]) and np.isfinite(self.prior_cost[goal_rc]):
                cells, _, _ = theta_star(self.prior_cost, here, goal_rc)
                if len(cells) >= 2:
                    towards = _point_along(np.vstack([self.pos, self.spec.to_xy(np.array(cells[1:]))]), .7)
            off = _wrap(atan2(towards[1] - self.pos[1], towards[0] - self.pos[0]) - self.yaw)
            self._look_dir = -1.0 if off < 0 else 1.0
        return self._look_dir

    def _wall_distance(self, p=None, window=.6):
        """Distance (m) from ``p`` (default: the drone) to the nearest believed-occupied cell
        (capped at ``window``)."""
        spec, occ = self.spec, self.belief.occ
        p = self.pos if p is None else np.asarray(p, float)
        r, c = (int(v) for v in spec.to_cell(p))
        k = int(np.ceil(window / spec.resolution))
        sub = occ[max(0, r - k):r + k + 1, max(0, c - k):c + k + 1]
        cells = np.argwhere(sub)
        if not len(cells):
            return window
        xy = spec.to_xy(cells + [max(0, r - k), max(0, c - k)])
        return float(max(0.0, np.linalg.norm(xy - p, axis=1).min() - spec.resolution / 2))

    def _commit(self, cost, new_poly):
        """Path commitment: keep the current route unless it is blocked, ends elsewhere, or the
        new one is more than ``commit_margin`` cheaper (both costed on the current map).

        Near an unmapped obstacle two routes can cost almost the same; without this the plan
        flipped between them every replan and the look-ahead turn never settled (Gazebo
        verify-3: 150 s turning in place)."""
        old = self.poly
        margin = self.cfg.commit_margin
        if margin <= 0 or old is None or len(old) < 2 or self.optimistic:
            return new_poly
        if np.linalg.norm(old[-1] - new_poly[-1]) > 1e-6:          # different goal cell
            return new_poly
        old = np.vstack([self.pos, old[1:]])
        old_cost, new_cost = self._poly_cost(cost, old), self._poly_cost(cost, new_poly)
        if np.isfinite(old_cost) and old_cost <= new_cost * (1 + margin):
            self.stats["kept"] = self.stats.get("kept", 0) + 1
            return old
        return new_poly

    def _poly_cost(self, cost, poly):
        """Weighted line cost (cell units) of an xy polyline, as Theta* measures it."""
        from .grid_search import polyline_cost
        spec = self.spec
        cells = (np.asarray(poly)[:, ::-1] + spec.half_size) / spec.resolution - .5   # (row, col) floats
        cells = np.clip(cells, 0, cost.shape[0] - 1)
        return polyline_cost(cost, [tuple(c) for c in cells])

    def _snap_goal(self, cost, cp):
        return snap_goal(cost, self.spec, self.env, cp, self.cfg.goal_beyond)

    def _fence(self, cp, width=.35):
        """Cells around the far side of a line checkpoint's goal pocket (its two flanks and its
        back). Planning treats them as blocked, so the only way into the pocket is across the
        line. Without it, phantom walls in the first corridor (fake-PX4 runs) sent the route
        round through the maze into the pocket from behind -> "goal without crossing" -> land.
        None for the pad and for the chicane (Gate B is a legitimate other way in)."""
        if cp.kind == "pad" or cp.other_route:
            return None
        d = self._grid_xy - cp.centre
        along = d @ cp.normal
        lateral = np.abs(d @ np.array([-cp.normal[1], cp.normal[0]]))
        depth = self.env.radius + .6                         # the goal pocket's far edge (snap_goal)
        flanks = (along >= 0) & (along <= depth + width) & (lateral > cp.half_width) & (lateral <= cp.half_width + width)
        back = (along > depth) & (along <= depth + width) & (lateral <= cp.half_width + width)
        return flanks | back

    def _fenced(self, cost, here):
        if cost is None or self.fence is None or self.fence[here] or self.router.kind == "dstar":
            return cost
        cost = cost.copy()
        cost[self.fence] = inf
        return cost

    def _near_line(self, cp, dist):
        """Cells within ``dist`` of checkpoint ``cp``'s line extended ``dist`` past each end."""
        xy = self._grid_xy
        if cp.kind == "pad":
            return np.linalg.norm(xy - cp.centre, axis=-1) < cp.half_width + dist
        lateral = np.array([-cp.normal[1], cp.normal[0]])
        return (np.abs((xy - cp.centre) @ cp.normal) < dist) & (np.abs((xy - cp.centre) @ lateral) < cp.half_width + dist)


# ------------------------------------------------------------------ geometry
def snap_goal(cost, spec, env, cp, goal_beyond=.10):
    """Nearest planable cell to checkpoint ``cp``'s goal that still lies fully past its line."""
    want = cp.goal_xy(env.radius + goal_beyond)
    rc = np.argwhere(np.isfinite(cost))
    if not len(rc):
        return None
    xy = spec.to_xy(rc)
    if cp.kind == "pad":
        ok = np.linalg.norm(xy - cp.centre, axis=1) <= cp.half_width
    else:
        along = (xy - cp.centre) @ cp.normal
        lateral = np.abs((xy - cp.centre) @ np.array([-cp.normal[1], cp.normal[0]]))
        # >= radius + 5 cm past the line, so reaching the goal implies the crossing fired first.
        # The goal may lie anywhere along the line's span: whether the whole drone crossed
        # inside the aperture is checked at the crossing itself (Checkpoint.crossed), and a
        # tight passage can force the route to cross off-centre.
        ok = (along >= env.radius + .05) & (lateral <= cp.half_width) & (along <= env.radius + .6)
    if not ok.any():
        return None
    k = int(np.argmin(np.linalg.norm(xy[ok] - want, axis=1)))
    return tuple(int(v) for v in rc[ok][k])


def _point_along(poly, dist):
    """The point ``dist`` metres along ``poly`` (or its end)."""
    remaining, cur = dist, poly[0]
    for p in poly[1:]:
        seg = float(np.linalg.norm(p - cur))
        if seg >= remaining:
            return cur + (p - cur) * (remaining / max(seg, 1e-12))
        remaining -= seg
        cur = p
    return cur


def _trim(poly, xy):
    """Drop the part of ``poly`` already behind ``xy`` (projected onto the nearest segment)."""
    if poly is None or len(poly) < 2:
        return poly
    a, b = poly[:-1], poly[1:]
    ab = b - a
    t = np.clip(((xy - a) * ab).sum(1) / np.maximum((ab * ab).sum(1), 1e-12), 0, 1)
    k = int(np.argmin(np.linalg.norm(a + t[:, None] * ab - xy, axis=1)))
    rest = poly[k + 1:]
    if len(rest) and np.linalg.norm(rest[0] - xy) < 1e-9:
        rest = rest[1:]
    return np.vstack([xy, rest]) if len(rest) else np.array([xy])


def _near_path(points, poly, dist):
    """True if any of ``points`` (K, 2) lies within ``dist`` of polyline ``poly``."""
    if poly is None or len(poly) < 2 or not len(points):
        return True
    a, b = poly[:-1], poly[1:]
    ab = b - a
    t = ((points[:, None, :] - a[None]) * ab[None]).sum(-1) / np.maximum((ab * ab).sum(-1), 1e-12)[None]
    closest = a[None] + np.clip(t, 0, 1)[..., None] * ab[None]
    return bool((np.linalg.norm(points[:, None, :] - closest, axis=-1) < dist).any())


def _wrap(a):
    return float(np.arctan2(np.sin(a), np.cos(a)))
