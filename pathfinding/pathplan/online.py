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


@dataclass
class PlannerConfig:
    planner: str = "theta"          # theta | astar | static (no sensing) | dstar (experimental)
    goal_beyond: float = .10        # goal sits this far past (radius beyond) each line
    suspect_penalty: float = 8.0    # see BeliefMap; 0 disables
    lookahead_m: float = .7         # 0 = no yaw command (caller faces the motion)
    turn_in_place_deg: float = 30.
    turn_step_deg: float = 15.
    replan_every_m: float = .5

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
        self.router = Router(self.cfg.planner)
        self.sensing = self.cfg.planner != "static"
        self.cleared, self.events, self.plan_times = [], [], []
        self.stats = dict(expansions=0, replans=0, plan_calls=0, escapes=0)
        self.pos = self.yaw = None
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
        cp = self.checkpoint
        if cp is None:
            self.status = "done"
            return
        self.trusted = self._near_line(cp, .5)               # checkpoint structure is never doubted
        self.goal = self._snap_goal(self.belief.cost, cp)
        if self.goal is None:
            self._fail("no goal cell")

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
            # a newly seen obstacle puts us inside the keep-out: back out to the nearest planable cell
            free = np.argwhere(np.isfinite(cost))
            if not len(free):
                self._fail("no free cell")
                return Command("failed", reason="no free cell")
            target = spec.to_xy(free[np.argmin(np.linalg.norm(spec.to_xy(free) - self.pos, axis=1))])
            self.stats["escapes"] += 1
            self._event("escape")
            self.router.reset()
            self.poly, self.dirty = None, True
            return Command("move", np.array([self.pos, target]), self.yaw, "escape")

        if not np.isfinite(cost[self.goal]):
            goal = self._snap_goal(cost, self.checkpoint)
            if goal is None and self.optimistic:
                goal = self._snap_goal(relaxed, self.checkpoint)
            if goal is None:
                self._fail("no goal cell")
                return Command("failed", reason="no goal cell")
            self.goal = goal
            self.router.reset()
            self.dirty = True
        if self.sensing and self.since_plan >= cfg.replan_every_m:
            self.dirty = True

        if self.dirty or self.poly is None or len(self.poly) < 2:
            self.since_plan, self.dirty = 0.0, False
            t = time.perf_counter()
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
                # 1) look around once, sensing; 2) doubt unconfirmed published walls; 3) give up
                if self.looked < 2 * np.pi:
                    if self.looked == 0:
                        self._event("look around")
                    turn = np.deg2rad(cfg.turn_step_deg)
                    self.looked += turn
                    self.dirty = True
                    return Command("turn", yaw=self.yaw + turn, reason="look around")
                if not self.optimistic:
                    self.optimistic, self.dirty = True, True
                    self._event("doubt unconfirmed map")
                    return self._decide()
                self._fail("no path")
                return Command("failed", reason="no path")
            self.looked = 0.0
            # start at the true position; a one-cell plan (already in the goal cell) still
            # flies to the cell centre instead of stalling
            self.poly = np.vstack([self.pos, spec.to_xy(np.array(cells[1:] or cells[-1:]))])

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

    def _snap_goal(self, cost, cp):
        return snap_goal(cost, self.spec, self.env, cp, self.cfg.goal_beyond)

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
