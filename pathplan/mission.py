"""Simulated world for ``OnlinePlanner``: kinematic drone + depth camera + evaluator.

The drone-side logic lives in ``online.OnlinePlanner`` (the same object the ROS 2
node runs). This module only plays the world around it:

* the camera (``DepthCamera.observe``) is ray-cast against the *true* course;
* a "move" command advances ``step_m`` along the commanded path; a "turn" yaws in
  place by at most ``turn_step_deg``;
* every step is checked against the true course for contact and clearance.

Planners: theta (default), astar, static (plans on the published map once per
checkpoint, no sensing: today's offline approach), dstar (experimental).
Kinematic only: no dynamics, tracking error or estimator drift
(``Envelope.tracking_margin`` is the allowance for them).
"""
from __future__ import annotations

from dataclasses import dataclass, field
from math import atan2, inf

import numpy as np

from .online import OnlinePlanner, PlannerConfig, _wrap
from .sensor import DepthCamera
from .world import Envelope, GridSpec, clearance


@dataclass
class MissionConfig(PlannerConfig):
    step_m: float = .10
    max_steps: int = 3000
    camera: DepthCamera = field(default_factory=DepthCamera)

    def validate(self):
        super().validate()
        if self.step_m <= 0:
            raise ValueError("step_m must be positive")
        return self


@dataclass
class MissionResult:
    planner: str
    trajectory: np.ndarray
    cleared: list
    outcome: str                   # finished | crashed | no_path | timeout
    collision_xy: list | None
    min_true_clearance: float
    keepout_violations: int        # samples inside radius + tracking margin of a true obstacle
    plan_calls: int
    replans: int                   # map changes near the path that triggered a replan
    plan_time_s: list
    expansions: int
    escapes: int
    belief: np.ndarray
    events: list
    turn_in_place_rad: float = 0.0

    @property
    def length(self):
        if len(self.trajectory) < 2:
            return 0.0
        return float(np.linalg.norm(np.diff(self.trajectory, axis=0), axis=1).sum())


def _advance(poly, start, dist):
    """Walk ``dist`` m from ``start`` along ``poly``. Returns (new position, points walked through)."""
    walked, remaining, cur = [start], dist, start
    for p in poly[1:]:
        seg = np.linalg.norm(p - cur)
        if seg >= remaining:
            cur = cur + (p - cur) * (remaining / max(seg, 1e-12))
            walked.append(cur)
            return cur, np.asarray(walked)
        remaining -= seg
        cur = p
        walked.append(cur)
    return cur, np.asarray(walked)


def _min_clearance(true_clear, spec, a, b):
    n = max(2, int(np.ceil(np.linalg.norm(b - a) / .01)) + 1)
    pts = a[None] + np.linspace(0, 1, n)[:, None] * (b - a)[None]
    rc = spec.to_cell(pts)
    vals = true_clear[rc[:, 0], rc[:, 1]]
    k = int(np.argmin(vals))
    return float(vals[k]), pts[k]


def run_mission(prior_occ, truth_occ, spec: GridSpec, env: Envelope, checkpoints, start_xy, start_yaw,
                cfg: MissionConfig | None = None, rng=None) -> MissionResult:
    cfg = (cfg or MissionConfig()).validate()
    rng = rng or np.random.default_rng(0)
    planner = OnlinePlanner(prior_occ, spec, env, checkpoints, cfg)
    true_clear = clearance(truth_occ, spec)       # evaluation only

    pos, yaw = np.asarray(start_xy, float), float(start_yaw)
    traj = [pos.copy()]
    min_clear, collision, turned, violations = inf, None, 0.0, 0
    max_turn = np.deg2rad(cfg.turn_step_deg)

    def sense():
        if planner.sensing:
            planner.observe(*cfg.camera.observe(truth_occ, spec, pos, yaw, rng, expected=prior_occ))

    sense()
    outcome = "timeout"
    for _ in range(cfg.max_steps):
        cmd = planner.update(pos, yaw)
        if cmd.mode == "done":
            outcome = "finished"
            break
        if cmd.mode == "failed":
            outcome = "no_path"
            break
        if cmd.mode == "turn":
            delta = float(np.clip(_wrap(cmd.yaw - yaw), -max_turn, max_turn))
            yaw += delta
            turned += abs(delta)
            sense()
            continue
        new, walked = _advance(cmd.path, pos, cfg.step_m)
        for a, b in zip(walked, walked[1:]):
            c, at = _min_clearance(true_clear, spec, a, b)
            min_clear = min(min_clear, c)
            violations += int(c < env.hard)
            if c < env.radius and collision is None:
                collision = at.round(3).tolist()
        if cmd.yaw is not None:
            yaw = cmd.yaw
        elif np.linalg.norm(new - pos) > 1e-9:
            yaw = atan2(new[1] - pos[1], new[0] - pos[0])
        pos = new
        traj.append(pos.copy())
        if collision is not None:
            outcome = "crashed"
            planner.update(pos, yaw)               # let it record checkpoints cleared on this step
            break
        sense()
    s = planner.stats
    return MissionResult(cfg.planner, np.asarray(traj), planner.cleared, outcome, collision, float(min_clear),
                         violations, s["plan_calls"], s["replans"], planner.plan_times, s["expansions"],
                         s["escapes"], planner.belief.occ, planner.events, turned)


def time_estimate(traj, *, v_max=1.2, a_max=.6, a_lat=.6, omega_max=.45, smooth_m=.4, ds=.01):
    """Arc-length time of a smoothed copy of ``traj`` with the team's fast-f5 limits.

    Heading is assumed to follow the path tangent (yaw rate = speed * curvature),
    which is conservative against the heading-DP reference.
    """
    traj = np.asarray(traj, float)
    seg = np.linalg.norm(np.diff(traj, axis=0), axis=1)
    traj = traj[np.r_[True, seg > 1e-9]]
    if len(traj) < 3:
        return 0.0
    s = np.r_[0, np.cumsum(np.linalg.norm(np.diff(traj, axis=0), axis=1))]
    grid = np.r_[np.arange(0, s[-1], ds), s[-1]]
    pts = np.column_stack([np.interp(grid, s, traj[:, k]) for k in range(2)])
    w = max(3, int(smooth_m / ds) | 1)
    kern = np.hanning(w)
    kern /= kern.sum()
    pad = np.vstack([np.repeat(pts[:1], w // 2, 0), pts, np.repeat(pts[-1:], w // 2, 0)])
    sm = np.column_stack([np.convolve(pad[:, k], kern, mode="valid") for k in range(2)])
    d1 = np.gradient(sm, grid, axis=0)
    d2 = np.gradient(d1, grid, axis=0)
    speed = np.linalg.norm(d1, axis=1)
    curv = np.abs(d1[:, 0] * d2[:, 1] - d1[:, 1] * d2[:, 0]) / np.maximum(speed, 1e-9) ** 3
    v = speed_profile(grid, curv, curv, v_max=v_max, a_max=a_max, a_lat=a_lat, omega_max=omega_max)
    return float(time_of(grid, v)[-1])


# Speed profile and timing, from the team's sim_stack/tools/prepare_fast_reference.py
# (Suraj Shivakumar), unchanged.
def speed_profile(s, curvature, dpsi_ds, *, v_max, a_max, a_lat, omega_max, v_floor=0., stops=()):
    v = np.full(len(s), v_max)
    v = np.minimum(v, np.sqrt(a_lat/np.maximum(curvature, 1e-9)))
    v = np.minimum(v, omega_max/np.maximum(np.abs(dpsi_ds), 1e-9))
    v = np.maximum(v, v_floor)
    v[0] = v[-1] = 0.
    v[list(stops)] = 0.
    ds = np.diff(s)
    for i in range(1, len(s)):
        v[i] = min(v[i], np.sqrt(v[i-1]**2+2*a_max*ds[i-1]))
    for i in range(len(s)-2, -1, -1):
        v[i] = min(v[i], np.sqrt(v[i+1]**2+2*a_max*ds[i]))
    return v


def time_of(s, v):
    ds = np.diff(s)
    vm = .5*(v[1:]+v[:-1])
    dt = np.where(vm > 1e-9, ds/np.maximum(vm, 1e-9), 0.)
    return np.r_[0, np.cumsum(np.maximum(dt, 0.))]
