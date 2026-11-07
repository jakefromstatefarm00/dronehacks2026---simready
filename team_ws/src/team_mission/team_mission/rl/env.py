"""Training environment: fly from checkpoint to checkpoint, slowly, inside a corridor.

One episode = the part of the mission the policy controls: start hovering at the takeoff point
(or part-way along the path), visit every checkpoint in order, and stop in a hover inside the
last checkpoint's radius. Takeoff and landing are NOT learned; the mission node does those.

The environment uses exactly the drone's code for everything that matters:
  * observation  -> team_mission.control.build_observation
  * action       -> team_mission.control.ActionMapper (carrot pose setpoint + yaw target)
  * reached/path -> team_mission.path.Path
and a randomised model of PX4 + cuVSLAM (team_mission.rl.dynamics) in between.

Paths: each episode is either the checkpoint file you train for (e.g. the course) or, with
probability `random_path_prob`, a random zig-zag, so the policy learns to follow ANY checkpoint
list, not to memorise one.

Requires gymnasium (`pip install gymnasium`). Not imported by the drone node.
"""
from __future__ import annotations

import math
from typing import Optional

import gymnasium as gym
import numpy as np

from team_mission.control import (ACT_DIM, OBS_DIM, ActionMapper, CarrotPolicy, Limits,
                                  PolicyContext, build_observation, speed_scale)
from team_mission.geometry import wrap_pi
from team_mission.path import CheckpointSpec, Path, resolve_path
from team_mission.rl.dynamics import DroneSim, DynamicsRanges


def random_path(rng: np.random.Generator) -> Path:
    n = int(rng.integers(3, 9))
    p = np.array([0.0, 0.0, 1.0])
    heading = rng.uniform(-math.pi, math.pi)
    pts = [p.copy()]
    for k in range(n):
        heading += rng.choice([0.0, rng.uniform(-2.1, 2.1)], p=[0.25, 0.75])
        L = rng.uniform(0.6, 2.5)
        z = float(np.clip(p[2] + rng.uniform(-0.3, 0.3), 0.6, 1.4))
        if k == n - 1:
            z = float(rng.uniform(0.5, 1.0))
        p = np.array([p[0] + L * math.cos(heading), p[1] + L * math.sin(heading), z])
        pts.append(p.copy())
    radius = np.concatenate([[0.0], rng.uniform(0.15, 0.3, n)])
    corridor = rng.uniform(0.2, 0.45, n + 1)
    corridor = np.maximum(corridor, radius + 0.05)
    return Path(np.array(pts), radius, corridor, ['takeoff'] + [f'r{k}' for k in range(n)])


class CheckpointEnv(gym.Env):
    metadata = {'render_modes': []}

    def __init__(self, checkpoints: Optional[str] = None, limits: Optional[Limits] = None,
                 dynamics: Optional[DynamicsRanges] = None, randomize: bool = True,
                 random_path_prob: float = 0.5, mid_start_prob: float = 0.5,
                 jitter_m: float = 0.10, residual_scale: float = 0.0, course_obj: Optional[str] = None,
                 seed: Optional[int] = None, reward: Optional[dict] = None):
        super().__init__()
        self.limits = limits or Limits()
        self.mapper = ActionMapper(self.limits)
        self.ranges = dynamics or (DynamicsRanges() if randomize else DynamicsRanges.nominal())
        self.spec = CheckpointSpec.load(checkpoints) if checkpoints else None
        self.train_start = None
        if self.spec is not None:
            import yaml
            with open(checkpoints) as fh:
                st = (yaml.safe_load(fh) or {}).get('train_start')
            if st:
                self.train_start = (np.asarray(st['xyz'], float), math.radians(float(st.get('yaw_deg', 0.0))))
        self.random_path_prob = random_path_prob if self.spec is not None else 1.0
        self.mid_start_prob = mid_start_prob
        self.jitter = jitter_m
        self.residual_scale = residual_scale
        self.base = CarrotPolicy() if residual_scale > 0 else None
        self.sim = DroneSim()
        self.course = None
        if course_obj:
            from team_mission.rl.course_collision import load_course
            self.course = load_course(course_obj)
        self.w = dict(progress=2.0, checkpoint=1.0, finish=5.0, crash=-10.0, lateral=-0.05,
                      smooth=-0.02, heading=-0.01, overspeed=-0.5, time=-0.005)
        self.w.update(reward or {})
        self.observation_space = gym.spaces.Box(-3.0, 3.0, (OBS_DIM,), np.float32)
        self.action_space = gym.spaces.Box(-1.0, 1.0, (ACT_DIM,), np.float32)
        self.rng = np.random.default_rng(seed)

    # ------------------------------------------------------------------ episode setup
    def _course_path(self) -> Path:
        if self.train_start is not None:
            g, yaw = self.train_start
        else:
            g, yaw = np.zeros(3), 0.0
        path = resolve_path(self.spec, g, yaw, 1.0)  # course frame == odom frame for training
        if self.jitter > 0:  # the real course is never exactly the published one
            path.points[1:] += self.rng.normal(0.0, self.jitter / 2, path.points[1:].shape) * np.array([1, 1, 0.3])
            path.__post_init__()
        return path

    def reset(self, *, seed=None, options=None):
        if seed is not None:
            self.rng = np.random.default_rng(seed)
        use_random = self.spec is None or self.rng.random() < self.random_path_prob
        self.path = random_path(self.rng) if use_random else self._course_path()
        self.on_course = not use_random
        self.sim.sample(self.ranges, self.rng)
        P = self.path
        self.i = 1
        start = P.points[0].copy()
        d = P.points[1] - P.points[0]
        yaw = math.atan2(d[1], d[0]) + self.rng.uniform(-0.6, 0.6)
        if self.rng.random() < self.mid_start_prob and P.n > 1:
            self.i = int(self.rng.integers(1, P.n + 1))
            a, b = P.points[self.i - 1], P.points[self.i]
            start = a + self.rng.uniform(0.0, 0.6) * (b - a)
            yaw = math.atan2(*(b - a)[[1, 0]]) + self.rng.uniform(-0.6, 0.6)
        start = start + self.rng.normal(0, 0.05, 3)
        self.sim.reset(start, yaw, self.rng, vel=self.rng.normal(0, 0.05, 3))
        self.prev_action = np.zeros(ACT_DIM)
        self.steps = 0
        self.max_steps = int((P.length / 0.12 + 20.0) / self.sim.dt)
        self.prog = P.progress(self.sim.p, self.i)
        self.max_lat = 0.0
        self.reached = 0
        return self._obs(), {}

    def _state(self):
        self.p_est, self.v_est, self.yaw_est = self.sim.estimate()

    def _obs(self):
        self._state()
        return build_observation(self.p_est, self.v_est, self.yaw_est, self.path, self.i,
                                 self.prev_action, self.limits)

    def context(self) -> PolicyContext:
        return PolicyContext(self.p_est, self.v_est, self.yaw_est, self.path, self.i, self.limits)

    # ------------------------------------------------------------------ step
    def step(self, action):
        a = np.clip(np.asarray(action, float), -1.0, 1.0)
        if self.base is not None:
            a = np.clip(self.base.act(None, self.context()) + self.residual_scale * a, -1.0, 1.0)
        cmd = self.mapper.command(a, self.p_est, self.yaw_est, speed_scale(self.path, self.i))
        self.sim.step(cmd.carrot, cmd.yaw_target)
        self.steps += 1
        P, w, sim = self.path, self.w, self.sim
        r, terminated, info = 0.0, False, {}

        prog = P.progress(sim.p, self.i)
        r += w['progress'] * (prog - self.prog)
        self.prog = prog
        self._state()                                   # the estimate the drone decides on
        # checkpoint bookkeeping uses the estimate, exactly like the node
        while self.i < P.n and P.reached(self.p_est, self.i):
            self.i += 1
            self.reached += 1
            r += w['checkpoint']
            self.prog = P.progress(sim.p, self.i)
        lat, corridor = P.off_path(sim.p, self.i)
        self.max_lat = max(self.max_lat, lat)
        r += w['lateral'] * (lat / corridor) ** 2
        r += w['smooth'] * float(np.sum((a - self.prev_action) ** 2))
        speed = float(np.linalg.norm(sim.v[:2]))
        travel = sim.v[:2] if speed > 0.1 else (P.points[self.i] - sim.p)[:2]
        if np.linalg.norm(travel) > 1e-6:
            r += w['heading'] * (1.0 - math.cos(wrap_pi(math.atan2(travel[1], travel[0]) - sim.yaw)))
        over = speed - 1.25 * self.limits.max_xy_speed * speed_scale(P, self.i)
        if over > 0:
            r += w['overspeed'] * over
        r += w['time']

        reason = ''
        if lat > corridor:
            reason = 'left the corridor'
        elif not (0.25 <= sim.p[2] <= 1.8):
            reason = 'altitude out of band'
        elif self.course is not None and self.course.collides(sim.p[None, :], 0.25, 0.10)[0]:
            reason = 'hit the course'
        if reason:
            r += w['crash']
            terminated = True
        elif self.i == P.n and P.reached(self.p_est, P.n) and np.linalg.norm(sim.v) < 0.15:
            r += w['finish']
            terminated = True
            reason = 'finished'
            self.reached += 1
        truncated = (not terminated) and self.steps >= self.max_steps
        if truncated:
            reason = 'timeout'
        self.prev_action = a
        if terminated or truncated:
            info = {'success': reason == 'finished', 'reason': reason, 'time_s': self.steps * sim.dt,
                    'checkpoints': self.reached, 'n_checkpoints': P.n, 'max_lateral_m': self.max_lat,
                    'on_course': self.on_course}
        obs = build_observation(self.p_est, self.v_est, self.yaw_est, P, self.i, self.prev_action, self.limits)
        return obs, float(r), terminated, truncated, info
