"""What the policy sees, what it outputs, and how that becomes a setpoint.

THIS FILE IS THE CONTRACT BETWEEN TRAINING AND THE DRONE. The training environment
(team_mission/rl/env.py) and the drone node (team_mission/mission_node.py) both call
build_observation() and ActionMapper.command() from here, so a policy sees exactly the same
numbers in both. If you change anything here, bump OBS_VERSION and retrain.

Observation (OBS_DIM = 20 floats, all roughly in [-1, 1], heading frame H = level, x forward):
    0:3   next checkpoint relative to the drone, H frame, clipped to 2 m, / 2
    3:6   the checkpoint after that (same as 0:3 at the last one), H frame, clipped to 2 m, / 2
    6:9   velocity estimate, H frame, / 0.5 m/s, clipped to +-2
    9:11  sin, cos of (direction to next checkpoint - current yaw)
    11:14 sideways offset from the current path segment, H frame, / 0.5 m, clipped to +-2
    14:18 previous action
    18    1.0 if the next checkpoint is the last one (the landing spot), else 0.0
    19    corridor half-width of the current segment / 0.5 m

Action (4 floats in [-1, 1]):
    0, 1  forward / left velocity, times max_xy_speed (horizontal norm clipped to max_xy_speed)
    2     up velocity, times max_z_speed
    3     yaw rate (CCW), times max_yaw_rate

Tight segments are flown slower automatically: the speeds above are multiplied by
speed_scale(path, i) = clip(corridor / 0.30 m, 0.5, 1.0), so a 0.20 m corridor (the zone-1 maze)
gets 2/3 of the speed and the 0.18 m window 0.6 of it.

How an action is flown (both in training and on the drone): it becomes a POSE setpoint
("carrot") a short distance ahead of the drone, carrot = position + velocity * lead_time, capped
at max_carrot_m, and a yaw target = yaw + yaw_rate * yaw_lead_s. PX4's position controller then
flies toward the carrot. A pose setpoint is used (not a twist) because if our node ever stops,
flight_interface keeps holding the LAST pose: a point at most max_carrot_m ahead of the drone.
A stale twist instead falls back to the last *pose* we sent, which could be metres behind us.
"""
from __future__ import annotations

import json
import math
from dataclasses import asdict, dataclass

import numpy as np

from team_mission.geometry import clip_norm, heading_to_world, world_to_heading, wrap_pi
from team_mission.path import Path

OBS_VERSION = 2          # the contract version: bump on ANY change to observation or action meaning
OBS_DIM = 20
ACT_DIM = 4


@dataclass
class Limits:
    """Speed limits. These are deliberately SLOW. Training and the drone use the same numbers."""
    max_xy_speed: float = 0.40      # m/s
    max_z_speed: float = 0.25       # m/s
    max_yaw_rate: float = 0.50      # rad/s
    lead_time: float = 1.0          # s; carrot = velocity * lead_time ahead (~1 / PX4 MPC_XY_P)
    max_carrot_m: float = 0.60      # m; never put the setpoint further than this from the drone
    yaw_lead_s: float = 0.6         # s

    @staticmethod
    def from_dict(d) -> 'Limits':
        base = asdict(Limits())
        base.update({k: float(v) for k, v in (d or {}).items() if k in base})
        return Limits(**base)


# ----------------------------------------------------------------------------- observation
def build_observation(pos, vel_world, yaw: float, path: Path, i: int, prev_action,
                      limits: Limits) -> np.ndarray:
    """pos, vel_world: estimate in odom ENU. i: index of the checkpoint we are heading for (1..n)."""
    pos = np.asarray(pos, float)
    i = int(min(max(i, 1), path.n))
    j = min(i + 1, path.n)
    rel1 = world_to_heading(clip_norm(path.points[i] - pos, 2.0), yaw) / 2.0
    rel2 = world_to_heading(clip_norm(path.points[j] - pos, 2.0), yaw) / 2.0
    vel = np.clip(world_to_heading(vel_world, yaw) / 0.5, -2.0, 2.0)
    d = path.points[i] - pos
    if math.hypot(d[0], d[1]) < 0.05:
        d = path.points[i] - path.points[i - 1]
    err = wrap_pi(math.atan2(d[1], d[0]) - yaw) if math.hypot(d[0], d[1]) > 1e-6 else 0.0
    lat = np.clip(world_to_heading(path.lateral(pos, i), yaw) / 0.5, -2.0, 2.0)
    obs = np.concatenate([rel1, rel2, vel, [math.sin(err), math.cos(err)], lat,
                          np.clip(np.asarray(prev_action, float), -1.0, 1.0),
                          [1.0 if i == path.n else 0.0], [path.corridor[i] / 0.5]])
    return obs.astype(np.float32)


# ----------------------------------------------------------------------------- action -> setpoint
def speed_scale(path: Path, i: int) -> float:
    """Slower in tight corridors: 1.0 at >= 0.30 m, down to 0.5."""
    i = int(min(max(i, 1), path.n))
    return float(np.clip(path.corridor[i] / 0.30, 0.5, 1.0))


@dataclass
class Command:
    carrot: np.ndarray       # pose setpoint position, odom ENU
    yaw_target: float        # pose setpoint yaw, odom ENU
    velocity: np.ndarray     # the velocity the action asked for (world), for logs
    yaw_rate: float


class ActionMapper:
    def __init__(self, limits: Limits):
        self.limits = limits

    def velocity(self, action, yaw: float, scale: float = 1.0):
        L = self.limits
        a = np.clip(np.nan_to_num(np.asarray(action, float), nan=0.0), -1.0, 1.0)
        vmax = L.max_xy_speed * scale
        v_h = np.array([a[0] * vmax, a[1] * vmax, a[2] * L.max_z_speed])
        v_h[:2] = clip_norm(v_h[:2], vmax)
        return heading_to_world(v_h, yaw), float(a[3] * L.max_yaw_rate)

    def command(self, action, pos, yaw: float, scale: float = 1.0) -> Command:
        """scale: speed_scale(path, i) for the segment being flown."""
        L = self.limits
        v_w, r = self.velocity(action, yaw, scale)
        carrot = np.asarray(pos, float) + clip_norm(v_w * L.lead_time, L.max_carrot_m)
        return Command(carrot, wrap_pi(yaw + r * L.yaw_lead_s), v_w, r)


# ----------------------------------------------------------------------------- policies
@dataclass
class PolicyContext:
    """Raw state for hand-written policies. A neural policy only uses the observation."""
    pos: np.ndarray
    vel: np.ndarray
    yaw: float
    path: Path
    i: int
    limits: Limits


class CarrotPolicy:
    """Hand-written baseline (pure pursuit). Flies the path with no training at all.

    Uses: (1) the default policy until a trained one beats it, (2) the 'teacher' for behaviour
    cloning, (3) the base for residual RL (the network learns a correction on top of it).
    """
    name = 'carrot'

    def __init__(self, lookahead_m: float = 0.35, slow_zone_m: float = 0.8, yaw_gain: float = 1.2):
        self.lookahead = lookahead_m
        self.slow_zone = slow_zone_m
        self.yaw_gain = yaw_gain

    def act(self, obs, ctx: PolicyContext) -> np.ndarray:
        P, i, L = ctx.path, ctx.i, ctx.limits
        pos = np.asarray(ctx.pos, float)
        s = P.progress(pos, i)
        target = P.point_at(min(s + self.lookahead, P.cum[i]))
        to_target = target - pos
        dist_cp = float(np.linalg.norm(P.points[i] - pos))
        # slow down for corners in proportion to how sharp they are; stop at the last checkpoint
        if i == P.n:
            corner_factor = 0.0
        else:
            u = P.points[i] - P.points[i - 1]
            w = P.points[i + 1] - P.points[i]
            nu, nw = np.linalg.norm(u), np.linalg.norm(w)
            cos_turn = float(u @ w / (nu * nw)) if nu > 1e-6 and nw > 1e-6 else 1.0
            corner_factor = 0.15 + 0.85 * ((1.0 + cos_turn) / 2.0) ** 2   # 90 deg -> 0.36, U-turn -> 0.15
        f = 1.0 if dist_cp >= self.slow_zone else corner_factor + (1 - corner_factor) * dist_cp / self.slow_zone
        speed = L.max_xy_speed * max(f, 0.12 if i < P.n else 0.0)
        if i == P.n:  # final approach: speed proportional to distance, never overshoot
            speed = min(L.max_xy_speed, 0.8 * dist_cp)
        d = to_target / max(np.linalg.norm(to_target), 1e-6)
        v_w = d * speed
        v_w[2] = np.clip(1.2 * (target[2] - pos[2]), -L.max_z_speed, L.max_z_speed)
        v_h = world_to_heading(v_w, ctx.yaw)
        # face the direction of travel (the camera needs to see where we go)
        hv = P.points[i] - pos
        if math.hypot(hv[0], hv[1]) < 0.1:
            hv = P.points[i] - P.points[i - 1]
        err = wrap_pi(math.atan2(hv[1], hv[0]) - ctx.yaw) if math.hypot(hv[0], hv[1]) > 1e-6 else 0.0
        yaw_rate = np.clip(self.yaw_gain * err, -L.max_yaw_rate, L.max_yaw_rate)
        a = np.array([v_h[0] / L.max_xy_speed, v_h[1] / L.max_xy_speed,
                      v_h[2] / L.max_z_speed, yaw_rate / L.max_yaw_rate])
        return np.clip(a, -1.0, 1.0)


class MLPPolicy:
    """A trained network exported by `python -m team_mission.rl.export` (numpy only, no torch).

    The .npz holds W0, b0, W1, b1, ... (tanh hidden layers, linear output) and a JSON 'meta'
    string. If meta.residual_scale > 0 the network output is a CORRECTION added to the carrot
    baseline: action = clip(carrot + residual_scale * network).
    """
    def __init__(self, path: str):
        data = np.load(path, allow_pickle=False)
        self.meta = json.loads(str(data['meta']))
        if int(self.meta.get('obs_version', -1)) != OBS_VERSION or int(self.meta['obs_dim']) != OBS_DIM:
            raise ValueError(f'{path}: trained for obs_version {self.meta.get("obs_version")} / '
                             f'dim {self.meta.get("obs_dim")}, this code is {OBS_VERSION} / {OBS_DIM}. Retrain.')
        n = int(self.meta['n_layers'])
        self.W = [data[f'W{k}'].astype(np.float64) for k in range(n)]
        self.b = [data[f'b{k}'].astype(np.float64) for k in range(n)]
        self.residual_scale = float(self.meta.get('residual_scale', 0.0))
        self.base = CarrotPolicy() if self.residual_scale > 0 else None
        self.name = self.meta.get('name', path)
        self.limits = Limits.from_dict(self.meta.get('limits'))

    def network(self, obs) -> np.ndarray:
        x = np.asarray(obs, np.float64)
        for k, (W, b) in enumerate(zip(self.W, self.b)):
            x = W @ x + b
            if k < len(self.W) - 1:
                x = np.tanh(x)
        return x

    def act(self, obs, ctx: PolicyContext) -> np.ndarray:
        out = self.network(obs)
        if self.base is not None:
            out = self.base.act(obs, ctx) + self.residual_scale * np.clip(out, -1.0, 1.0)
        return np.clip(out, -1.0, 1.0)


def load_policy(spec: str):
    """'carrot' -> the baseline; anything else -> path to an exported .npz."""
    if spec in ('', 'carrot', 'baseline'):
        return CarrotPolicy()
    return MLPPolicy(spec)

