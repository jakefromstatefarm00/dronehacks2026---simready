"""A fast model of "the drone + PX4 position controller + cuVSLAM", for training.

We never command motors. We send a pose setpoint (the carrot) and PX4 flies to it. So the thing
to model is how the drone's velocity responds to a carrot, not rotor physics:

    PX4 position loop  v_ref = Kp * (carrot - p_estimated)          (clipped to px4_max_speed)
    PX4 velocity loop  a = (v_ref - v) / tau + vel_i * integral(v_ref - v)   (clipped to max_accel)
    the world          v += (a + disturbance) dt,  p += v dt
    yaw                yaw_rate = Kyaw * (yaw_target - yaw_estimated) (clipped), with a lag
    cuVSLAM            p_estimated = p + drift + noise,  v_estimated = v + noise
                       (drift is a random walk that grows with distance flown, plus a little with time)
    the link           setpoints arrive delay_steps control steps late

Every number above is drawn from a RANGE at the start of each episode (domain randomisation),
so the policy learns to fly any drone inside the ranges. Measure your real drone with
`python -m team_mission.rl.fit_dynamics <flight_log.csv>` and narrow or move the ranges in a
dynamics YAML (see config/dynamics_default.yaml) when the real numbers are known.
"""
from __future__ import annotations

import math
from dataclasses import asdict, dataclass, field
from typing import Dict, Tuple

import numpy as np

from team_mission.geometry import wrap_pi

Range = Tuple[float, float]


@dataclass
class DynamicsRanges:
    kp_xy: Range = (0.7, 1.3)            # 1/s, PX4 MPC_XY_P (default 0.95)
    kp_z: Range = (0.7, 1.3)             # 1/s, PX4 MPC_Z_P (default 1.0)
    tau: Range = (0.2, 0.6)              # s, velocity response time constant
    vel_i: Range = (0.5, 2.0)            # 1/s^2, PX4 velocity-loop integral (cancels steady pushes)
    max_accel: Range = (1.5, 4.0)        # m/s^2
    px4_max_speed: Range = (1.0, 2.0)    # m/s, the carrot never asks for more anyway
    k_yaw: Range = (1.5, 3.0)            # 1/s, PX4 MC_YAW_P-ish
    max_yaw_rate: Range = (0.6, 1.2)     # rad/s
    yaw_tau: Range = (0.05, 0.25)        # s
    delay_steps: Tuple[int, int] = (0, 3)  # control steps (50 ms each at 20 Hz)
    pos_noise: Range = (0.002, 0.015)    # m, cuVSLAM position jitter (std)
    vel_noise: Range = (0.01, 0.06)      # m/s
    drift_per_sqrt_m: Range = (0.0, 0.02)  # m per sqrt(metre flown): cuVSLAM drift grows with distance
    drift_rate: Range = (0.0, 0.003)     # m/sqrt(s): small drift even when hovering
    yaw_drift_rate: Range = (0.0, 0.01)  # rad/sqrt(s)
    disturbance: Range = (0.0, 0.35)     # m/s^2 std of a slowly changing push (prop wash, air)
    disturbance_tau: Range = (0.5, 3.0)  # s

    @staticmethod
    def from_dict(d) -> 'DynamicsRanges':
        base = asdict(DynamicsRanges())
        for k, v in (d or {}).items():
            if k in base:
                base[k] = tuple(v)
        return DynamicsRanges(**base)

    @staticmethod
    def nominal() -> 'DynamicsRanges':
        """No randomisation: every range collapsed to its middle, no noise."""
        r = DynamicsRanges()
        out = {}
        for k, v in asdict(r).items():
            m = (v[0] + v[1]) / 2
            out[k] = (int(round(m)), int(round(m))) if k == 'delay_steps' else (m, m)
        for k in ('pos_noise', 'vel_noise', 'drift_rate', 'drift_per_sqrt_m', 'yaw_drift_rate', 'disturbance'):
            out[k] = (0.0, 0.0)
        out['delay_steps'] = (1, 1)
        return DynamicsRanges(**out)


@dataclass
class DroneSim:
    dt: float = 0.05                 # control period (20 Hz, same as the node)
    substeps: int = 5
    params: Dict[str, float] = field(default_factory=dict)

    def sample(self, ranges: DynamicsRanges, rng: np.random.Generator):
        p = {}
        for k, (lo, hi) in asdict(ranges).items():
            p[k] = int(rng.integers(lo, hi + 1)) if k == 'delay_steps' else float(rng.uniform(lo, hi))
        self.params = p
        return p

    def reset(self, pos, yaw: float, rng: np.random.Generator, vel=None):
        self.rng = rng
        self.p = np.asarray(pos, float).copy()
        self.v = np.zeros(3) if vel is None else np.asarray(vel, float).copy()
        self.yaw = float(yaw)
        self.yaw_rate = 0.0
        self.bias = np.zeros(3)
        self.yaw_bias = 0.0
        self.dist = np.zeros(3)
        self.vi = np.zeros(3)
        self.queue = []
        self.t = 0.0

    # ---------------------------------------------------------------- what cuVSLAM reports
    def estimate(self):
        P = self.params
        p_est = self.p + self.bias + self.rng.normal(0.0, P['pos_noise'], 3)
        v_est = self.v + self.rng.normal(0.0, P['vel_noise'], 3)
        yaw_est = wrap_pi(self.yaw + self.yaw_bias + self.rng.normal(0.0, 0.003))
        return p_est, v_est, yaw_est

    # ---------------------------------------------------------------- one control step
    def step(self, carrot, yaw_target: float, velocity=None, ground_z=None):
        """Pose mode: fly to `carrot`. Velocity mode (velocity given, carrot None): follow it
        (flight_interface twist). ground_z: if given, the drone cannot go below it."""
        P = self.params
        item = ('v', np.asarray(velocity, float), None) if velocity is not None else \
            ('p', np.asarray(carrot, float), float(yaw_target))
        self.queue.append(item)
        if len(self.queue) > P['delay_steps']:
            kind, target, yaw_target = self.queue.pop(0)
        else:  # the very first steps after reset: hold where we are
            kind, target, yaw_target = 'p', self.p + self.bias, self.yaw + self.yaw_bias
        if yaw_target is None:
            yaw_target = self.yaw + self.yaw_bias
        h = self.dt / self.substeps
        for _ in range(self.substeps):
            p_px4 = self.p + self.bias            # PX4 fuses cuVSLAM, so it believes the drift too
            if kind == 'v':
                v_ref = target.copy()
            else:
                e = target - p_px4
                v_ref = np.array([P['kp_xy'] * e[0], P['kp_xy'] * e[1], P['kp_z'] * e[2]])
            n = np.linalg.norm(v_ref)
            if n > P['px4_max_speed']:
                v_ref *= P['px4_max_speed'] / n
            err = v_ref - self.v
            self.vi = np.clip(self.vi + err * h, -0.5, 0.5)
            a = err / max(P['tau'], 1e-3) + P['vel_i'] * self.vi
            na = np.linalg.norm(a)
            if na > P['max_accel']:
                a *= P['max_accel'] / na
            # slowly varying push (Ornstein-Uhlenbeck)
            th = 1.0 / max(P['disturbance_tau'], 1e-3)
            self.dist += -th * self.dist * h + P['disturbance'] * math.sqrt(2 * th * h) * self.rng.normal(0, 1, 3)
            self.dist[2] *= 0.5
            self.v += (a + self.dist) * h
            step = self.v * h
            self.p += step
            moved = float(np.linalg.norm(step))
            self.bias += self.rng.normal(0.0, P['drift_per_sqrt_m'] * math.sqrt(moved), 3) * np.array([1, 1, 0.5])
            if ground_z is not None and self.p[2] <= ground_z:
                self.p[2] = ground_z
                if self.v[2] < 0:
                    self.v[:] = 0.0          # sitting on the ground: no sliding
            yr_ref = np.clip(P['k_yaw'] * wrap_pi(yaw_target - (self.yaw + self.yaw_bias)),
                             -P['max_yaw_rate'], P['max_yaw_rate'])
            self.yaw_rate += (yr_ref - self.yaw_rate) * min(1.0, h / max(P['yaw_tau'], 1e-3))
            self.yaw = wrap_pi(self.yaw + self.yaw_rate * h)
            self.bias += self.rng.normal(0.0, P['drift_rate'] * math.sqrt(h), 3) * np.array([1, 1, 0.5])
            self.yaw_bias += self.rng.normal(0.0, P['yaw_drift_rate'] * math.sqrt(h))
        self.t += self.dt
