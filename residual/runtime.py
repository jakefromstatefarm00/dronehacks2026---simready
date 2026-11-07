"""Residual carrot policy on top of Jolie's FlyX-proven mission: the runtime (numpy only).

This module is the single source of the residual: the training simulator imports it, and
jolie_course/build_residual_copy.py pastes it verbatim into the generated flight files.

Where it acts
-------------
Jolie's ``Mission.plan()`` computes a carrot ``base_carrot`` metres along the planner's safe
path (0.5 m aligned, down to 0.15 m while the nose is still turning) and ``_limit_yaw`` caps the
commanded heading at 0.25 rad/s.  The residual adjusts exactly those two things, and only in
NAVIGATE:

    carrot length  x s,      s in [0.6, 1.3]   (1.0 = base)   shorter = slower, longer = faster
    lateral offset  l,       l in [-0.08, 0.08] m (0 = base)  sideways, away from a near wall
    yaw-rate cap   x y,      y in [0.6, 1.0]   (1.0 = base)   slower heading change (VIO)

Safety, by construction:
* the base stays in charge: route, turn-in-place, checkpoints, pad centring, landing and every
  guard are Jolie's code, untouched; the residual only moves the setpoint along / beside the
  planner's own safe chord;
* lengthening (s > 1) uses an extension point that the planner process checked against the
  keep-out on the drone's belief map (straight lines current->ext and chord-end->ext are finite
  cost); otherwise s is capped at 1;
* a lateral offset is applied only to a side the planner process checked the same way;
* zero correction (the exact base target and cap) whenever the inputs are stale or invalid:
  odometry older than ``STALE_ODOM_S``, a control stall in the last ``STALL_HOLDOFF_S``,
  missing planner info, non-finite numbers, or any state other than NAVIGATE.
"""
import math
from collections import deque

import numpy as np

RESIDUAL_FORMAT = "jolie-residual-mlp-v1"
CARROT_SCALE_MIN, CARROT_SCALE_MAX = 0.6, 1.3
LATERAL_MAX_M = 0.08
YAW_SCALE_MIN = 0.6
EXT_FACTOR = CARROT_SCALE_MAX          # the extension point is this x carrot_m along the plan
PROBE_ANGLES = (0.0, math.pi / 4, -math.pi / 4, math.pi / 2, -math.pi / 2)
PROBE_RANGE_M = 1.2
PROBE_STEP_M = 0.025
STALE_ODOM_S = 0.3                     # = MissionConfig.pose_max_age_s
STALL_GAP_S = 0.15                     # control-loop gap counted as a stall (the node's own rule)
STALL_HOLDOFF_S = 0.5
OBS_DIM = 20
ZERO = (1.0, 0.0, 1.0)                 # (carrot scale, lateral m, yaw-rate scale): the base


# ------------------------------------------------------------------------------ planner side
def probe_walls(occ, spec, xy, heading):
    """Distance (m, <= PROBE_RANGE_M) from xy to the first believed-occupied cell along each
    PROBE_ANGLES direction relative to ``heading``. Drone-side: the belief map only."""
    t = np.arange(PROBE_STEP_M, PROBE_RANGE_M + 1e-9, PROBE_STEP_M)
    ang = heading + np.asarray(PROBE_ANGLES)
    pts = np.asarray(xy, float)[None, None, :] + t[None, :, None] * np.stack([np.cos(ang), np.sin(ang)], 1)[:, None, :]
    rc = np.floor((pts[..., ::-1] + spec.half_size) / spec.resolution).astype(int)
    rc = np.clip(rc, 0, occ.shape[0] - 1)
    hit = occ[rc[..., 0], rc[..., 1]]
    first = np.where(hit.any(1), hit.argmax(1), len(t))
    return np.where(first < len(t), t[np.minimum(first, len(t) - 1)] - PROBE_STEP_M / 2, PROBE_RANGE_M)


def residual_info(planner, spec, xy, yaw, cmd, carrot_m, safe_carrot_path, line_cost, point_along):
    """What the mission-side residual needs from the planner process (drone-side only):
    wall probes on the belief map, the next checkpoint, the safe extension point for a longer
    carrot and which lateral sides are clear. None for done/failed."""
    if cmd.mode not in ("move", "turn"):
        return None
    cost, occ = planner.belief.cost, planner.belief.occ
    xy = np.asarray(xy, float)
    cp = planner.checkpoint
    info = {"cp_index": int(planner.index), "cp_dist": float(np.linalg.norm(cp.centre - xy)) if cp is not None else 0.0,
            "ext": None, "lat_ok": (False, False)}
    heading = float(yaw)
    if cmd.mode == "move" and cmd.path is not None and len(cmd.path) >= 2:
        cell = lambda p: np.clip((np.asarray(p, float)[::-1] + spec.half_size) / spec.resolution - .5, 0, cost.shape[0] - 1)
        ok = lambda a, b: bool(np.isfinite(line_cost(cost, cell(a), [cell(b)])[0]))
        base = safe_carrot_path(cost, spec, xy, cmd.path, carrot_m)       # what the child sends
        full = point_along(cmd.path, carrot_m)
        chord = base[1] - xy
        if np.linalg.norm(chord) > 1e-6:
            heading = math.atan2(chord[1], chord[0])
        if np.allclose(base[1], full):
            ext = point_along(cmd.path, carrot_m * EXT_FACTOR)
            if np.linalg.norm(ext - base[1]) > 1e-6 and ok(xy, ext) and ok(base[1], ext):
                info["ext"] = [float(ext[0]), float(ext[1])]
        if np.linalg.norm(chord) > 1e-6:
            n = np.array([-chord[1], chord[0]]) / np.linalg.norm(chord)
            ends = [base[1]] + ([np.asarray(info["ext"])] if info["ext"] is not None else [])
            info["lat_ok"] = tuple(all(ok(xy, e + side * LATERAL_MAX_M * n) for e in ends) for side in (1.0, -1.0))
    info["probes"] = [float(v) for v in probe_walls(occ, spec, xy, heading)]
    return info


def wrap_build_planner(build_planner, carrot_m):
    """build_planner for PlannerProcess that attaches ``cmd.residual_info`` to each command.
    It only reads the planner; the commands and their paths are unchanged."""
    def build(*args, **kw):
        planner, spec, env, prior = build_planner(*args, **kw)
        from pathplan.grid_search import line_cost
        from pathplan.online import _point_along
        from pathplan.planner_process import safe_carrot_path
        inner = planner.update

        def update(xy, yaw):
            cmd = inner(xy, yaw)
            try:
                cmd.residual_info = residual_info(planner, spec, xy, yaw, cmd, carrot_m, safe_carrot_path,
                                                  line_cost, _point_along)
            except Exception:                       # never let the residual break planning
                cmd.residual_info = None
            return cmd
        planner.update = update
        return planner, spec, env, prior
    return build


# ------------------------------------------------------------------------------ policy
class MLPPolicy:
    """tanh MLP, obs -> 3 numbers in [-1, 1]. Weights from a JSON-able dict (no torch)."""

    def __init__(self, params):
        if params.get("format") != RESIDUAL_FORMAT:
            raise ValueError("not a " + RESIDUAL_FORMAT + " policy")
        self.layers = [(np.asarray(w, float), np.asarray(b, float)) for w, b in params["layers"]]
        self.obs_dim = int(params["obs_dim"])
        if self.obs_dim != OBS_DIM or self.layers[0][0].shape[1] != OBS_DIM or self.layers[-1][0].shape[0] != 3:
            raise ValueError("policy shape does not match this runtime")

    def act(self, obs):
        h = np.asarray(obs, float)
        for w, b in self.layers:
            h = np.tanh(w @ h + b)
        return h

    @staticmethod
    def from_flat(theta, sizes):
        layers, k = [], 0
        for i, o in zip(sizes[:-1], sizes[1:]):
            w = np.asarray(theta[k:k + i * o]).reshape(o, i)
            k += i * o
            b = np.asarray(theta[k:k + o])
            k += o
            layers.append((w.tolist(), b.tolist()))
        return MLPPolicy({"format": RESIDUAL_FORMAT, "obs_dim": sizes[0], "layers": layers})


def action_to_residual(a):
    """[-1, 1]^3 -> (carrot scale, lateral m, yaw-rate scale); a = 0 is exactly the base."""
    a = np.clip(np.asarray(a, float), -1.0, 1.0)
    s = 1.0 + (a[0] * (CARROT_SCALE_MAX - 1.0) if a[0] >= 0 else a[0] * (1.0 - CARROT_SCALE_MIN))
    lat = float(a[1]) * LATERAL_MAX_M
    y = 1.0 + min(float(a[2]), 0.0) * (1.0 - YAW_SCALE_MIN)
    return float(s), lat, float(y)


def _wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def make_residual_mission(Mission, policy_params):
    """Subclass of Jolie's Mission that adds the residual at the carrot computation.
    ``policy_params`` None -> the residual is always zero (diagnostics only)."""
    policy = MLPPolicy(policy_params) if policy_params is not None else None

    class ResidualMission(Mission):
        def __init__(self, planner, cfg=None):
            super().__init__(planner, cfg)
            self.policy = policy
            self.residual = ZERO
            self.residual_stats = {"applied": 0, "zero": 0, "zero_reasons": {}}
            self._r_last_tick = None
            self._r_last_stall = -1e9
            self._r_now = None
            self._r_odom_t = None
            self._r_hist = deque(maxlen=12)       # (odometry arrival t, course xy)
            self._r_prev = np.zeros(3)

        # ---------------------------------------------------------------- control loop
        def tick(self, tel):
            now = tel.now
            if self._r_last_tick is not None and now - self._r_last_tick > STALL_GAP_S:
                self._r_last_stall = now
            self._r_last_tick, self._r_now, self._r_odom_t = now, now, tel.odom_t
            if (tel.odom_t is None or now - tel.odom_t > STALE_ODOM_S or now - self._r_last_stall < STALL_HOLDOFF_S
                    or self.state != "NAVIGATE"):
                self.residual = ZERO                 # stale inputs: the base cap
            out = super().tick(tel)
            if self.pose_course is not None and tel.odom_t is not None and (
                    not self._r_hist or tel.odom_t > self._r_hist[-1][0]):
                self._r_hist.append((tel.odom_t, np.asarray(self.pose_course[0][:2], float).copy()))
            return out

        def _limit_yaw(self, setpoint, tel):
            """Jolie's heading rate limiter with the cap scaled by the residual (1.0 = hers)."""
            position, yaw = setpoint
            now = tel.now
            if self._sp_yaw is None:
                from flyx.px4_link import yaw_from_q_ned
                self._sp_yaw, self._sp_yaw_t = yaw_from_q_ned(tel.q_ned_frd), now
            scale = self.residual[2] if self.state == "NAVIGATE" else 1.0
            step = self.cfg.yaw_rate_max * scale * max(0.0, min(now - self._sp_yaw_t, .2))
            err = float(np.arctan2(np.sin(yaw - self._sp_yaw), np.cos(yaw - self._sp_yaw)))
            new = self._sp_yaw + float(np.clip(err, -step, step))
            self._sp_yaw, self._sp_yaw_t = float(np.arctan2(np.sin(new), np.cos(new))), now
            return position, self._sp_yaw

        # ---------------------------------------------------------------- planning
        def plan(self):
            from pathplan.ros_node import target_from_command
            if not self.planning_allowed():
                return None
            xyz, yaw, _ = self.pose_course
            cmd = self.planner.update(xyz[:2], yaw)
            base_carrot = self._carrot_for(cmd, yaw)
            self.target = target_from_command(cmd, xyz[:2], yaw, base_carrot)
            self.target = self._residual_target(cmd, xyz, yaw, base_carrot, self.target)
            if cmd.mode == "turn":
                if self._turn_xy is None:
                    self._turn_xy = np.asarray(xyz[:2], float).copy()
                self.target = (self._turn_xy, self.target[1])
            else:
                self._turn_xy = None
            self.planner_status = cmd.mode
            return cmd

        def _zero(self, why):
            self.residual = ZERO
            st = self.residual_stats
            st["zero"] += 1
            st["zero_reasons"][why] = st["zero_reasons"].get(why, 0) + 1

        def residual_obs(self, cmd, xyz, yaw, base_carrot):
            """Observation from drone-side information only; None if stale or invalid."""
            if self.state != "NAVIGATE" or cmd.mode not in ("move", "turn"):
                return None, "not navigating"
            now = self._r_now
            if now is None or self._r_odom_t is None or now - self._r_odom_t > STALE_ODOM_S:
                return None, "odometry stale"
            if now - self._r_last_stall < STALL_HOLDOFF_S:
                return None, "control stall"
            info = getattr(cmd, "residual_info", None)
            if not info or "probes" not in info:
                return None, "no planner info"
            xy = np.asarray(xyz[:2], float)
            v = np.zeros(2)
            old = [h for h in self._r_hist if 0.1 <= self._r_odom_t - h[0] <= 0.6]
            if old:
                t0, p0 = old[-1]
                v = (self._r_hist[-1][1] - p0) / max(self._r_odom_t - t0, 1e-3)
            if cmd.mode == "move" and cmd.path is not None and len(cmd.path) >= 2:
                chord = np.asarray(cmd.path[1], float) - xy
            else:
                chord = np.array([math.cos(yaw), math.sin(yaw)]) * 1e-3
            cl = float(np.linalg.norm(chord))
            u = chord / cl if cl > 1e-9 else np.array([math.cos(yaw), math.sin(yaw)])
            n = np.array([-u[1], u[0]])
            herr = _wrap((cmd.yaw if cmd.yaw is not None else yaw) - yaw)
            carrot_m = self.cfg.carrot_m
            pr = np.asarray(info["probes"], float) / PROBE_RANGE_M
            obs = np.array([
                float(v @ u) / 0.6, float(v @ n) / 0.3, math.sin(herr), math.cos(herr),
                base_carrot / carrot_m, (cl if cmd.mode == "move" else 0.0) / carrot_m,
                1.0 if info.get("ext") is not None else 0.0, min(info["cp_dist"], 3.0) / 3.0,
                1.0 if cmd.mode == "turn" else 0.0, *pr,
                min(now - self._r_odom_t, STALE_ODOM_S) / STALE_ODOM_S,
                math.exp(-(now - self._r_last_stall) / 2.0), info["cp_index"] / 4.0, *self._r_prev])
            if obs.shape != (OBS_DIM,) or not np.isfinite(obs).all():
                return None, "invalid observation"
            return obs, ""

        def _residual_target(self, cmd, xyz, yaw, base_carrot, base_target):
            if self.policy is None:
                self._zero("no policy")
                return base_target
            obs, why = self.residual_obs(cmd, xyz, yaw, base_carrot)
            if obs is None:
                self._zero(why)
                return base_target
            a = self.policy.act(obs)
            if not np.isfinite(a).all():
                self._zero("non-finite action")
                return base_target
            s, lat, ys = action_to_residual(a)
            self._r_prev = np.clip(a, -1, 1)
            info = cmd.residual_info
            target = base_target
            if cmd.mode == "move" and cmd.path is not None and len(cmd.path) >= 2:
                from pathplan.online import _point_along
                xy = np.asarray(xyz[:2], float)
                end = np.asarray(cmd.path[1], float)
                chord = float(np.linalg.norm(end - xy))
                if chord > 1e-6:
                    d0 = min(base_carrot, chord)
                    if s <= 1.0:
                        p = xy + (end - xy) * (d0 * s / chord)
                    elif info.get("ext") is not None:
                        p = _point_along(np.array([xy, end, info["ext"]]), d0 + base_carrot * (s - 1.0))
                    else:
                        p, s = np.asarray(base_target[0], float), 1.0
                    u = (p - xy) / max(float(np.linalg.norm(p - xy)), 1e-9)
                    side_ok = info["lat_ok"][0 if lat > 0 else 1]
                    if lat != 0.0 and side_ok:
                        p = p + lat * np.array([-u[1], u[0]])
                    else:
                        lat = 0.0
                    target = (np.asarray(p, float), base_target[1])
            else:
                s, lat = 1.0, 0.0                      # turn in place: only the yaw cap
            self.residual = (s, lat, ys)
            self.residual_stats["applied"] += 1
            return target

    return ResidualMission
