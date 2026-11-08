"""Offline kinematic check of a single-file program's planner + carrot + yaw logic (no sensing, no ROS).

python kinsim.py <program.py> [--yaw-rate 0.30] [--plot out.csv]
Point mass: v = k (target - p) (k=0.85/s, <= 1 m/s), heading follows the rate-limited command with 0.3 s lag.
"""
import importlib.util
import sys

import numpy as np

import corner_analysis as ca

prog = sys.argv[1]
yaw_rate = float(sys.argv[sys.argv.index("--yaw-rate") + 1]) if "--yaw-rate" in sys.argv else .30
spec = importlib.util.spec_from_file_location("prog", prog)
m = importlib.util.module_from_spec(spec)
spec.loader.exec_module(m)
planner, gspec, env, prior = m._build_planner_embedded()
import pathplan.planner_process as pp  # noqa: E402
from pathplan.ros_node import target_from_command  # noqa: E402
import flyx.mission as fm  # noqa: E402

planner.cfg.turn_in_place_deg = pp.TURN_IN_PLACE_DEG
mission = fm.Mission(planner, fm.MissionConfig())
has_rule = hasattr(pp, "corner_rule")
for kv in [a for a in sys.argv if a.startswith('PP.')]:
    k, v = kv[3:].split('=')
    setattr(pp, k, eval(v))
for kv in [a for a in sys.argv if a.startswith('MC.')]:
    k, v = kv[3:].split('=')
    setattr(mission.cfg, k, eval(v))

dt, k_pos = .05, .85
p = np.array([-2.515, .914])
yaw = np.pi / 2
sp_yaw = yaw
target = (p.copy(), yaw)
turn_xy = None
log = []
reasons = {}
t = 0.0
cmd = None
rng = np.random.default_rng(int(sys.argv[sys.argv.index('--seed') + 1]) if '--seed' in sys.argv else 0)
while t < 200:
    if int(round(t / dt)) % 3 == 0:                      # planner ~6.7 Hz
        p_meas = p + rng.normal(0, .003, 2)              # estimate noise (a noiseless point parks on a route vertex)
        cmd = planner.update(p_meas, yaw)
        if cmd.mode in ("done", "failed"):
            break
        if cmd.mode == "move" and cmd.path is not None and len(cmd.path) >= 2:
            if has_rule:
                pp.corner_rule(planner, cmd, p_meas, yaw)
            cmd.path = pp.safe_carrot_path(planner.belief.cost, gspec, p_meas, cmd.path, mission.cfg.carrot_m)
        reasons[cmd.reason or cmd.mode] = reasons.get(cmd.reason or cmd.mode, 0) + 1
        if "--debug" in sys.argv and planner.index <= 1 and p[1] > 2.3 and p[0] < -2.0:
            print(f"DBG t={t:.2f} p={np.round(p, 2)} yaw={np.degrees(yaw):.0f} mode={cmd.mode} reason={cmd.reason} "
                  f"cmdyaw={np.degrees(cmd.yaw) if cmd.yaw is not None else None} sign={getattr(planner, '_corner_sign', None)} "
                  f"poly={np.round(planner.poly[:4], 2).tolist() if planner.poly is not None else None}")
        target = target_from_command(cmd, p, yaw, mission._carrot_for(cmd, yaw))
        if cmd.mode == "turn":
            turn_xy = p.copy() if turn_xy is None else turn_xy
            target = (turn_xy, target[1])
        else:
            turn_xy = None
    err = np.arctan2(np.sin(target[1] - sp_yaw), np.cos(target[1] - sp_yaw))
    sp_yaw += float(np.clip(err, -yaw_rate * dt, yaw_rate * dt))
    v = k_pos * (np.asarray(target[0]) - p)
    sp = np.linalg.norm(v)
    if sp > 1.0:
        v *= 1.0 / sp
    p = p + v * dt
    yaw_prev = yaw
    yaw += np.arctan2(np.sin(sp_yaw - yaw), np.cos(sp_yaw - yaw)) * dt / .3
    log.append((t, p[0], p[1], yaw, np.linalg.norm(v), (yaw - yaw_prev) / dt, planner.index))
    t += dt

L = np.array(log)
print(f"{prog.split('/')[-1]}: end {cmd.mode if cmd else None} after {t:.1f} s, checkpoints {planner.index}, reasons {reasons}")
# first corner: from lift (t=0) until checkpoint index reaches 2 (Gate A)
fc = L[L[:, 6] < 2]
looks = np.array([ca.ray(np.array([r[1], r[2], .9]), r[3])[0] for r in fc[::4]])
turning = np.abs(fc[::4, 5]) > .1
P3 = np.c_[L[:, 1], L[:, 2], np.full(len(L), .9)]
clr = ca.free_radius(P3[::4])
print(f"  to Gate A: {fc[-1, 0]:.1f} s; while turning there: {turning.sum() * .2:.1f} s, look<1.0 m for "
      f"{(turning & (looks < 1.0)).sum() * .2:.1f} s, min look {looks[turning].min() if turning.any() else np.nan:.2f} m, "
      f"mean look {looks[turning].mean() if turning.any() else np.nan:.2f} m; mean speed x |yaw rate| while turning "
      f"{(fc[::4, 4] * np.abs(fc[::4, 5]))[turning].mean():.3f}; min clearance whole run {clr.min():.3f} m")
# first corner only: from y > 1.5 until 3 s after checkpoint 1 is cleared
i0 = int(np.argmax(L[:, 2] > 1.5))
i1 = int(np.argmax(L[:, 6] >= 1)) + int(3 / dt)
W = L[i0:i1:2]
lk = np.array([ca.ray(np.array([r[1], r[2], .9]), r[3])[0] for r in W])
tr = np.abs(W[:, 5]) > .1
print(f"  FIRST CORNER: {W[-1, 0] - W[0, 0]:.1f} s; turning {tr.sum() * 2 * dt:.1f} s; look<1.0 m while turning "
      f"{(tr & (lk < 1.0)).sum() * 2 * dt:.1f} s; min look while turning {lk[tr].min():.2f} m; mean speed while "
      f"turning {W[tr, 4].mean():.2f} m/s; looks: {np.round(lk[::5], 2).tolist()}")
if "--plot" in sys.argv:
    np.savetxt(sys.argv[sys.argv.index("--plot") + 1], L, delimiter=",", header="t,x,y,yaw,speed,yawrate,cp")
