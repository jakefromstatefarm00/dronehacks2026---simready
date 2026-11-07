"""Measure the real (or Gazebo / flyx) drone from a flight log, to make training match it.

    python -m team_mission.rl.fit_dynamics flight_logs/flight_20261107_141502.csv [more.csv ...] \
        --out config/dynamics_measured.yaml

It looks at the FLY and TAKEOFF parts of each log, where we know the carrot we sent and the
position/velocity cuVSLAM reported, and finds the model parameters (team_mission/rl/dynamics.py)
that best predict the next velocity:

    v_ref = Kp * (carrot - p)        (z uses Kp_z)
    v[k+1] = v[k] + (1 - exp(-dt/tau)) * (v_ref[k - delay] - v[k])

It grid-searches delay (control steps) and tau, and fits Kp by least squares, then prints the fit
quality and writes a dynamics YAML whose ranges are centred on what it measured (+-25 %, delay
+-1 step). Train with it:  python -m team_mission.rl.train --dynamics config/dynamics_measured.yaml ...

It also reports the noise of the velocity estimate (residual after the fit) and how fast the
yaw follows the yaw target, so you can see whether the default ranges were too optimistic.
"""
from __future__ import annotations

import argparse
import csv
import math

import numpy as np


def load(path):
    rows = list(csv.DictReader(open(path)))
    keep = [r for r in rows if r['phase'] in ('FLY', 'TAKEOFF') and r['cmd_kind'] == 'pose' and r['px'] != '']
    if len(keep) < 50:
        raise SystemExit(f'{path}: only {len(keep)} usable rows (need FLY/TAKEOFF with pose setpoints)')
    f = lambda k: np.array([float(r[k]) if r[k] != '' else np.nan for r in keep])  # noqa: E731
    return dict(t=f('t'), p=np.c_[f('px'), f('py'), f('pz')], v=np.c_[f('vx'), f('vy'), f('vz')],
                c=np.c_[f('cmd_x'), f('cmd_y'), f('cmd_z')], yaw=f('yaw'), cyaw=f('cmd_yaw'))


def fit(logs, axis_sets=((0, 1), (2,))):
    best = None
    for delay in range(0, 6):
        for tau in np.arange(0.1, 1.01, 0.05):
            res, kps = 0.0, []
            ok = True
            for axes in axis_sets:
                X, Y = [], []
                for L in logs:
                    dt = np.median(np.diff(L['t']))
                    alpha = 1 - math.exp(-dt / tau)
                    e = L['c'] - L['p']
                    for k in range(delay, len(L['t']) - 1):
                        if L['t'][k + 1] - L['t'][k] > 3 * dt:
                            continue
                        for ax in axes:
                            # v[k+1] - (1-alpha) v[k] = alpha * Kp * e[k-delay]
                            Y.append(L['v'][k + 1, ax] - (1 - alpha) * L['v'][k, ax])
                            X.append(alpha * e[k - delay, ax])
                X, Y = np.array(X), np.array(Y)
                m = np.isfinite(X) & np.isfinite(Y)
                if m.sum() < 30 or np.sum(X[m] ** 2) < 1e-9:
                    ok = False
                    break
                kp = float(np.sum(X[m] * Y[m]) / np.sum(X[m] ** 2))
                kps.append(kp)
                res += float(np.mean((Y[m] - kp * X[m]) ** 2))
            if ok and (best is None or res < best[0]):
                best = (res, delay, float(tau), kps)
    return best


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('logs', nargs='+')
    ap.add_argument('--out', default='')
    a = ap.parse_args()
    logs = [load(p) for p in a.logs]
    res, delay, tau, (kp_xy, kp_z) = fit(logs)
    dt = float(np.median(np.diff(logs[0]['t'])))
    vel_noise = math.sqrt(res)
    # yaw: how many steps until the yaw closes half the gap
    yaw_gain = []
    for L in logs:
        e = np.array([(c - y + math.pi) % (2 * math.pi) - math.pi for c, y in zip(L['cyaw'], L['yaw'])])
        dy = np.array([(b - c + math.pi) % (2 * math.pi) - math.pi for b, c in zip(L['yaw'][1:], L['yaw'][:-1])]) / dt
        m = np.isfinite(e[:-1]) & np.isfinite(dy) & (np.abs(e[:-1]) > 0.05)
        if m.sum() > 20:
            yaw_gain.append(float(np.sum(e[:-1][m] * dy[m]) / np.sum(e[:-1][m] ** 2)))
    print(f'control period      {dt * 1000:.0f} ms')
    print(f'delay               {delay} steps ({delay * dt * 1000:.0f} ms)')
    print(f'tau (velocity lag)  {tau:.2f} s')
    print(f'Kp xy / z           {kp_xy:.2f} / {kp_z:.2f} 1/s   (with lead_time 1.0 s, Kp*lead = speed gain)')
    print(f'velocity residual   {vel_noise:.3f} m/s per step (noise + unmodelled)')
    if yaw_gain:
        print(f'yaw gain            {np.median(yaw_gain):.2f} 1/s')
    if a.out:
        import yaml
        lo, hi = 0.75, 1.25
        d = dict(kp_xy=[kp_xy * lo, kp_xy * hi], kp_z=[kp_z * lo, kp_z * hi], tau=[tau * lo, tau * hi],
                 delay_steps=[max(0, delay - 1), delay + 1],
                 vel_noise=[0.5 * vel_noise * math.sqrt(dt), 1.5 * vel_noise * math.sqrt(dt) + 0.01])
        if yaw_gain:
            k = float(np.median(yaw_gain))
            d['k_yaw'] = [k * lo, k * hi]
        d = {k: [round(float(x), 4) if not isinstance(x, int) else x for x in v] for k, v in d.items()}
        with open(a.out, 'w') as fh:
            fh.write('# measured by team_mission.rl.fit_dynamics from:\n')
            for p in a.logs:
                fh.write(f'#   {p}\n')
            fh.write('# ranges not listed keep the defaults in team_mission/rl/dynamics.py\n')
            yaml.safe_dump(d, fh, default_flow_style=None)
        print(f'wrote {a.out}')


if __name__ == '__main__':
    main()
