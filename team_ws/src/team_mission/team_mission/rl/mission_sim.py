"""Fly the WHOLE mission in the training simulator: power on -> arm -> takeoff -> checkpoints ->
descend -> touchdown -> disarm. No ROS needed.

This runs the drone node's exact state machine (team_mission/mission.py) against simple stand-ins
for the parts that are not ours:
  * the pilot           arms at t=2 s in Altitude, switches to Position at t=4 s
  * flight_interface    Offboard only after "armed, then Position" with a live team setpoint;
                        pose setpoints floored at 0.3 m above the boot origin; twist capped
  * PX4                 the randomised position/velocity model from dynamics.py, the ground,
                        a land detector, auto-disarm COM_DISARM_LAND s after landing, and the
                        10 s "disarm if it never took off" rule (COM_DISARM_PRFLT)
  * cuVSLAM             position/velocity/yaw estimate with noise and drift

    python -m team_mission.rl.mission_sim                                   # test_line, carrot
    python -m team_mission.rl.mission_sim --checkpoints course_v3.yaml --policy policies/v0_ppo.npz \
        --launch -2.515 0.914 0 90 --log /tmp/sim_flight.csv --plot /tmp/sim_flight.png
    python -m team_mission.rl.mission_sim --runs 50 --checkpoints course_v3.yaml --launch -2.515 0.914 0 90

The CSV it writes has the same columns as a real flight log, so every analysis tool works on both.
"""
from __future__ import annotations

import argparse
import collections
import math
import os

import numpy as np

from team_mission.control import Limits, load_policy
from team_mission.flight_log import FlightLogger
from team_mission.geometry import CourseFrame
from team_mission.mission import DORMANT, Inputs, Mission, MissionParams
from team_mission.path import CheckpointSpec
from team_mission.rl.dynamics import DroneSim, DynamicsRanges

HERE = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))


def find(name, sub):
    if os.path.exists(name):
        return name
    c = os.path.join(HERE, sub, name)
    return c if os.path.exists(c) else name


def run_once(spec, policy, cf: CourseFrame, seed=0, randomize=True, log_path='', params=None,
             max_s=400.0, verbose=False, course=None):
    rng = np.random.default_rng(seed)
    sim = DroneSim()
    sim.sample(DynamicsRanges() if randomize else DynamicsRanges.nominal(), rng)
    sim.reset(np.zeros(3), 0.0, rng)            # boot: odom origin = base_link, x = nose
    msgs = []
    mission = Mission(spec, policy, Limits(), params or MissionParams(), cf,
                      logger=(lambda lv, m: msgs.append((round(sim.t, 2), lv, m))))
    logger = FlightLogger(os.path.dirname(log_path) or '.') if log_path else None
    armed = offboard = False
    position_switch = False
    last_sp_t, mode, sp = -1e9, 'hold', None
    landed_since, arm_t, took_off = None, None, False
    out_kind = collections.Counter()
    truth = []
    result = 'timeout'
    path = None
    while sim.t < max_s:
        t = sim.t
        # ---- the pilot
        if not armed and arm_t is None and t >= 2.0:
            armed, arm_t = True, t
        if armed and not position_switch and t >= 4.0:
            position_switch = True
            if t - last_sp_t < 1.0:           # flight_interface's auto offboard checks
                offboard = True
        # ---- estimate + inputs
        p_est, v_est, yaw_est = sim.estimate()
        landed = sim.p[2] <= 0.02 and abs(sim.v[2]) < 0.1
        s = Inputs(now=t, armed=armed, offboard_active=offboard, landed=landed,
                   nav_state_name='OFFBOARD' if offboard else ('POSCTL' if position_switch else 'ALTCTL'),
                   pos=p_est, vel=v_est, yaw=yaw_est, odom_age=0.03, vo_state=1)
        out = mission.step(s)
        if mission.path is not None:
            path = mission.path
        out_kind[out.kind] += 1
        if logger is not None:
            if mission.phase != DORMANT and not logger.open:
                logger.start(t, {'simulated': True, 'seed': seed})
                logger.path = log_path
            if logger.open:
                logger.row(t, s, out)
        # ---- flight_interface
        if out.kind == 'pose':
            sp = np.array(out.position, float)
            sp[2] = max(sp[2], 0.3)               # min_altitude floor above the boot origin
            mode, sp_yaw, last_sp_t = 'pose', out.yaw, t
        elif out.kind == 'twist':
            v = np.asarray(out.velocity, float)
            n = np.linalg.norm(v)
            mode, sp_v, last_sp_t = 'twist', (v if n <= 1.5 else v * 1.5 / n), t
        # ---- PX4
        if armed and offboard:
            if mode == 'twist' and t - last_sp_t < 1.0:
                sim.step(None, None, velocity=sp_v, ground_z=0.0)
            elif sp is not None:
                sim.step(sp, sp_yaw if sp_yaw is not None else sim.yaw, ground_z=0.0)
            else:
                sim.step(sim.p.copy(), sim.yaw, ground_z=0.0)
        else:
            sim.t += sim.dt                         # on the ground, motors idle
        if sim.p[2] > 0.15:
            took_off = True
        if course is not None:
            pc = cf.odom_to_course(sim.p)
            if pc[2] > 0.3 and course.collides(pc[None, :], 0.25, 0.1)[0]:
                result = 'HIT THE COURSE'
                break
        truth.append(np.r_[sim.t, sim.p, sim.yaw])
        # PX4 rules
        if armed and not took_off and arm_t is not None and t - arm_t > 10.0:
            armed, offboard = False, False
            result = 'disarmed: never took off (COM_DISARM_PRFLT)'
            break
        if armed and took_off and landed:
            landed_since = landed_since if landed_since is not None else t
            if t - landed_since > 1.0:              # COM_DISARM_LAND
                armed, offboard = False, False
                mission.step(Inputs(now=sim.t, armed=False))
                result = 'landed and disarmed'
                break
        else:
            landed_since = None
        if mission.phase == 'ABORTED':
            result = 'ABORTED: ' + mission.events[-1][3]
            break
    if logger is not None:
        logger.close()
    final_c = cf.odom_to_course(sim.p)
    target_c = cf.odom_to_course(path.points[-1]) if path is not None else None
    landing_err = float(np.linalg.norm((final_c - target_c)[:2])) if target_c is not None else None
    if verbose:
        for m in msgs:
            print(f'  t={m[0]:6.2f}s  {m[1]:5s} {m[2]}')
    return dict(result=result, time_s=round(sim.t, 2), landing_error_m=landing_err,
                truth=np.array(truth), path=path, messages=msgs)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--checkpoints', default='test_line.yaml')
    ap.add_argument('--policy', default='carrot')
    ap.add_argument('--launch', nargs=4, type=float, default=None, metavar=('X', 'Y', 'Z', 'YAW_DEG'),
                    help='launch pose in course coordinates (needed for frame: course files)')
    ap.add_argument('--runs', type=int, default=1)
    ap.add_argument('--seed', type=int, default=0)
    ap.add_argument('--nominal', action='store_true')
    ap.add_argument('--log', default='')
    ap.add_argument('--plot', default='')
    ap.add_argument('--course-obj', default='', help='also check the real course walls (needs repo drl/)')
    a = ap.parse_args()
    spec = CheckpointSpec.load(find(a.checkpoints, 'config/checkpoints'))
    policy = load_policy(a.policy if a.policy in ('carrot', 'baseline') else find(a.policy, 'policies'))
    if a.launch is None and spec.frame == 'course':
        import yaml
        st = (yaml.safe_load(open(find(a.checkpoints, 'config/checkpoints'))) or {}).get('train_start')
        a.launch = [*st['xyz'], st.get('yaw_deg', 0.0)] if st else [0, 0, 0, 0]
    cf = CourseFrame(*(a.launch[:3] if a.launch else (0, 0, 0)), math.radians(a.launch[3]) if a.launch else 0.0)
    course = None
    if a.course_obj:
        from team_mission.rl.course_collision import load_course
        course = load_course(a.course_obj)
    results = collections.Counter()
    times = []
    for k in range(a.runs):
        r = run_once(spec, policy, cf, seed=a.seed + k, randomize=not a.nominal,
                     log_path=a.log if k == 0 else '', verbose=(a.runs == 1), course=course)
        results[r['result']] += 1
        if r['result'] == 'landed and disarmed':
            times.append(r['time_s'])
        if a.runs == 1:
            print(f"result: {r['result']} after {r['time_s']} s; landed {r['landing_error_m']:.2f} m from the "
                  f"last checkpoint" if r['landing_error_m'] is not None else f"result: {r['result']}")
            if a.log:
                print(f'flight log: {a.log}')
            if a.plot:
                plot(r, cf, a.plot)
    if a.runs > 1:
        print(dict(results))
        if times:
            print(f'arm -> disarm: mean {np.mean(times):.1f} s, max {np.max(times):.1f} s')


def plot(r, cf, out):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    tr = r['truth']
    P = np.array([cf.odom_to_course(p) for p in r['path'].points])
    T = np.array([cf.odom_to_course(p) for p in tr[:, 1:4]])
    fig, (ax, az) = plt.subplots(1, 2, figsize=(13, 6), gridspec_kw={'width_ratios': [1.2, 1]})
    ax.plot(P[:, 0], P[:, 1], 'k--', lw=1)
    for k, p in enumerate(P[1:], 1):
        ax.annotate(r['path'].names[k], p[:2], fontsize=7)
    ax.plot(T[:, 0], T[:, 1], lw=1.5)
    ax.set_aspect('equal')
    ax.set_title('top view (course frame)')
    az.plot(tr[:, 0], tr[:, 3])
    az.set_xlabel('time (s)')
    az.set_ylabel('height above start (m)')
    az.set_title(r['result'])
    fig.savefig(out, dpi=120, bbox_inches='tight')
    print(f'wrote {out}')


if __name__ == '__main__':
    main()
