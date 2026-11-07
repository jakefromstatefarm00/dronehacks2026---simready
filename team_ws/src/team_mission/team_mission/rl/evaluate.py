"""Evaluate a policy in the training simulator: success rate, lap time, how close to the walls.

    python -m team_mission.rl.evaluate carrot                       # the hand-written baseline
    python -m team_mission.rl.evaluate policies/v0_ppo.npz --episodes 100 --plot eval.png
    python -m team_mission.rl.evaluate policies/v0_ppo.npz --nominal  # perfect drone, no noise
    python -m team_mission.rl.evaluate policies/v0_ppo.npz --checkpoints config/checkpoints/test_square.yaml

Always compare against `carrot` on the SAME --seed: the episodes (dynamics, noise, starting
error) are then identical, so the difference is the policy.
"""
from __future__ import annotations

import argparse
import collections
import json
import os

import numpy as np


def evaluate(policy_spec: str, checkpoints: str, episodes: int = 50, seed: int = 0, nominal: bool = False,
             random_path_prob: float = 0.0, course_obj=None, dynamics=None, quiet: bool = False,
             keep_tracks: int = 0, **_):
    from team_mission.control import load_policy
    from team_mission.rl.env import CheckpointEnv
    policy = load_policy(policy_spec)
    env = CheckpointEnv(checkpoints, randomize=not nominal, random_path_prob=random_path_prob,
                        mid_start_prob=0.0, jitter_m=0.0 if nominal else 0.10, dynamics=dynamics,
                        course_obj=course_obj, seed=seed)
    infos, tracks, fail_at = [], [], collections.Counter()
    for ep in range(episodes):
        obs, _ = env.reset(seed=seed + ep)
        tr = [] if ep < keep_tracks else None
        while True:
            a = policy.act(obs, env.context())
            obs, _, te, tc, info = env.step(a)
            if tr is not None:
                tr.append(np.r_[env.sim.t, env.sim.p, env.sim.v, env.sim.yaw, env.i])
            if te or tc:
                break
        infos.append(info)
        if tr is not None:
            tracks.append((np.array(tr), env.path))
        if not info['success']:
            fail_at[env.path.names[env.i]] += 1
    ok = [i for i in infos if i['success']]
    lat = np.array([i['max_lateral_m'] for i in infos])
    s = dict(policy=str(policy_spec), episodes=episodes, success_rate=len(ok) / episodes,
             mean_time_s=float(np.mean([i['time_s'] for i in ok])) if ok else None,
             p95_max_lateral_m=float(np.percentile(lat, 95)), mean_max_lateral_m=float(lat.mean()),
             reasons=dict(collections.Counter(i['reason'] for i in infos)), failed_heading_for=dict(fail_at))
    if not quiet:
        print(json.dumps(s, indent=2))
    if keep_tracks:
        s['tracks'] = tracks
    return s


def plot_tracks(tracks, out_png: str, title: str):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig, ax = plt.subplots(figsize=(8, 8))
    path = tracks[0][1]
    ax.plot(path.points[:, 0], path.points[:, 1], 'k--', lw=1, label='checkpoint path')
    for k, p in enumerate(path.points[1:], 1):
        ax.add_patch(plt.Circle(p[:2], path.radius[k], fill=False, color='k', lw=0.5))
        ax.annotate(path.names[k], p[:2], fontsize=7, xytext=(3, 3), textcoords='offset points')
    for tr, _ in tracks:
        sc = ax.scatter(tr[:, 1], tr[:, 2], c=np.linalg.norm(tr[:, 4:6], axis=1), s=2, cmap='viridis', vmin=0, vmax=0.5)
    fig.colorbar(sc, ax=ax, label='horizontal speed (m/s)')
    ax.set_aspect('equal')
    ax.set_title(title)
    ax.legend(loc='lower left')
    fig.savefig(out_png, dpi=130, bbox_inches='tight')
    print(f'wrote {out_png}')


def main():
    here = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
    ap = argparse.ArgumentParser()
    ap.add_argument('policy', help="'carrot' or a .npz")
    ap.add_argument('--checkpoints', default=os.path.join(here, 'config', 'checkpoints', 'course_v3.yaml'))
    ap.add_argument('--episodes', type=int, default=50)
    ap.add_argument('--seed', type=int, default=12345)
    ap.add_argument('--nominal', action='store_true')
    ap.add_argument('--random-path-prob', type=float, default=0.0)
    ap.add_argument('--course-obj', default='')
    ap.add_argument('--dynamics', default='')
    ap.add_argument('--plot', default='')
    a = ap.parse_args()
    dyn = None
    if a.dynamics:
        import yaml
        from team_mission.rl.dynamics import DynamicsRanges
        dyn = DynamicsRanges.from_dict(yaml.safe_load(open(a.dynamics)))
    s = evaluate(a.policy, a.checkpoints, a.episodes, a.seed, a.nominal, a.random_path_prob,
                 a.course_obj or None, dyn, keep_tracks=10 if a.plot else 0)
    if a.plot:
        plot_tracks(s['tracks'], a.plot, f"{os.path.basename(a.policy)}: {s['success_rate'] * 100:.0f}% success")


if __name__ == '__main__':
    main()
