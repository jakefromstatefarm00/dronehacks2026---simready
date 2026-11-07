"""Train a checkpoint-following policy with PPO (stable-baselines3). Laptop / desktop only.

    cd team_ws/src/team_mission
    pip install -r requirements-train.txt
    python -m team_mission.rl.train --out runs/v1                       # course + random paths
    python -m team_mission.rl.train --out runs/v2 --resume runs/v1/model.zip --steps 1000000
    python -m team_mission.rl.train --out runs/res1 --residual-scale 0.3  # learn a correction to the baseline

What it does
  1. Behaviour cloning (--bc-episodes, default 300): fly the hand-written CarrotPolicy in the
     randomised environment and teach the network to copy it. The network starts out already
     able to fly, so PPO only has to improve it (much faster and more reliable than from zero).
  2. PPO for --steps environment steps, in --n-envs parallel environments.
  3. Every --eval-every steps it evaluates on a fixed set of randomised course episodes and keeps
     the best model (by success rate, then time) as best_model.zip / policy.npz.
  4. Exports policy.npz (numpy weights, what the drone loads) and writes everything you need to
     reproduce the run into config.json.

Outputs in --out:  model.zip, best_model.zip, policy.npz (best), policy_last.npz, config.json,
                   progress.csv (PPO stats), eval.csv (success / time per evaluation).
"""
from __future__ import annotations

import argparse
import csv
import json
import os
import time

import numpy as np


def make_env_fn(a, seed):
    def _f():
        from team_mission.control import Limits
        from team_mission.rl.dynamics import DynamicsRanges
        from team_mission.rl.env import CheckpointEnv
        dyn = None
        if a.dynamics:
            import yaml
            dyn = DynamicsRanges.from_dict(yaml.safe_load(open(a.dynamics)))
        limits = Limits()
        if a.limits:
            import yaml
            limits = Limits.from_dict((yaml.safe_load(open(a.limits)) or {}).get('limits', {}))
        return CheckpointEnv(a.checkpoints, limits=limits, dynamics=dyn, randomize=not a.nominal,
                             random_path_prob=a.random_path_prob, mid_start_prob=a.mid_start_prob,
                             residual_scale=a.residual_scale, course_obj=a.course_obj, seed=seed)
    return _f


def behaviour_clone(model, env_fn, episodes: int, epochs: int = 30, seed: int = 0):
    """Copy the CarrotPolicy into the PPO actor (mean) with a plain MSE fit."""
    import torch
    from team_mission.control import CarrotPolicy
    env = env_fn()
    teacher = CarrotPolicy()
    X, Y = [], []
    for ep in range(episodes):
        obs, _ = env.reset(seed=seed + ep)
        while True:
            a = teacher.act(obs, env.context())
            X.append(obs)
            Y.append(a)
            # DAgger-lite: act with a little noise so the student also sees off-path states
            obs, _, te, tr, _ = env.step(np.clip(a + np.random.normal(0, 0.15, a.shape), -1, 1))
            if te or tr:
                break
    X = torch.as_tensor(np.array(X), dtype=torch.float32)
    Y = torch.as_tensor(np.array(Y), dtype=torch.float32)
    pol = model.policy
    params = list(pol.mlp_extractor.policy_net.parameters()) + list(pol.action_net.parameters())
    opt = torch.optim.Adam(params, lr=1e-3)
    n = len(X)
    for ep in range(epochs):
        perm = torch.randperm(n)
        tot = 0.0
        for k in range(0, n, 1024):
            idx = perm[k:k + 1024]
            mean = pol.action_net(pol.mlp_extractor.forward_actor(X[idx]))
            loss = ((mean - Y[idx]) ** 2).mean()
            opt.zero_grad()
            loss.backward()
            opt.step()
            tot += float(loss.detach()) * len(idx)
        if ep % 10 == 9 or ep == epochs - 1:
            print(f'[bc] epoch {ep + 1}/{epochs}  mse {tot / n:.4f}  ({n} samples)')
    with torch.no_grad():
        pol.log_std.fill_(-1.5)   # start PPO exploring gently around the copied behaviour


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    here = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
    ap.add_argument('--checkpoints', default=os.path.join(here, 'config', 'checkpoints', 'course_v3.yaml'))
    ap.add_argument('--out', required=True)
    ap.add_argument('--steps', type=float, default=1.5e6)
    ap.add_argument('--n-envs', type=int, default=8)
    ap.add_argument('--seed', type=int, default=0)
    ap.add_argument('--resume', default='')
    ap.add_argument('--bc-episodes', type=int, default=300, help='0 = no behaviour cloning')
    ap.add_argument('--residual-scale', type=float, default=0.0)
    ap.add_argument('--random-path-prob', type=float, default=0.5)
    ap.add_argument('--mid-start-prob', type=float, default=0.5)
    ap.add_argument('--dynamics', default='', help='YAML of DynamicsRanges (e.g. from fit_dynamics)')
    ap.add_argument('--limits', default='', help='mission.yaml-style file with a limits: section')
    ap.add_argument('--course-obj', default='', help='also crash on the real course walls (needs the repo drl/)')
    ap.add_argument('--nominal', action='store_true', help='no randomisation (debug only)')
    ap.add_argument('--eval-every', type=float, default=100_000)
    ap.add_argument('--eval-episodes', type=int, default=40)
    ap.add_argument('--lr', type=float, default=1e-4)
    a = ap.parse_args()

    import torch
    from stable_baselines3 import PPO
    from stable_baselines3.common.callbacks import BaseCallback
    from stable_baselines3.common.logger import configure
    from stable_baselines3.common.vec_env import DummyVecEnv, SubprocVecEnv

    from team_mission.rl.evaluate import evaluate
    from team_mission.rl.export import export_sb3

    os.makedirs(a.out, exist_ok=True)
    cfg = vars(a).copy()
    cfg['started'] = time.strftime('%Y-%m-%d %H:%M:%S')
    json.dump(cfg, open(os.path.join(a.out, 'config.json'), 'w'), indent=2)
    fns = [make_env_fn(a, a.seed * 1000 + k) for k in range(a.n_envs)]
    venv = SubprocVecEnv(fns) if a.n_envs > 1 and os.cpu_count() and os.cpu_count() > 2 else DummyVecEnv(fns)

    if a.resume:
        model = PPO.load(a.resume, env=venv, device='cpu')
    else:
        model = PPO('MlpPolicy', venv, learning_rate=a.lr, n_steps=512, batch_size=1024, n_epochs=10,
                    gamma=0.99, gae_lambda=0.95, clip_range=0.2, ent_coef=0.0, seed=a.seed,
                    policy_kwargs=dict(net_arch=dict(pi=[64, 64], vf=[128, 128]), activation_fn=torch.nn.Tanh),
                    device='cpu', verbose=0)
        if a.bc_episodes > 0 and a.residual_scale <= 0:
            behaviour_clone(model, fns[0], a.bc_episodes, seed=a.seed)
        elif a.residual_scale > 0:
            with torch.no_grad():   # residual: start exactly at the baseline, explore gently
                model.policy.log_std.fill_(-1.0)
    model.set_logger(configure(a.out, ['csv']))

    meta = dict(name=os.path.basename(os.path.normpath(a.out)), residual_scale=a.residual_scale,
                checkpoints=os.path.basename(a.checkpoints), dynamics=a.dynamics or 'default ranges')
    eval_csv = os.path.join(a.out, 'eval.csv')
    with open(eval_csv, 'w', newline='') as fh:
        csv.writer(fh).writerow(['steps', 'success', 'mean_time_s', 'p95_lateral_m', 'reasons'])
    best = {'key': (-1.0, 0.0)}

    def run_eval(steps):
        tmp = os.path.join(a.out, 'policy_last.npz')
        export_sb3(model, tmp, **meta, steps=int(steps))
        s = evaluate(tmp, a.checkpoints, episodes=a.eval_episodes, seed=12345,
                     course_obj=a.course_obj or None, quiet=True)
        with open(eval_csv, 'a', newline='') as fh:
            csv.writer(fh).writerow([int(steps), s['success_rate'], s['mean_time_s'], s['p95_max_lateral_m'],
                                     json.dumps(s['reasons'])])
        key = (s['success_rate'], -(s['mean_time_s'] or 1e9))
        tag = ''
        if key > best['key']:
            best['key'] = key
            model.save(os.path.join(a.out, 'best_model.zip'))
            export_sb3(model, os.path.join(a.out, 'policy.npz'), **meta, steps=int(steps))
            tag = '  <- best'
        print(f'[eval] {int(steps):>9d} steps  success {s["success_rate"] * 100:5.1f}%  '
              f'time {s["mean_time_s"] or 0:5.1f} s  p95 lateral {s["p95_max_lateral_m"]:.3f} m{tag}', flush=True)

    class EvalCb(BaseCallback):
        def __init__(self):
            super().__init__()
            self.next = a.eval_every

        def _on_step(self):
            if self.num_timesteps >= self.next:
                self.next += a.eval_every
                run_eval(self.num_timesteps)
            return True

    run_eval(0)
    t0 = time.time()
    model.learn(total_timesteps=int(a.steps), callback=EvalCb(), reset_num_timesteps=not a.resume)
    model.save(os.path.join(a.out, 'model.zip'))
    run_eval(model.num_timesteps)
    print(f'done in {(time.time() - t0) / 60:.1f} min. Best: {a.out}/policy.npz  (copy it to policies/)')


if __name__ == '__main__':
    main()
