"""Export a trained stable-baselines3 PPO model to policy.npz (numpy weights the drone can run).

    python -m team_mission.rl.export runs/v1/best_model.zip policies/v1.npz

The drone container has numpy but not torch, so the node runs the network itself
(team_mission.control.MLPPolicy). Only the deterministic action (the mean) is exported.
"""
from __future__ import annotations

import argparse
import json
import time

import numpy as np

from team_mission.control import ACT_DIM, OBS_DIM, OBS_VERSION, Limits


def export_sb3(model, out_path: str, limits: Limits = None, **meta_extra):
    pol = model.policy
    layers = [m for m in pol.mlp_extractor.policy_net if hasattr(m, 'weight')] + [pol.action_net]
    acts = [type(m).__name__ for m in pol.mlp_extractor.policy_net if not hasattr(m, 'weight')]
    if any(x != 'Tanh' for x in acts):
        raise ValueError(f'only tanh hidden layers are supported, got {acts}')
    arrays = {}
    for k, m in enumerate(layers):
        arrays[f'W{k}'] = m.weight.detach().cpu().numpy().astype(np.float32)
        arrays[f'b{k}'] = m.bias.detach().cpu().numpy().astype(np.float32)
    if arrays['W0'].shape[1] != OBS_DIM or arrays[f'W{len(layers) - 1}'].shape[0] != ACT_DIM:
        raise ValueError('network shape does not match OBS_DIM/ACT_DIM')
    meta = dict(obs_version=OBS_VERSION, obs_dim=OBS_DIM, act_dim=ACT_DIM, n_layers=len(layers),
                activation='tanh', exported=time.strftime('%Y-%m-%d %H:%M:%S'),
                limits=(limits or Limits()).__dict__)
    meta.update(meta_extra)
    np.savez(out_path, meta=np.array(json.dumps(meta)), **arrays)
    return out_path


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('model')
    ap.add_argument('out')
    ap.add_argument('--residual-scale', type=float, default=0.0)
    a = ap.parse_args()
    from stable_baselines3 import PPO
    model = PPO.load(a.model, device='cpu')
    export_sb3(model, a.out, residual_scale=a.residual_scale, name=a.out)
    # self-check: the numpy network must equal torch's deterministic action
    import torch
    from team_mission.control import MLPPolicy
    p = MLPPolicy(a.out)
    x = np.random.default_rng(0).uniform(-1, 1, (16, OBS_DIM)).astype(np.float32)
    with torch.no_grad():
        ref = model.policy._predict(torch.as_tensor(x), deterministic=True).numpy()
    mine = np.array([np.clip(p.network(o), -1, 1) for o in x])
    err = float(np.abs(np.clip(ref, -1, 1) - mine).max())
    print(f'exported {a.out}; max difference vs torch {err:.2e}')
    if err > 1e-4:
        raise SystemExit('export mismatch!')


if __name__ == '__main__':
    main()
