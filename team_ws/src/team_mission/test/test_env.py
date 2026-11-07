"""Training environment. Skipped where gymnasium is not installed (e.g. on the drone)."""
import os

import numpy as np
import pytest

gym = pytest.importorskip('gymnasium')

from team_mission.control import ACT_DIM, OBS_DIM, CarrotPolicy  # noqa: E402
from team_mission.rl.env import CheckpointEnv  # noqa: E402

PKG = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
COURSE = os.path.join(PKG, 'config', 'checkpoints', 'course_v3.yaml')


def test_spaces_and_step():
    env = CheckpointEnv(COURSE, seed=0)
    obs, _ = env.reset()
    assert obs.shape == (OBS_DIM,) and env.action_space.shape == (ACT_DIM,)
    for _ in range(50):
        obs, r, te, tr, info = env.step(env.action_space.sample())
        assert np.all(np.isfinite(obs)) and np.isfinite(r)
        if te or tr:
            break


def test_baseline_finishes_nominal_course():
    env = CheckpointEnv(COURSE, randomize=False, random_path_prob=0.0, mid_start_prob=0.0, jitter_m=0.0, seed=0)
    obs, _ = env.reset()
    pol = CarrotPolicy()
    while True:
        obs, r, te, tr, info = env.step(pol.act(obs, env.context()))
        if te or tr:
            break
    assert info['success'], info


def test_hovering_in_place_does_not_pay():
    env = CheckpointEnv(COURSE, randomize=False, random_path_prob=0.0, mid_start_prob=0.0, jitter_m=0.0, seed=0)
    env.reset()
    total = sum(env.step(np.zeros(ACT_DIM))[1] for _ in range(200))
    assert total < 0
