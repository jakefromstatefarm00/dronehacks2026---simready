"""Check a course-frame checkpoint file against the course model: every straight segment must
keep `--radius` metres from walls, tube and window frame.

    DRONEHACKS_REPO=<repo root> python -m team_mission.rl.check_course config/checkpoints/course_v3.yaml \
        --obj course/dronehacks-course-v3.obj --radius 0.25
"""
from __future__ import annotations

import argparse
import math

import numpy as np

from team_mission.path import CheckpointSpec, resolve_path
from team_mission.rl.course_collision import load_course


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('checkpoints')
    ap.add_argument('--obj', default='course/dronehacks-course-v3.obj')
    ap.add_argument('--radius', type=float, default=0.25)
    a = ap.parse_args()
    import yaml
    spec = CheckpointSpec.load(a.checkpoints)
    st = (yaml.safe_load(open(a.checkpoints)) or {}).get('train_start', {'xyz': [0, 0, 0], 'yaw_deg': 0})
    path = resolve_path(spec, np.asarray(st['xyz'], float), math.radians(st.get('yaw_deg', 0)), 1.0)
    course = load_course(a.obj)
    bad = 0
    for i in range(1, path.n + 1):
        A, B = path.points[i - 1], path.points[i]
        P = A + (B - A) * np.linspace(0, 1, 200)[:, None]
        hits = int(course.collides(P, a.radius, 0.10).sum())
        bad += hits > 0
        print(f'{path.names[i - 1]:>12} -> {path.names[i]:<12} {np.linalg.norm(B - A):5.2f} m  '
              f'{"OK" if hits == 0 else f"TOO CLOSE ({hits}/200 samples)"}')
    print(f'path length {path.length:.2f} m, {path.n} checkpoints, {bad} bad segment(s)')
    raise SystemExit(1 if bad else 0)


if __name__ == '__main__':
    main()
