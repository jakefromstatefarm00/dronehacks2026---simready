"""Optional: use the real course walls in training (instead of only the corridor).

Uses the RL repo's course model (drl/course.py), which reads the organisers' OBJ. Point
DRONEHACKS_REPO at the repo root (the folder that contains drl/), or run from inside it:

    DRONEHACKS_REPO=~/dronehacks2026 python -m team_mission.rl.train ... --course-obj course/dronehacks-course-v3.obj
"""
from __future__ import annotations

import os
import sys


def load_course(obj_path: str):
    repo = os.environ.get('DRONEHACKS_REPO')
    candidates = [repo] if repo else []
    here = os.path.abspath(os.path.dirname(__file__))
    for _ in range(8):                      # walk up looking for drl/course.py
        candidates.append(here)
        here = os.path.dirname(here)
    candidates.append(os.getcwd())
    for c in candidates:
        if c and os.path.isfile(os.path.join(c, 'drl', 'course.py')):
            if c not in sys.path:
                sys.path.insert(0, c)
            break
    try:
        from drl.course import build_course
    except ImportError as e:
        raise ImportError('course_obj needs the repo\'s drl package: set DRONEHACKS_REPO to the repo root') from e
    path = obj_path
    if not os.path.isabs(path) and not os.path.exists(path) and repo:
        path = os.path.join(repo, obj_path)
    return build_course(path)
