"""Jolie's course: her online path planner (pathfinding/) flying the course on the Sky Beta contract,
the same interface as on the drone. No /fmu topics.

    bash flyx_skybeta/run.sh jolie            # FlyX / sim: flight interface + this + sim pilot
    python3 jolie_course/jolie_course.py      # on the drone (the interface is up; the pilot arms)
        [--depth-topic /camera/depth/image_raw] [--shadow] [--odometry-topic ...]

This file only starts pathfinding/flyx/fly_pathplan.py, so Jolie's code stays the one source:
    pathfinding/pathplan/       the planner (Theta* on the published map + depth corrections)
    pathfinding/flyx/mission.py the state machine (takeoff, carrot, yaw cap, pad centring, guards)
    pathfinding/flyx/fly_pathplan.py   the ROS node (Sky Beta I/O, planner process, depth, journal)
Needs numpy and scipy. Journal: jolie_course/runs/jolie-<time>/journal.jsonl.
"""
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "pathfinding"))

from flyx.fly_pathplan import main  # noqa: E402

if __name__ == "__main__":
    sys.exit(main())
