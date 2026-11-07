"""Jolie's course: her online path planner flying the course on the Sky Beta contract, the same
interface as on the drone. No /fmu topics.

    bash flyx_skybeta/run.sh jolie            # FlyX / sim: flight interface + this + sim pilot
    python3 jolie_course/jolie_course.py      # on the drone (the interface is up; the pilot arms)
        [--ros-args -p depth_topic:=/camera/depth/image_raw -p shadow:=true ...]

This file only runs pathfinding/flyx/pathplan_mission.py, the self-contained Sky Beta node
generated from Jolie's planner (pathfinding/pathplan/) and its ROS node
(pathfinding/ros2/dronehacks_pathplan/, kept in step with her FlyX fixes by Isaac). Edit those
sources and rebuild with `python3 pathfinding/tools/build_single_file.py`. Needs numpy.
Parameters: see pathfinding/ros2/dronehacks_pathplan/README.md.
"""
from pathlib import Path
import runpy
import sys

NODE = Path(__file__).resolve().parents[1] / "pathfinding" / "flyx" / "pathplan_mission.py"

if __name__ == "__main__":
    sys.argv[0] = str(NODE)
    runpy.run_path(str(NODE), run_name="__main__")
