# jolie_course: Jolie's online path planner

**Sky Beta contract only, the same interface as on the drone. No `/fmu` topics.**

```
bash flyx_skybeta/run.sh jolie          # FlyX / sim: organizer's flight interface + this + sim pilot
python3 jolie_course/jolie_course.py    # on the drone (the interface is already up; the pilot arms)
    [--ros-args -p depth_topic:=<depth image> -p shadow:=true]
```

The drone is given only the five checkpoints: first corridor, maze exit, tunnel, window or
Gate B, and the landing pad. It plans with Theta\* on a grid built from the published layout,
corrects that grid with the depth camera while it flies, and follows a 0.5 m carrot along the
plan. Its heading looks ahead along the path at ≤ 0.25 rad/s. At the end it centres over the pad
to 8 cm and holds (status `HOLD`, `ready_to_land: true`), and the pilot lands.

`jolie_course.py` only runs `pathfinding/flyx/pathplan_mission.py`, a self-contained node
generated from Jolie's code:

| Source | |
|---|---|
| `pathfinding/pathplan/` | the planner, the map, depth projection, the planner process, and an offline benchmark |
| `pathfinding/ros2/dronehacks_pathplan/` | the Sky Beta ROS node (also a colcon package). Isaac keeps it in step with Jolie's FlyX fixes |

Edit the sources, then rebuild with `python3 pathfinding/tools/build_single_file.py`.
`tests/test_single_files.py` fails while the generated node is stale.

| | Topic |
|---|---|
| reads | `/visual_slam/tracking/odometry`, `/skybeta/state`, optional depth image + `camera_info` |
| writes | `/skybeta/setpoint/pose`, `/dronehacks_pathplan/status` (JSON: `state`, `ready_to_land`, `cleared`, ...) |

Depth: in FlyX, use `/camera/depth/image_raw` (`run.sh` passes it). The drone has no depth topic,
so leave `depth_topic` empty and it flies the published map only. Parameters:
`pathfinding/ros2/dronehacks_pathplan/README.md`.

Needs numpy. Tests: `cd pathfinding && python3 -m pytest -q tests`.
