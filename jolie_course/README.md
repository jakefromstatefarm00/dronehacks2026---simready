# jolie_course: Jolie's online path planner

**Sky Beta contract only, the same interface as on the drone. No `/fmu` topics.**

```
bash flyx_skybeta/run.sh jolie          # FlyX / sim: organizer's flight interface + this + sim pilot
python3 jolie_course/jolie_course.py    # on the drone (the interface is already up; the pilot arms)
    [--depth-topic <depth image>] [--shadow]
```

The drone is given only the five checkpoints: first corridor, maze exit, tunnel, window or
Gate B, and the landing pad. It plans with Theta\* on a grid built from the published layout,
corrects that grid with the depth camera while it flies, and follows a 0.5 m carrot along the
plan. Its heading looks ahead along the path at ≤ 0.25 rad/s. At the end it centres over the pad
and holds (status `LAND`), and the pilot lands.

`jolie_course.py` is only the entry point. The code is Jolie's, in `pathfinding/`:

| | |
|---|---|
| `pathfinding/pathplan/` | the planner, the map and depth projection, plus an offline benchmark |
| `pathfinding/flyx/mission.py` | the state machine: takeoff, carrot, yaw cap, pad centring, guards |
| `pathfinding/flyx/fly_pathplan.py` | the ROS node: Sky Beta I/O, planner in its own process, depth at capture-time pose, journal |

| | Topic |
|---|---|
| reads | `/visual_slam/tracking/odometry`, `/skybeta/state`, optional depth image + `camera_info` |
| writes | `/skybeta/setpoint/pose`, `/jolie_course/status` |

`mission.py` still models PX4's Offboard, arm and Land steps. On this contract those requests are
not sent: the pilot arms and the interface enters Offboard. They are journalled as
`pilot_action`.

Depth: in FlyX, use `/camera/depth/image_raw` (`run.sh` passes it). The drone has no depth topic,
so leave `--depth-topic` empty and it flies the published map only.

Needs numpy and scipy. Tests: `cd pathfinding && python3 -m pytest -q tests`.
Journal: `jolie_course/runs/jolie-<time>/journal.jsonl`.
