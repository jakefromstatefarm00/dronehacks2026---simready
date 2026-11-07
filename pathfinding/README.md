# pathfinding — online path planner, FlyX single file, Gazebo test bed

Everything needed to run, test and change our pathfinding. Three things live here:

| | What | Where |
|---|---|---|
| **The algorithm** | Theta\* re-planning on a map built from the published layout and corrected by a depth camera | `pathplan/` |
| **The FlyX file** | One self-contained ROS 2 node to drop into FlyX in place of `square_mission.py` | `flyx/pathplan_mission.py` |
| **The simulation** | Offline benchmark (seconds) + Gazebo Harmonic flights in Docker (minutes) | `pathplan/harness.py`, `gzsim/` |

## Quick start

```bash
cd pathfinding
python3 -m venv .venv && .venv/bin/pip install -r requirements.txt     # Windows: .venv\Scripts\pip
.venv/bin/python -m pytest -q tests                                     # 126 tests, ~15 s
.venv/bin/python -m pathplan.harness --variants 4 --output runs/bench   # offline benchmark, ~4 min
bash gzsim/run.sh                                                        # Gazebo flight (Linux/WSL + Docker)
```

## The algorithm (`pathplan/`)

The drone gets only the **five checkpoints** (first corridor, maze exit, tunnel, window or
Gate B, landing pad) — no waypoint list. It plans on a 2-D occupancy grid at 0.9 m, built
from the published OBJ, and corrects the grid from depth images while flying.

| Module | Role |
|---|---|
| `world.py` | course -> occupancy/clearance/cost grids; checkpoint lines |
| `belief.py` | the drone's map: published layout + depth observations |
| `grid_search.py` | **Theta\*** (default, any-angle) and A\* |
| `online.py` | `OnlinePlanner`: `observe(depth cells)`, `update(pose) -> move / turn / done / failed` |
| `depth_projection.py` | depth image + intrinsics + pose -> map cells |
| `ros_node.py` | odom frame helpers and `build_planner()` |
| `mission.py`, `sensor.py`, `variants.py`, `harness.py` | offline simulator + benchmark over perturbed courses |
| `dstar_lite.py` | D\* Lite (experimental; slower than Theta\* here) |

Behaviours that each fixed a failure we saw: look ahead before moving (turn in place if the
next stretch is outside the camera view); treat space just behind a newly seen obstacle as
risky; never straighten a path toward walls; replan when the map changes near the path;
when stuck, look around, then doubt published walls the camera has not confirmed; accept
Gate B when the window approach is blocked.

## The FlyX file (`flyx/pathplan_mission.py`)

A fork of the organisers' `competitor_example/square_mission.py` (same skybeta contract:
`/skybeta/state` + `/visual_slam/tracking/odometry` in, `/skybeta/setpoint/pose` out; never
`/fmu/*`, never arms). WAIT_FOR_OFFBOARD -> TAKEOFF 0.9 m -> NAVIGATE (planner) -> HOLD
over the pad. Needs only `rclpy`, the standard message packages and `numpy`.

```bash
python3 pathplan_mission.py --ros-args -p shadow:=true                 # plan only, no setpoints
python3 pathplan_mission.py --ros-args -p depth_topic:=<depth image>   # fly with depth
ros2 topic echo /dronehacks_pathplan/status
```

- **Start on the launch mark facing up the first corridor**: the map is aligned from the
  start pose (cuVSLAM's `odom` origin).
- Without `depth_topic` it flies the published map only (no obstacle sensing).
- It ends holding over the pad: the skybeta interface has no land command.
- **Do not edit it by hand.** It is generated from `ros2/dronehacks_pathplan/` + `pathplan/`:
  `python tools/build_single_file.py`. The ROS 2 package version is in `ros2/`.

## The simulation

**Offline benchmark** (`python -m pathplan.harness`): 13 courses (published + shifted walls +
unmapped clutter), kinematic drone, ray-cast depth camera. Sky Beta size, Theta\*:

| Planner | Finished | Crashed | Mean checkpoint pts (of 35) |
|---|---:|---:|---:|
| published map only (`static`) | 5 | 8 | 25.4 |
| **Theta\* replanning** | **13** | 0 | **35.0** |

**Gazebo** (`gzsim/run.sh`): real collisions and multicopter dynamics, Gazebo Harmonic 8.15 in
Docker, using the course world and Open Robotics X3 quad from
[machmoon/dronehacks-sim](https://github.com/machmoon/dronehacks-sim) (cloned on first run,
not our code). `gzsim/make_world.py` adds a forward depth camera (80° FOV) and optional
unmapped cylinders; `gzsim/fly_planner.py` flies `OnlinePlanner` with ground-truth pose.

| Run | World | Depth | Result |
|---|---|---|---|
| 01 | published | off | completed, 59.1 s, 0 contacts, through the window, landed 0.17 m from pad centre |
| 02 | published | on | completed, 80.3 s, 0 contacts, landed 0.05 m |
| 05 | + 1 unmapped cylinder | on | completed, 79.0 s, 0 contacts (cylinder included), landed 0.07 m |
| 06 | same cylinder | **off** | **failed: hit the cylinder** after the tunnel |
| verify-1 | published | on | completed, 107.2 s, 0 contacts, min clearance 0.369 m (X3 radius 0.36), landed 0.03 m � with tight-spot slow-down |
| verify-2 | published | on | completed, 92.2 s, 0 contacts, min clearance 0.378 m, landed 0.05 m � same settings, repeat |

```bash
bash gzsim/run.sh                          # published course, depth on
bash gzsim/run.sh --clutter 3              # + unmapped obstacles
bash gzsim/run.sh --clutter 3 --no-depth   # same world without sensing (expect a crash)
# watch: http://localhost:8080/vnc.html?autoconnect=1&resize=scale ; results: gzsim/runs/<name>/
```

**Known limits.** The X3 (radius 0.36 m) is bigger than Sky Beta (0.28 m): the passage over
maze wall M1 leaves it ~6 cm, so Gazebo runs need the 2.5 cm grid (`--resolution`) and are
not always clean there (one earlier run touched the north net). The adapter now slows down where
the published map is tight (`--tight-speed`, from map clearance, never ground truth); both runs since
were clean. Gazebo
uses ground-truth pose (no cuVSLAM) and velocity control (not PX4). Depth frames are
projected with the latest pose, not the pose at capture time — fine at these speeds, the
next thing to fix. Checkpoint lines 1–2 are our reading of the rules, not official
coordinates.

## Layout

```
pathplan/        the algorithm + offline benchmark
flyx/            pathplan_mission.py: THE file for FlyX (generated)
ros2/            dronehacks_pathplan: the same node as a ROS 2 package (source of the file)
gzsim/           Gazebo test bed: run.sh, make_world.py, fly_planner.py
tools/           build_single_file.py
course/ config/  the published course (OBJ) and its gate list
drl/             the team's OBJ -> collision-primitives loader (used by pathplan)
tests/
```
