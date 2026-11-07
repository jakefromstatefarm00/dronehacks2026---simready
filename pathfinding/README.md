# pathfinding: Jolie's online path planner

Everything needed to run, test and change Jolie's pathfinding. It flies as **`jolie_course`**
(`../jolie_course/`), on the Sky Beta contract, the same interface as on the drone.

| | What | Where |
|---|---|---|
| **The algorithm** | Theta\* re-planning on a map built from the published layout and corrected by a depth camera | `pathplan/` |
| **The flight program** | state machine + ROS node on the Sky Beta contract (no `/fmu`) | `flyx/mission.py`, `flyx/fly_pathplan.py` |
| **The simulation** | Offline benchmark (seconds) | `pathplan/harness.py` |

## Quick start

```bash
cd pathfinding
python3 -m venv .venv && .venv/bin/pip install -r requirements.txt     # Windows: .venv\Scripts\pip
.venv/bin/python -m pytest -q tests                                     # 154 tests, ~15 s
.venv/bin/python -m pathplan.harness --variants 4 --output runs/bench   # offline benchmark, ~4 min
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

## Flying it (`flyx/`, run as `jolie_course`)

```bash
bash flyx_skybeta/run.sh jolie                  # FlyX / sim, from the repo root
python3 jolie_course/jolie_course.py            # on the drone (the pilot arms)
```

`flyx/fly_pathplan.py` is the ROS node. It reads `/visual_slam/tracking/odometry` and
`/skybeta/state` and publishes `/skybeta/setpoint/pose`. It never arms, never changes mode and
never lands. `flyx/mission.py` is the state machine; it is pure Python and unit-tested. See
[`flyx/README.md`](flyx/README.md) and [`../jolie_course/README.md`](../jolie_course/README.md).

## The simulation

**Offline benchmark** (`python -m pathplan.harness`): 13 courses (published + shifted walls +
unmapped clutter), kinematic drone, ray-cast depth camera. Sky Beta size, Theta\*:

| Planner | Finished | Crashed | Mean checkpoint pts (of 35) |
|---|---:|---:|---:|
| published map only (`static`) | 5 | 8 | 25.4 |
| **Theta\* replanning** | **13** | 0 | **35.0** |

**Gazebo (history).** Earlier versions were flown in a plain Gazebo Harmonic world, using the
course and the X3 quad from [machmoon/dronehacks-sim](https://github.com/machmoon/dronehacks-sim)
with ground-truth pose. That test bed (`gzsim/`) has been removed; it is in git history. Results:

| Run | World | Depth | Result |
|---|---|---|---|
| 02 | published | on | completed, 80.3 s, 0 contacts, landed 0.05 m |
| 05 | + 1 unmapped cylinder | on | completed, 79.0 s, 0 contacts (cylinder included), landed 0.07 m |
| 06 | same cylinder | **off** | **failed: hit the cylinder** after the tunnel |
| commit-1 | + 1 unmapped cylinder | on | completed, 109.1 s, 0 contacts, landed 0.07 m (with path commitment) |
| commit-3 | published | on | completed, 98.1 s, 0 contacts, min clearance 0.379 m, landed 0.06 m |

**Known limits.** The X3 (radius 0.36 m) is bigger than Sky Beta (0.28 m): the passage over
maze wall M1 leaves it ~6 cm, so Gazebo runs need the 2.5 cm grid (`--resolution`) and are
not always clean there (one earlier run touched the north net). The adapter now slows down where
the published map is tight (`--tight-speed`, from map clearance, never ground truth); both runs since
were clean. Gazebo
uses ground-truth pose (no cuVSLAM) and velocity control (not PX4). Near an unmapped obstacle two routes can cost almost the same; the planner now keeps its route
unless a new one is >10 % cheaper (`commit_margin`), which fixed the verify-3 stall. Gazebo
times are slow (98–197 s) because of the X3's cautious tight-spot speed (0.15 m/s) — speed
tuning is open. Depth frames are
projected with the latest pose, not the pose at capture time — fine at these speeds, the
next thing to fix. Checkpoint lines 1–2 are our reading of the rules, not official
coordinates.

## Layout

```
pathplan/        the algorithm + offline benchmark
flyx/            mission.py (state machine), fly_pathplan.py (ROS node, Sky Beta), px4_link.py (frames)
course/ config/  the published course (OBJ) and its gate list
drl/             the team's OBJ -> collision-primitives loader (used by pathplan)
tests/
```
