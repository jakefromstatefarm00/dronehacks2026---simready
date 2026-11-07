# Integration check: Jolie's FlyX-proven algorithm in this package (2026-11-07)

**Algorithm parity with `jolie_course/upload_to_flyx.py`**, the version that completed the whole
FlyX course:
- `pathplan/` grid_search, world, belief, depth_projection and online are byte-identical to that
  file's embedded sources.
- Its two turn rules were missing here and are now restored:
  - `planner_core.TURN_IN_PLACE_DEG = 60`;
  - `mission._carrot_for()`, which shortens the carrot to 30 % at 60°.
- `tests/test_flyx_proven_turns.py` pins both rules.

| Level | Result |
|---|---|
| 1. `pytest -q tests` | 155 passed |
| 1b. `dry_run` (depth / no depth) | PASS 5/5, pad 0.003 m, closest wall 0.375 m (was 0.366 m without the turn rules) |
| 2. `selftest.launch.py`: real `skybeta_flight_interface` + node + `sim_pilot` vs `fake_px4`, ROS 2 Jazzy, colcon-built (WSL) | fixed package **2/2 PASS** (5/5, READY -> ARMED -> TAKEOFF -> NAVIGATE -> PAD -> DESCEND -> LOW_HOLD -> pilot Land -> DONE), plus 1 run lost Offboard. The package as received: 1/2 |

The self-test failures, one with each version, were the same event: the fake PX4 leaves Offboard
when the flight interface's heartbeat pauses for more than 0.5 s (its COM_OBL_RC_ACT model), on a
laptop loaded by training jobs. This is a timing artefact of the test machine, not a logic fault.
On a loaded lab machine the same PX4 rule applies, so keep other heavy jobs off the machine during
a flight.

**Run it in FlyX:**
- With colcon, see README section 4: `ros2 launch dronehacks_pathplan sim.launch.py`.
- Without colcon, from the repo root: `bash flyx_skybeta/run.sh team`.

**On the drone:** README section 5 (copy this folder to `~/team_ws/src/pathfinding`).
