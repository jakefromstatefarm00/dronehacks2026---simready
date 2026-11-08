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

## Review 2 fixes (2026-11-07)

| Level | Result |
|---|---|
| `pytest -q tests` | 190 passed (155 + 35 new: `test_review_fixes.py`, `test_runtime.py`, `test_launch_config.py`) |
| `dry_run` default / `--no-depth` / `--dropout 30 1` | PASS 5/5 each, pad 0.003 m, closest wall 0.375 m |
| Reviewer scenarios (`review2/scen.py`) | 1: no FAULT on a 0.2 s px4_link drop; 2: AUTO_LOITER 1.5 s ridden out, back to NAVIGATE; 3: READY -> ARMED -> TAKEOFF; 5: LOW_HOLD 4.7 s after the descent began despite a 9 cm z offset; 6: LOW_HOLD at the 175 s run limit |
| `selftest.launch.py` (WSL, Jazzy, colcon) | **3/3 PASS** (5/5, pad 0.07-0.08 m). In every run the fake PX4 entered its failsafe Hold (14 -> 4 -> 14) three times on > 0.5 s heartbeat gaps and the node rode them out (0.4 s in total); before fix 2 each of these ended the flight in TAKEN_OVER. |

The fake PX4 now models PX4 v1.16's failsafe Hold (COM_FAIL_ACT_T 5 s) instead of dropping
straight to Position, and flight_interface runs on the wall clock in simulators
(`interface_use_sim_time:=false`); see the commit messages for the PX4 source evidence.
