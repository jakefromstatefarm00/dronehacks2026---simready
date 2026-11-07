# flyx — run the planner in the FlyX cloud lab

**Upload `upload_to_flyx.py` to FlyX** ("Run saved file"). FlyX copies only that one file
(as `project/main.py`), so it is self-contained: planner, state machine and course map are
embedded. **It flies by default** (Offboard, arm, takeoff, course, land) with depth from
`/camera/depth/image_raw`; pass `--shadow` to plan without sending anything to PX4.
Reset world first; start on the launch mark facing up the first corridor.

The FlyX lab is native PX4 with **no `/skybeta/state`**: `pathplan_mission.py` (skybeta
interface) would wait for Offboard forever there.

| File | What it is |
|---|---|
| `check_lab.py` | **Run first.** Read-only: finds PX4 topics, rates, depth + CameraInfo, sim speed. Last line = the command to run. |
| `fly_pathplan.py` | The mission program. **Shadow by default** (sends nothing to PX4); `--execute` flies. |
| `mission.py` | The state machine (pure Python, unit-tested): what to publish, when to land. |
| `px4_link.py` | NED <-> course frame, PX4 command constants, topic discovery. |
| `fake_px4.py` | Test only: fake PX4 for wiring checks without a simulator. Never point it at a real PX4. |
| `upload_to_flyx.py` | **Generated** single file of `fly_pathplan.py` for FlyX upload (flies by default). |
| `pathplan_mission.py` | **Generated** single-file skybeta node (see `../README.md`). |

Don't edit the generated files by hand: change the sources and run `python tools/build_single_file.py`
(`tests/test_single_files.py` fails if they are stale).

## Runbook (terminal in the lab, from a clone of this repo)

```bash
cd pathfinding
python3 -c "import numpy, scipy, yaml" || python3 -m pip install --user numpy scipy pyyaml
python3 -m flyx.check_lab                                              # ~20 s, read-only
python3 -m flyx.fly_pathplan --depth-topic /camera/depth/image_raw     # SHADOW: Ctrl-C after ~20 s
python3 -m flyx.fly_pathplan --execute --depth-topic /camera/depth/image_raw   # flies (Reset world first)
ros2 topic echo /flyx_pathplan/status                                  # second terminal
```

- Use the depth topic `check_lab` prints (in the lab: `/camera/depth/image_raw`, 16UC1 640x400).
  A wrong topic is not an error: it just flies the published map with no sensing.
- **Reset world** before each `--execute` run; start on the launch mark facing up the first corridor.
- Stop: Ctrl-C or SIGTERM sends Land while airborne. Results: `runs/flyx-*/journal.jsonl`
  (`state` lines say *why* it changed state; `timing` lines record clock jumps / stalls).

```
WAIT_HEALTH  fresh odometry (NED), status, local position xy/z valid, landed + disarmed, 2 s
  -> SHADOW (default) plans and logs only          -> STREAM hold setpoints 1 s
  -> Offboard -> arm -> TAKEOFF to 0.9 m -> NAVIGATE (Theta* replanning, carrot 0.5 m ahead)
  -> LAND (PX4 native) at the pad -> DONE when landed + disarmed
```
Lands on: odometry > 0.3 s old, local position (validity flags) > 2 s, vehicle_status > 3 s, estimator
invalid, planner failure, 175 s of sim time, Ctrl-C/SIGTERM. If PX4 leaves Offboard itself
(failsafe, RC) it stops commanding (TAKEN_OVER).

## FlyX run 1 (teammate) and what changed

Reached NAVIGATE, then landed ~20 s in: `pose stale or estimator invalid`. Found and fixed:

1. **Clock jumps.** Message ages were measured on the ROS clock (sim time). That clock can
   step (~1 s seen in WSL wall time; sim `/clock` can arrive in bursts), so fresh odometry
   looked 1.4 s old. Reproduced against `fake_px4.py`; ages now use a monotonic clock, sim
   time only for the flight limit. Same test then completed the course.
2. **Status rate.** `vehicle_status` comes at 1.7 Hz and `land_detected` at 0.9 Hz in the lab;
   the old 1.0 s limit was too tight (now 3 s; odometry still 0.3 s).
3. The land reason now names the failed check.
4. Shutdown: no more `rcl_shutdown already called` / `I/O operation on closed file`;
   Ctrl-C no longer kills ROS before Land is published; SIGTERM also lands.
5. `--depth-topic /stereo/depth` does not exist in the lab; use `/camera/depth/image_raw`.

**Runs 3-4** landed during takeoff on "local_position stale (0.69 s)", then "odometry stale
(0.42 s)": our own process stalled, not PX4. Planning and depth projection are pure-Python CPU
work and shared the GIL with the PX4 callbacks. Reproduced with `FAKE_DEPTH=1 fake_px4.py`
(ray-cast 640x400 depth at 13 Hz) on a fully loaded CPU: 8 stalls up to 0.41 s. Fixes:
- the planner + depth projection run in a **separate process** (`PlannerProcess`): same test,
  0 stalls, full course flown and landed on the pad;
- depth frames are placed with the **pose at capture time** (odometry nearest the image stamp),
  not the latest pose: with the latest pose, turns smeared walls and sealed the bend after
  checkpoint 1 ("no path");
- local_position (validity flags only) may be 2 s old; odometry still 0.3 s.

**Run 5** took off, then PX4 left Offboard 0.8 s into NAVIGATE (TAKEN_OVER, cause not logged).
Review + fixes, verified on 6/6 fake-PX4 flights with ray-cast depth (3 idle, 3 with half the
CPU busy) and 13/13 offline courses:
- setpoint loop on a steady clock and timestamps in PX4's own time base (sim-clock stalls or
  slow sim time make PX4 see the Offboard stream as stale); TAKEN_OVER now says *why*
  (PX4 failsafe vs someone else changed mode) and PX4 mode changes are journalled (`px4_mode`);
- depth that a slightly wrong pose invents is dropped (returns within 15 cm of a published
  wall, published walls seen "free", frames taken while turning > 0.5 rad/s): offline the course
  now completes with 0.3 s of pose error, before it failed at 0.1 s;
- "look around" counts real heading change and steps 15 deg per depth frame; "no goal cell"
  looks around and doubts the map before giving up; a line checkpoint's goal pocket can only be
  entered across the line (no back route via the maze);
- the setpoint carrot is pulled in so the straight line to it stays out of the keep-out;
- no mapping/planning below 0.5 m; Offboard/arm/Land re-sent every 1 s until PX4 shows them;
  takeoff accepts within 0.25 m after 10 s; an exception in any callback lands the vehicle.

**Run 6** cleared checkpoint 1 and entered the maze. The journal showed: (1) the control loop
stalled ~1 s twice at TAKEOFF -> NAVIGATE and PX4 briefly failsafed (`offboard_control_signal_lost`)
- the planner process now runs at lower CPU priority (`os.nice`); (2) "no path" at the tight passage
past maze wall M1 - new depth obstacles must be seen in 2 of ~3 frames, and returns within 25 cm of a
published wall count as that wall; (3) the look-around always turned left and re-held "here" each step
(drifted 11 cm toward a wall), then VIO lost xy facing the wall -> land - it now turns toward the
goal's side first and holds the spot where it began. New journal diagnostics: `depth.quality`
(share of depth hits on published walls: low = wrong camera mount) and `tf_static` (the sim's mounts).
Also: the commanded heading is rate-limited to **0.25 rad/s** (`yaw_rate_max`). Suraj's
flyx_course data: 30 of 40 visual-odometry losses at yaw rates above 0.35 rad/s, 3/3 Gazebo
completions at 0.25. Fake-PX4: 4/4 complete, ~45 s takeoff to landing.

**Lab robustness (from Suraj's flyx_course journals):** in the air, odometry older than 0.3 s
*holds* the last setpoint and only a 3 s outage lands (FlyX pauses odometry up to ~0.7 s);
status / local position land after 10 s of silence (gaps of 2.4-3.3 s measured), a fresh
"estimator invalid" still lands at once. Another program publishing `/fmu/in` setpoints blocks
arming and lands in flight. At the pad it centres to 8 cm before Land (was up to 0.4 m off;
fake-PX4 now 0.04-0.07 m). Every run ends with a terminal summary: result and reason,
checkpoints, PX4 mode changes + failsafe flags, stalls, depth hit quality, planner events.

Tested: `tests/test_flyx_mission.py`; full ROS wiring in Jazzy against `fake_px4.py`
(status at the lab's slow rates): course completed, 3 commands (Offboard, arm, land);
shadow + Ctrl-C sends nothing; `--execute` + SIGTERM mid-flight sends Land.
Not tested: real PX4 dynamics, cuVSLAM, real depth.

| Option | Default | Notes |
|---|---|---|
| `--depth-topic`, `--camera-info-topic` | none / sibling `camera_info` | from `check_lab` |
| `--camera-offset X Y Z` | `0.115 0 0.029` | depth origin in body FLU (Sky Beta CAD) |
| `--altitude` | `0.9` | course frame, below the 1.73 m walls |
| `--carrot` | `0.5` | m ahead on the plan; lower if PX4 flies too fast |
| `--launch-xy`, `--launch-heading` | `-2.515 0.914`, `1.5708` | launch mark, facing +y |
| `--planner` | `theta` | `astar`, or `static` (no sensing) as a baseline |
| `--wall-clock` | off | only for `fake_px4.py` tests (no `/clock`) |
