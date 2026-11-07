# flyx — run the planner in the FlyX cloud lab

**The FlyX lab is native PX4** (ROS 2 Jazzy, PX4 SITL, `/fmu/in|out/*`, cuVSLAM, simulated
OAK-D depth). It has **no `/skybeta/state`**, so use `fly_pathplan.py` there.
`pathplan_mission.py` is the skybeta-interface version, for a stack that runs skybeta
(the real drone); in FlyX it would wait for Offboard forever.

| File | What it is |
|---|---|
| `check_lab.py` | **Run first.** Read-only: finds PX4 topics, rates, depth + CameraInfo, sim speed. Last line = the command to run. |
| `fly_pathplan.py` | The mission program. **Shadow by default** (sends nothing to PX4); `--execute` flies. |
| `mission.py` | The state machine (pure Python, unit-tested): what to publish, when to land. |
| `px4_link.py` | NED <-> course frame, PX4 command constants, topic discovery. |
| `fake_px4.py` | Test only: fake PX4 for wiring checks without a simulator. Never point it at a real PX4. |
| `pathplan_mission.py` | Generated single-file skybeta node (see `../README.md`). |

## Runbook

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
Lands on: odometry > 0.3 s old, local position > 0.5 s, vehicle_status > 3 s, estimator
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
