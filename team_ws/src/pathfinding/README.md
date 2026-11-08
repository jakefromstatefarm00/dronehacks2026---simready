# pathfinding: our path planner as a Sky Beta competitor package

This whole folder is **one ROS 2 package, `dronehacks_pathplan`**. It goes in `team_ws/src/`
**unchanged, on the drone and in the simulator**. It runs beside the organisers' Sky Beta
stack, which stays untouched.

It is the flight logic of the old `upload_to_flyx.py`, moved onto the organisers' contract:

- the same Theta\* replanning and depth voting
- the same carrot setpoints
- the same 0.25 rad/s heading limit, odometry-pause hold and pad centring
- the planner in its own process
- the same run limit

The difference is that **nothing talks to PX4 directly any more**:

| | old `upload_to_flyx.py` (native PX4) | **this package (Sky Beta)** |
|---|---|---|
| Who arms / enters Offboard | the program itself | **the pilot** arms (Altitude), then selects Position; `flight_interface` enters Offboard |
| Setpoints | `/fmu/in/trajectory_setpoint` (NED) | `/skybeta/setpoint/pose` (ENU `odom`); `flight_interface` converts and clamps |
| Health | `/fmu/out/*` | `/skybeta/state` + `/visual_slam/tracking/odometry` |
| End of the course | PX4 Land | descend to the 0.3 m floor over the pad and **hold: the pilot lands** (Sky Beta has no land command) |
| Lost sensors in the air | PX4 Land | `/skybeta/hold` (freeze), stop commanding: **the pilot takes over** |
| Runs on the drone? | **no**: it breaks the "never publish `/fmu/in`" rule, and `flight_interface` blocks it | **yes**: it is what the competitor window launches |

The old single-file programs are kept outside the workspace, in
`dronehacks2026---simready-main/pathfinding_flyx_original/`. Do not put them back into
`team_ws`.

---

## 1. What is in the folder

```
pathfinding/                       <- = the ROS 2 package dronehacks_pathplan (package.xml, setup.py)
├── dronehacks_pathplan/           the Sky Beta side
│   ├── node.py                    THE NODE: ROS in/out, journal, graph checks         (pathplan_node)
│   ├── mission.py                 the state machine, pure Python: what to send, when
│   ├── planner_core.py            planner + depth voting; runs in its own process
│   ├── sim_pilot.py               SIM ONLY: stands in for the safety pilot            (sim_pilot)
│   ├── fake_px4.py                TEST ONLY: fake PX4 + cuVSLAM for the self-test     (fake_px4)
│   ├── check_pipeline.py          read-only: is everything flowing?                   (check_pipeline)
│   ├── report.py                  journal -> timeline, result, plot                   (report)
│   └── dry_run.py                 closed loop without ROS                             (dry_run)
├── launch/
│   ├── pathplan.launch.py         THE DRONE LAUNCH: flight_interface + our node (competitor window)
│   ├── sim.launch.py              simulator: the drone launch + sim pilot (+ vision bridge if missing)
│   └── selftest.launch.py         no simulator: real flight_interface + our node + fake PX4
├── pathplan/                      the planner (Theta*/A*, occupancy, depth projection, benchmark)
├── drl/                           OBJ -> course geometry loader
├── config/course_v3.yaml, course/ the published course (installed with the package)
├── tests/                         pytest, no ROS needed (152 tests, ~25 s)
└── gzsim/                         Gazebo test bed for the planner alone (development tool)
```

The build installs the `dronehacks_pathplan`, `pathplan` and `drl` modules, the launch files
and the course into the overlay. `gzsim/` and `tests/` are not installed.

---

## 2. How it fits in the pipeline

```
           DRONE                                         FLYX LAB (simulator)
 OAK-D -> cuVSLAM (slam window)                 simulated stereo -> lab cuVSLAM
      -> /visual_slam/tracking/odometry  ======  /visual_slam/tracking/odometry
 px4_odom_bridge (bridge window)                 lab vision bridge (or Sky Beta's; sim.launch decides)
      -> /fmu/in/vehicle_visual_odometry -> PX4 EKF2
 XRCE agent (agent window)                       lab XRCE agent (never start a second one)

 ---------------- competitor window = `ros2 launch dronehacks_pathplan pathplan.launch.py` ----------------
 flight_interface (Sky Beta, unmodified)  <- /skybeta/setpoint/pose -  dronehacks_pathplan (ours)
   |  owns the Offboard heartbeat, ENU->NED, 0.3 m floor,              -> /skybeta/state ->  |
   |  auto-Offboard when the pilot arms then selects Position           /visual_slam/... -> |
   v                                                                    depth (optional) -> |
 /fmu/in/{offboard_control_mode, trajectory_setpoint, vehicle_command} -> PX4
```

Our node only ever touches the four Sky Beta topics, plus `/skybeta/hold` on a fault.

---

## 3. What the drone does (the mission)

| State | What is happening | What our node sends |
|---|---|---|
| `WAIT_HEALTH` | Waits for **all** of: fresh odometry, `/skybeta/state`, PX4 link, a valid PX4 position, landed, disarmed, no `/skybeta/hold` that someone else asked for, nobody else commanding (incl. two vision bridges), odometry z within 0.1 m of `flight_interface`'s origin. All must hold for 2 s. | **nothing**, so `flight_interface` *cannot* enter Offboard |
| `READY` | Healthy on the ground. The note says *"pilot may arm"*. Odometry pauses up to 1.5 s keep it READY. Its first setpoint clears a `/skybeta/hold` this node asked for after an earlier FAULT. | "hold here" (the stream that allows Offboard) |
| `ARMED` | The pilot armed. **The course frame is latched now**: the drone must be on the launch mark, facing up the first corridor. | "hold here" |
| `TAKEOFF` | The pilot selected Position, so `flight_interface` entered Offboard. Climb on a 0.4 m/s ramp to 0.9 m. | climb setpoints |
| `NAVIGATE` | Theta\* through the five checkpoints. Carrot 0.5 m ahead, heading ≤ 0.25 rad/s. Odometry paused > 0.3 s: hold. | carrot setpoints |
| `PAD` → `DESCEND` | Pad reached: centre to ≤ 8 cm, then ramp down at 0.3 m/s to the floor (0.3 m above where SLAM started). `LOW_HOLD` when odometry z reaches the floor, or 3 s after the ramp does (PX4 flies on EKF2's height). | pad setpoints |
| `LOW_HOLD` | Holds low over the pad with `land_request: true`. **PILOT: LAND NOW.** PX4 disarms itself after touchdown. | hold low |
| `DONE` | Disarmed. After 3 s it goes back to `WAIT_HEALTH` with a fresh planner for the next flight. | nothing |

**When things go wrong:**

| Event | What the node does |
|---|---|
| PX4's failsafe Hold (`AUTO_LOITER`) after a setpoint-stream stall | keeps streaming the last setpoint for up to 5 s: PX4 returns to Offboard by itself (`COM_FAIL_ACT_T`). Still in Hold after 5 s: `TAKEN_OVER`. |
| The pilot takes over (sticks / mode switch to Position, Altitude, Manual, Land) | `TAKEN_OVER` after 0.5 s: stops commanding. Offboard returns only after disarm + re-arm. |
| `px4_link` false (no `vehicle_status` for 2 s) | holds the last setpoint, does not advance; `FAULT` after 8 s |
| Odometry gone 3 s, PX4 position invalid, `/skybeta/state` silent 5 s, another commander appears | `FAULT`: calls `/skybeta/hold` once and stops commanding. The pilot takes over and lands. The hold stays in force until this node is READY again (landed, disarmed, healthy). |
| Planner fails | descends where it is and `LOW_HOLD`s: the pilot lands |
| The 175 s run limit passes (TAKEOFF, NAVIGATE, PAD or DESCEND) | `LOW_HOLD` at once, ramping down to the floor: the pilot lands |
| Pilot armed before the node was READY | not flown: no setpoints. If our last setpoint was < 1.5 s old it calls `/skybeta/hold`, so `flight_interface` refuses Offboard. Disarm and wait for READY. |
| Our node crashes or is stopped in flight | `flight_interface` keeps holding the last setpoint (a hover). The pilot lands. |
| `shadow:=true` | maps and plans, never publishes a setpoint. Offboard cannot start. |

**Course frame.** cuVSLAM's `odom` frame starts wherever SLAM started. With
`frame_latch:=arm` (the default), the node reads the drone's odom pose **at the moment the
pilot arms**, and takes that pose to be the launch mark `(-2.515, 0.914)`, facing course +y.
It therefore does not matter where the drone was powered on, **as long as it sits on the mark,
facing up the first corridor, when it is armed**. `frame_latch:=slam_origin` instead assumes
SLAM started on the mark.

---

## 4. In the FlyX lab (simulator)

**Upload** into `/var/lib/flyx-lab/workspace`:

```
skybeta-main/                  (or skybeta/) the organisers' folder, unchanged
team_ws/src/pathfinding/       THIS folder
```

**Build** (once, and again after changing `package.xml`, `setup.py` or a launch file). Python
edits need no rebuild:

```bash
W=/var/lib/flyx-lab/workspace
SB=$W/skybeta-main                       # your Sky Beta folder name
cd $SB && colcon build --packages-select skybeta_flight_interface oakd_vslam_bringup || exit
source $SB/install/setup.bash
cd $W/team_ws && colcon build --symlink-install || exit
source $W/team_ws/install/setup.bash
ros2 pkg executables dronehacks_pathplan        # 6 executables
```

Every new terminal needs both `source` lines.

**Check, then fly.** Reset world first.

```bash
ros2 run dronehacks_pathplan check_pipeline                 # read-only, 15 s: fix every FAIL
ros2 launch dronehacks_pathplan sim.launch.py shadow:=true pilot:=none   # plan only (Ctrl-C after ~20 s)
# team approves the flight, Reset world, then:
ros2 launch dronehacks_pathplan sim.launch.py               # flies; the sim pilot arms and lands
ros2 run dronehacks_pathplan report                          # second terminal, after the flight
```

What `sim.launch.py` does:

- **Starts the drone launch** with `use_sim_time:=true` for our node (the run limit is in sim
  time), `flight_interface` on the wall clock (`interface_use_sim_time:=false`: its Offboard
  heartbeat stamps are what PX4 checks; sim-time stamps read as stale) and
  `allow_mode_request:=true` (sim only).
- **Starts the sim pilot**, which does what the RC pilot does:
  - `pilot:=drone_like` (default): Altitude → arm → Position. This exercises the real
    auto-Offboard path.
  - If the sim refuses Altitude without RC, use `pilot:=direct`: it requests Offboard, then arms.
  - `pilot:=none`: you fly the pilot part with keyboard RC: arm in Altitude, switch to
    Position, and land at `LOW_HOLD`.
- **Starts no vision bridge** (`vision_bridge:=false`): the lab bridges vision itself. Use
  `vision_bridge:=true` only for a simulator that has none; two bridges make PX4 fuse every
  sample twice, and our node then refuses READY.
- **Depth:** `depth_topic` defaults to the lab's `/camera/depth/image_raw`. Pass
  `depth_topic:=''` to fly like the stock drone, on the published map only.

**Pass criteria** for a lab flight. The node prints a summary, and `report` repeats it:

- `RESULT: over the pad (… m from its centre)`, with checkpoints cleared 5/5
- states `READY -> ARMED -> TAKEOFF -> NAVIGATE -> PAD -> DESCEND -> LOW_HOLD -> TAKEN_OVER -> DONE`.
  The `TAKEN_OVER` here is the pilot's Land.
- `odom vs PX4 local position gap` stays small (< 0.15 m)
- no `FAULT`, and control-loop stalls far below 1 s
- depth: most hits on published walls. A low share means a wrong camera mount or frame.

---

## 5. On the drone

1. **Copy this folder to `~/team_ws/src/pathfinding`.** Copy only this folder, never the
   whole repository: a `skybeta/` copy inside `team_ws` would replace the organisers'
   packages.
2. **Edit `/etc/skybeta/drone.env`.** Change only these two lines, and touch nothing else in
   it (DRONE_ID, ROS_DOMAIN_ID, TEAM):
   ```
   COMPETITOR_PKG=dronehacks_pathplan
   COMPETITOR_LAUNCH=pathplan.launch.py
   ```
3. **Restart the stack, on the ground, drone still:** `docker rm -f skybeta-stack`. It returns
   in about 10 s. The competitor window runs `team_build.sh`, then
   `ros2 launch dronehacks_pathplan pathplan.launch.py`.
4. **Watch it:** `docker exec -it skybeta-stack tmux attach -t skybeta`, then the competitor
   window. Or from a container shell (`host/run.sh shell`):
   ```bash
   ros2 topic echo /dronehacks_pathplan/status        # state, note, cleared
   ros2 run dronehacks_pathplan check_pipeline --depth-topic ''
   ```
5. **Flight:**
   1. Put the drone on the launch mark, facing up the first corridor.
   2. Wait for `"state": "READY"`.
   3. The pilot arms in Altitude, then switches to Position.
   4. The drone takes off and flies.
   5. At `LOW_HOLD` ("PILOT: LAND NOW"), the pilot lands.
6. **Afterwards:** `ros2 run dronehacks_pathplan report`. The journal is in
   `~/team_ws/runs/pathplan-*/`.

The stock drone has **no depth topic**, so the node flies the published map only. If you
enable OAK depth, pass `depth_topic:=<topic>` and set `camera_offset_flu_m` to
`CAMERA_X/Y/Z` from `drone.env`.

The organisers' guide says nothing about takeoff under Offboard is proven on any airframe yet.
Your first runs are that test:

- **Props off first:** arm + Position must show `TAKEOFF` in the status, and the setpoints in
  `/skybeta/setpoint/pose` must be sane.
- **Then low and tethered.**
- Record a bag every flight (`host/run.sh bag`).

---

## 6. How to know it works: four levels, cheapest first

| Level | Command | Needs | What it proves | Our result |
|---|---|---|---|---|
| 1. logic | `python3 -m pytest -q tests` (in this folder) | Python + numpy + yaml | state machine, frames, ramps, heading limit, faults, planner; also on Python 3.10 / numpy 1.26 / no scipy (= the drone) | 152 passed (3.10: 146 + 1 skipped) |
| 1b. closed loop | `python3 -m dronehacks_pathplan.dry_run` (+ `--no-depth`, `--dropout 30 1`, `--dropout 30 4`, `--carry 0.6 -0.3 25`, `--takeover 20`) | same | the mission + real planner fly the course through a model of `flight_interface` / PX4 / pilot | PASS: 5/5, landed ≤ 0.01 m from the pad, ~70 s, heading ≤ 0.25 rad/s, closest wall 0.37 m |
| 2. ROS pipeline | `ros2 launch dronehacks_pathplan selftest.launch.py` (also `pilot:=direct depth:=false`) | built workspace, **no simulator** | real `flight_interface` + our node + sim pilot vs a fake PX4: topic versions, QoS, frames, the arm → Position → auto-Offboard handshake, LOW_HOLD → Land → DONE | PASS: 5/5, landed 0.00 m from the pad, ends by itself in ~95 s; also with a 0.6× slow sim clock |
| 3. simulator | `check_pipeline`, then `sim.launch.py shadow:=true`, then `sim.launch.py` | FlyX lab | real cuVSLAM, real PX4, real depth, sim time | **your next step** |
| 4. drone | section 5: props off, then tethered | the drone | everything | after 3 |

Levels 1–2 take 2 minutes. **Run them after every change**, before spending lab GPU time.
Don't run the self-test beside a simulator or on the drone: its fake PX4 would answer
alongside the real one.

---

## 7. Parameters (launch arguments of the same name)

| Argument | Default | Meaning |
|---|---|---|
| `shadow` | `false` | plan and log only |
| `depth_topic` / `camera_info_topic` | `''` (sim.launch: `/camera/depth/image_raw`) | depth image; empty = published map only |
| `camera_offset_flu_m` | `[0.115, 0.0, 0.029]` | depth camera in body FLU (Sky Beta CAD; on the drone `CAMERA_X/Y/Z`) |
| `flight_altitude_m` | `0.9` | cruise height above the floor (walls 1.73 m) |
| `carrot_m` | `0.5` | how far ahead the setpoint is. Pose setpoints are not speed-capped, so this sets the speed (~0.5 m/s). |
| `yaw_rate_max` | `0.25` | rad/s heading limit. FlyX data: 30 of 40 VIO losses happened above 0.35 rad/s. |
| `planner` | `theta` | `theta`, `astar`, or `static` (no sensing) |
| `launch_xy_m`, `launch_heading_course_rad`, `launch_height_m` | `[-2.515, 0.914]`, `1.5708`, `0.05` | the launch mark in the course frame |
| `frame_latch` | `arm` | `arm` or `slam_origin` (section 3) |
| `max_flight_s` | `175.0` | run limit from takeoff (rule: 3 min) |
| `min_altitude_m`, `max_altitude_m`, `max_speed_mps`, `auto_offboard_on_arm`, `allow_mode_request` | as Sky Beta | `flight_interface`'s own; `min_altitude_m` is shared with our node |
| `use_sim_time` | `false` (sim.launch: `true`) | our node |
| `interface_use_sim_time` | `false` | `flight_interface`; keep false (drone and sims): `check_pipeline` checks its Offboard stamps against PX4's clock |
| `planner_process` | `true` | planner in its own process (keeps the 20 Hz setpoint stream stall-free) |

Mission constants (climb/descent rates, tolerances, freshness limits) are `MissionConfig` in
`dronehacks_pathplan/mission.py`.

---

## 8. Troubleshooting

| Symptom | Look at |
|---|---|
| Status stuck in `WAIT_HEALTH` | its `note` names the failed check (no odometry / no `/skybeta/state` / no PX4 position / not landed / another commander) |
| Armed + Position, no takeoff | `flight_interface` log: `auto offboard: staying out of Offboard: <reason>`. "no team setpoint received" means our node was not READY (or `shadow`). |
| Flies to the wrong place | `check_pipeline`: the `odom_enu vs local_enu` gap. If PX4's frame differs from cuVSLAM's, see Sky Beta `anchor_setpoints` / `odom_to_ned_yaw_offset_rad`. Also: was the drone on the mark facing +y when armed? |
| Hangs at a corner | `report`: CLAMPED in the PX4 lines, planner `turn`/`stuck` events |
| `FAULT: odometry lost` | cuVSLAM lost tracking (`/visual_slam/status`): too fast a turn, blank wall, bad light |
| `offboard_control_signal_lost` in PX4 (sim) | stalls in `report`; `check_pipeline`'s "Offboard stamps vs PX4 time" line (the interface must be on the wall clock). |
| Lab: two vision bridges | `check_pipeline` FAIL on `/fmu/in/vehicle_visual_odometry`: run `sim.launch.py vision_bridge:=false` |

---

## 9. Offline tools (unchanged)

- `python3 -m pathplan.harness --variants 4 --output runs/bench`: offline benchmark on 13
  perturbed courses
- `bash gzsim/run.sh`: the planner alone in Gazebo with ground-truth pose
- `requirements.txt`: what a laptop venv needs (numpy, scipy, pyyaml, matplotlib, pytest)

## 10. Not yet verified

These need the lab, then the drone:

- real cuVSLAM drift and tracking loss
- real depth noise
- the lab PX4's acceptance of `flight_interface`'s timestamps under sim time
- whether PX4's local frame on the drone matches the vision frame (Sky Beta's own open caveat)
- takeoff under Offboard on the real airframe
