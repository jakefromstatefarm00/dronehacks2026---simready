# team_mission — our replacement for `square_mission`

`team_mission` is a ROS 2 package that goes in **`~/team_ws/src/team_mission`** on the drone. It
replaces the organizers' demo node (`competitor_example/square_mission.py`) with our own mission.
Here is what it does, in order:

1. It stays dormant until the pilot arms.
2. When the pilot switches to Position, it takes off to **1 m** (measured by cuVSLAM z).
3. A **trainable policy** flies it **slowly** through a list of checkpoints.
4. At the last checkpoint it sinks to a low hover, touches down, and **PX4 disarms itself**.

The package also contains everything needed to train and improve that policy:

- a training environment
- PPO training
- export
- evaluation
- a full-mission simulator
- a tool that measures the real drone from its flight logs

It also contains two simulation-only helpers for Gazebo and the flyx lab.

> **Status, honestly.** Everything below the ROS layer has been run and tested here:
> - 22 unit tests pass.
> - Whole simulated missions run from arm to disarm.
> - The training pipeline ran end to end (BC + PPO + export + evaluation). No trained policy
>   ships: you train your own (section 7), and `policy:=carrot` flies without one.
>
> The ROS node was exercised through a full simulated flight, using stand-ins for ROS. However,
> **it has not yet run on real ROS 2, in Gazebo, or on the drone.** Expect small fixes the first
> time: topic names, QoS, parameter types. Run it **props off** first (section 6).

---

## Contents

1. [Where it fits in the x.zip stack](#1-where-it-fits-in-the-xzip-stack)
2. [The flight, step by step — what happens and why](#2-the-flight-step-by-step)
3. [Files](#3-files)
4. [The policy contract: what it sees, what it outputs, how that flies](#4-the-policy-contract)
5. [Safety layers](#5-safety-layers)
6. [On the drone: install, bench test, first flights](#6-on-the-drone)
7. [Training and refining the policy (RL)](#7-training-and-refining-the-policy)
8. [Verify 1: our own simulator (no ROS)](#8-verify-1-our-own-simulator-no-ros)
9. [Verify 2: Gazebo (on your laptop)](#9-verify-2-gazebo)
10. [Verify 3: the flyx lab](#10-verify-3-the-flyx-lab)
11. [Troubleshooting](#11-troubleshooting)
12. [Design decisions and their reasons](#12-design-decisions)

---

## 1. Where it fits in the x.zip stack

Nothing in `~/skybeta` changes. The organizers' stack keeps running exactly as it does with
x.zip. The only change is **which package the `competitor` tmux window launches**.

```
power on ─► skybeta-stack container (from x.zip, unchanged)
            ├─ agent       MicroXRCEAgent  (ROS <-> Pixhawk serial)
            ├─ slam        OAK-D driver -> grey images -> cuVSLAM (visual_slam_node)
            │                 publishes /visual_slam/tracking/odometry, /visual_slam/status
            ├─ bridge      px4_odom_bridge -> /fmu/in/vehicle_visual_odometry -> PX4 EKF2
            └─ competitor  launches $COMPETITOR_PKG $COMPETITOR_LAUNCH
                             x.zip default: competitor_example competitor.launch.py
                                            = flight_interface + square_mission
                             ours:          team_mission mission.launch.py
                                            = flight_interface (same settings) + team_mission
```

`team_mission` uses the same contract as the example. These are the only topics it touches:

| Direction | Topic | Type | Why |
|---|---|---|---|
| in | `/skybeta/state` | `std_msgs/String` (JSON) | armed? Offboard active? landed? (from flight_interface) |
| in | `/visual_slam/tracking/odometry` | `nav_msgs/Odometry` | where we are (cuVSLAM, `odom` frame, ENU) |
| in | `/visual_slam/status` | `isaac_ros_visual_slam_interfaces/VisualSlamStatus` | is cuVSLAM tracking (`vo_state == 1`) |
| in (optional) | e.g. `/team/localization/ok` | `std_msgs/Bool` | extra health from our `team_localization` package |
| out | `/skybeta/setpoint/pose` | `geometry_msgs/PoseStamped` | hold, takeoff and flight setpoints |
| out | `/skybeta/setpoint/twist` | `geometry_msgs/TwistStamped` | the final slow descent only |
| out | `/team_mission/status` | `std_msgs/String` (JSON, 2 Hz) | phase, checkpoint, hold reason, log file |

It **never arms**, never changes PX4 mode and never publishes to `/fmu/in/*`. Those are organizer
rules, and `flight_interface` owns all of it. The node refuses to start if a setpoint topic is
pointed at `/fmu/`.

---

## 2. The flight, step by step

Your six steps, mapped to what the code does. The phases are in `team_mission/mission.py`.

| Your step | Phase | What happens | Why it is done this way |
|---|---|---|---|
| 1. Power on, cuVSLAM fixes zero | (not ours) | The stack starts itself. cuVSLAM's `odom` origin is where base_link is at boot, and +x is the nose direction then. **Keep the drone still for the first minute.** | x.zip behaviour; we do not touch it. |
| 2. Measurements as usual | (not ours) | cuVSLAM → bridge → EKF2, about 30 Hz. The flight interface publishes `/skybeta/state` at 5 Hz. | Unchanged. |
| 3. Dormant until the pilot arms in Altitude | `DORMANT` | Publishes **nothing**. It waits for `/skybeta/state` → `armed: true`. If the pilot armed in another mode it logs a warning, because the procedure is to arm in Altitude. | You asked for dormant. Publishing nothing means nothing reaches PX4 before the pilot acts. |
| (between 3 and 4) | `ARMED_WAIT` | From the moment of arming it streams "stay where you are" pose setpoints at 20 Hz. | `flight_interface` only asks PX4 for Offboard if a team setpoint arrived **in the last second** (its auto-offboard rule). So the node must wake when the pilot arms, before the switch to Position. "Where you are" is safe: PX4 ignores it until Offboard starts. |
| 4. Pilot flips to Position → flight interface checks → takeoff to 1 m | `TAKEOFF` | When `offboard_active` turns true, the node records the start pose (position + yaw) and builds the path. The z target then rises at **0.3 m/s** from the ground to **start z + 1.0 m**. When it is within 8 cm for 1.5 s, it moves on. | A rising target (not a 1 m jump) keeps the climb slow, because PX4 would otherwise climb at its own maximum. Note: the first setpoints are floored by `flight_interface` at 0.3 m above the boot origin, so the drone lifts to 0.3 m almost at once. Remember the 10 s rule: PX4 disarms if it has not left the ground 10 s after arming (`COM_DISARM_PRFLT`). |
| 5. The policy flies to the checkpoints, slowly | `FLY` | Every 50 ms: build the 20-number observation → policy → 4-number action → a **pose setpoint at most 0.6 m ahead** (the "carrot") and a yaw target. A checkpoint counts as reached inside its radius, or when slightly past it and close sideways. | Section 4 explains the contract and why it is a pose and not a velocity. The policy can be the hand-written `carrot` baseline or a trained network (`policies/*.npz`). |
| (if something goes wrong) | `HOLD` | Health lost, too far off the path, or a bad policy output → hold position. For "off the path", it holds on the nearest point of the path, so it creeps back. It resumes after 2 s healthy, and gives up after 20 s, or after more than 4 corridor holds (→ `ABORTED`). | A slow, explainable fallback, so the pilot always knows what it is doing. |
| 6a. Hover close to the ground | `DESCEND` | At the last checkpoint: once slower than 0.15 m/s there, the hover target moves down at 0.2 m/s to **0.3 m above the start ground**. It holds there for 1.5 s. | 0.3 m is `flight_interface`'s floor for pose setpoints, so it is the lowest hover we can command. |
| 6b. Then disarm | `TOUCHDOWN` → `DONE` | It sends a **0.2 m/s downward twist**, refreshed at 20 Hz. The drone settles, PX4's land detector reports landed, and PX4 **disarms itself** after `COM_DISARM_LAND` (2 s in the fleet parameter files in x.zip). On disarm the node goes back to `DORMANT`, ready for the next flight. If PX4 has not disarmed 8 s after landing, it logs an error asking the pilot to disarm. | **Why not "PX4 land/disarm commands"?** Those are `VEHICLE_CMD_NAV_LAND` and `COMPONENT_ARM_DISARM` on `/fmu/in/vehicle_command`, and the organizer rules forbid publishing to `/fmu/in/*`. `flight_interface` deliberately has no land or disarm command. A twist setpoint is the one thing it does not floor at 0.3 m, so a slow downward twist plus PX4's own auto-disarm is the compliant way to finish. **Ask the organizers to confirm this is how they expect an autonomous landing.** |
| anywhere | `ABORTED` | `offboard_active` went false: the pilot moved a stick or flipped the switch. The node publishes nothing and waits for disarm. | Never fight the pilot. One Offboard attempt per arming is also a flight_interface rule. |
| anywhere | → `DORMANT` | Disarm. The flight log is closed. | Ready for the next flight. |

**Speed.** Every phase is slow:
- climb 0.3 m/s
- flight at most 0.4 m/s, and only 0.24–0.27 m/s in the tight zone-1 corridors and the window (section 4)
- yaw at most 0.5 rad/s
- descent 0.2 m/s

In simulation a full course lap takes about 90 s from arm to disarm (60–115 s depending on the
randomised drone).

---

## 3. Files

```
team_ws/src/team_mission/
├── package.xml, setup.py, setup.cfg, resource/team_mission   ROS 2 (ament_python) package files
├── README.md                                   this file
├── requirements-train.txt                      laptop-only Python deps for training
├── config/
│   ├── mission.yaml                            every node parameter (speeds, heights, topics, launch pose)
│   ├── dynamics_default.yaml                   the training randomisation ranges (copy + edit)
│   └── checkpoints/
│       ├── test_line.yaml                      FIRST test: 1 m forward and back (relative to takeoff)
│       ├── test_square.yaml                    SECOND test: 1 m square to the left
│       └── course_v3.yaml                      the DroneHacks course, launch mark -> pad (course frame)
├── launch/
│   ├── mission.launch.py                       on the drone: flight_interface + team_mission
│   └── sim.launch.py                           SIMULATION: + sim_pilot (+ sim_odom_adapter)
├── policies/                                   your trained networks (.npz); empty until you train one
├── team_mission/
│   ├── geometry.py                             frames: odom (ENU), heading frame, course <-> odom
│   ├── path.py                                 checkpoint files, path geometry (progress, off-path, reached)
│   ├── control.py                              THE CONTRACT: observation, action -> setpoint, baseline + network policies
│   ├── mission.py                              the state machine above (ROS-free, unit tested)
│   ├── mission_node.py                         the ROS 2 node (topics, parameters, logging)
│   ├── flight_log.py                           one CSV + JSON per flight
│   ├── rl/                                     LAPTOP ONLY (not used by the drone node)
│   │   ├── dynamics.py                         randomised model of PX4 position control + cuVSLAM errors
│   │   ├── env.py                              gymnasium environment: fly the checkpoints
│   │   ├── train.py                            behaviour cloning + PPO, evaluation, export
│   │   ├── export.py                           SB3 model -> policies/*.npz (numpy, checked against torch)
│   │   ├── evaluate.py                         success rate / time / clearance, plots
│   │   ├── mission_sim.py                      the WHOLE mission (arm -> disarm) in the simulator
│   │   ├── fit_dynamics.py                     measure the real drone from flight logs -> dynamics YAML
│   │   ├── check_course.py                     check a checkpoint file against the course walls
│   │   └── course_collision.py                 optional: use the repo's course model (drl/) for walls
│   └── sim/                                    SIMULATION ONLY: refuse to start on the drone
│       ├── sim_pilot.py                        plays the safety pilot (arm, hand over)
│       └── sim_odom_adapter.py                 stands in for cuVSLAM: PX4 odometry -> /visual_slam/tracking/odometry
└── test/                                       pytest: python3 -m pytest test -q
```

Only `mission_node`, `sim_pilot` and `sim_odom_adapter` are ROS executables. The drone node
imports `geometry`, `path`, `control`, `mission` and `flight_log`, which need only numpy and
PyYAML. Both are already in the skybeta image. **No torch, no gymnasium, no extra installs on the
drone.**

---

## 4. The policy contract

`team_mission/control.py` is the single place that defines what the policy sees and does. The
training environment and the drone node both call the same two functions, so a policy sees the
same numbers in both. If you change anything there, bump `OBS_VERSION`, because old `.npz`
files then refuse to load. The current version is `2`.

**Frames.** Everything is in cuVSLAM's `odom` frame (ENU). Observations and actions use the
*heading frame*: level, x = where the drone's nose points. A policy therefore never knows or
cares which way the course is built.

**Observation (20 numbers, roughly −1..1)**

| Index | What | Scale |
|---|---|---|
| 0–2 | next checkpoint relative to the drone (heading frame) | clipped to 2 m, ÷2 |
| 3–5 | the checkpoint after that (the same one at the last checkpoint) | clipped to 2 m, ÷2 |
| 6–8 | velocity (heading frame) | ÷0.5 m/s, clipped ±2 |
| 9–10 | sin, cos of the angle between the nose and the direction to the next checkpoint | — |
| 11–13 | sideways offset from the current path segment (heading frame) | ÷0.5 m, clipped ±2 |
| 14–17 | previous action | — |
| 18 | 1 if the next checkpoint is the last (landing) one | — |
| 19 | corridor half-width of this segment | ÷0.5 m |

**Action (4 numbers, −1..1)**

| Index | Meaning | Times |
|---|---|---|
| 0, 1 | forward and left velocity | `max_xy_speed` (0.40 m/s) × `speed_scale` |
| 2 | up velocity | `max_z_speed` (0.25 m/s) |
| 3 | yaw rate (counter-clockwise) | `max_yaw_rate` (0.5 rad/s) |

**`speed_scale`** = clip(corridor / 0.30 m, 0.5, 1.0). The zone-1 maze (0.20–0.22 m corridors)
gets about 0.67–0.73 of full speed, and the window (0.18 m) gets 0.6. Tight places are
automatically slower.

**How an action is flown.** The requested velocity becomes a **pose setpoint** ahead of the drone:

```
carrot = position + velocity × lead_time (1.0 s)        capped at max_carrot_m (0.6 m) from the drone
yaw    = current yaw + yaw_rate × yaw_lead_s (0.6 s)
```

PX4's position controller then flies to the carrot. Its velocity is about `MPC_XY_P × distance`,
and with `lead_time ≈ 1 / MPC_XY_P` that is the speed the policy asked for.

**Why a pose and not a twist:** if our node crashes or stalls, `flight_interface` keeps holding
the **last pose** it got, which is at most 0.6 m ahead of the drone. A stale twist instead
expires after 1 s and falls back to the **last pose** we sent, which during a twist-only flight
would be the takeoff point, possibly metres behind us through walls.

**Policies available**

| `policy:=` | What |
|---|---|
| `carrot` | Hand-written pure-pursuit baseline. Needs no training. It slows for corners in proportion to their sharpness, stops at the last checkpoint and faces the direction of travel (the camera needs to see where it is going). Use it for the first flights. |
| your own `.npz` | From `python -m team_mission.rl.export`. The node checks its contract version on load. |

A network may also be **residual** (`--residual-scale` at training): action = carrot + scale ×
network. It starts as the baseline and only learns corrections.

---

## 5. Safety layers

From the inside out:

1. **The action is bounded:**
   - every action is clipped to [−1, 1];
   - speeds are capped by `Limits`;
   - the carrot is at most 0.6 m from the drone;
   - NaN or Inf from the policy → HOLD → ABORTED.
2. **Health gate** (every tick). If any of these fails → HOLD:
   - odometry not older than 0.3 s and finite;
   - `vo_state == 1` (`require_vslam_status`);
   - optional extra health topic true.
3. **Corridor:**
   - off the path by more than the segment corridor + 5 cm → HOLD on the path;
   - more than 4 such holds → ABORTED.
4. **Timeouts:**
   - HOLD for more than 20 s → ABORTED;
   - the FLY phase longer than 300 s → ABORTED.
5. **flight_interface (x.zip):**
   - altitude 0.3–3.0 m above the boot origin;
   - twist ≤ 1.5 m/s;
   - Offboard only on the pilot's handshake, once per arming;
   - holds the last pose if we go quiet.
6. **PX4 and the pilot:**
   - the RC sticks or mode switch take over at any time;
   - kill switch;
   - `COM_DISARM_PRFLT`;
   - failsafes.

What is **not** protected: cuVSLAM **drift**. If cuVSLAM believes it is on the path while it has
drifted 15 cm, layers 1–4 believe it too. In simulation, drift and air pushes, not the policy,
cause most touches in the zone-1 maze (about 0.2 m of free space each side). This is where
`team_localization` (health cross-checks) and future in-run course alignment come in.

---

## 6. On the drone

### 6.1 Put it in place and build it

Do this on the drone over SSH, with the stack running. Clone the team repo inside `~/team_ws/src`
(a symlink from elsewhere does not work: only `~/team_ws` is mounted into the container). If the
repo layout puts this package somewhere else, copy just the `team_mission` folder to
`~/team_ws/src/team_mission`.

```bash
~/skybeta/host/run.sh shell                       # a shell inside the running container
team_build.sh --packages-select team_mission      # builds the team overlay
source skybeta_env.sh
python3 -m pytest /root/team_ws/src/team_mission/test -q   # optional: the tests run in the container too
```

### 6.2 Bench test (PROPS OFF)

First stop the example mission. Two nodes publishing to `/skybeta/setpoint/pose` must never run
together. Either switch the stack to our package (section 6.4) and restart it, or run by hand:
attach to the tmux session (`docker exec -it skybeta-stack tmux attach -t skybeta`), Ctrl-C the
`competitor` window, then in that window:

```bash
ros2 launch team_mission mission.launch.py checkpoints:=test_line.yaml policy:=carrot
```

The expected log, in order:

| You do | You should see |
|---|---|
| nothing | `team_mission up: 2 checkpoints (start frame) ... DORMANT until the pilot arms.` |
| `ros2 topic echo /team_mission/status` | `"phase": "DORMANT"` |
| pilot arms (Altitude, throttle low, props off) | `DORMANT -> ARMED_WAIT: armed`, `logging this flight to /root/team_ws/flight_logs/flight_....csv` |
| pilot switches to Position | `ARMED_WAIT -> TAKEOFF: start at (...)`. The motors spin up (props off!). The z target starts rising. With props off it never climbs, so it stays in TAKEOFF. |
| wait 10 s | PX4 disarms by itself (`COM_DISARM_PRFLT`, it never took off) → `disarmed in TAKEOFF -> DORMANT` |
| cover the camera while armed in TAKEOFF | `TAKEOFF -> HOLD: cuVSLAM not tracking (vo_state=2)` |
| move a stick while in TAKEOFF | `-> ABORTED: Offboard lost - the pilot has control` |

That checks the handshake, the health gate, pilot takeover and logging.

### 6.3 First flights (props on, safety pilot ready, open space)

1. `checkpoints:=test_line.yaml`: up 1 m, 1 m forward, back, land. Needs 1.5 m clear in front.
2. `checkpoints:=test_square.yaml`: a 1 m square to the left, then land. Checks turns and yaw.
3. `checkpoints:=course_v3.yaml`, only after the launch pose is measured (6.5). Start with
   `policy:=carrot`.

After each flight, copy the log off the drone. It is in `~/team_ws/flight_logs/` on the host.
`/team_mission/status` shows the phase, the next checkpoint and the distance to it live.

### 6.4 Make it start with the stack

In `/etc/skybeta/drone.env` (do not touch `DRONE_ID`, `TEAM` or `ROS_DOMAIN_ID`):

```
COMPETITOR_PKG=team_mission
COMPETITOR_LAUNCH=mission.launch.py
```

Restart the stack. The `competitor` window now runs `flight_interface` plus `team_mission`. To
change checkpoints or the policy without arguments, edit `config/mission.yaml` and run
`team_build.sh`.

### 6.5 Course coordinates (the launch pose)

`frame: course` checkpoint files are in published-layout metres. The node converts them using
where base_link was **at boot** (`launch_x/y/z`, `launch_yaw_deg` in `mission.yaml`). The default
is the course v3 launch mark, nose pointing up the first guideline leg (course +y):
`(-2.515, 0.914, 0.0, 90°)`.

If the drone boots somewhere else, or facing elsewhere, everything is shifted or rotated. So:

- Boot it **on the launch mark, facing the right way, and keep it still**.
- Measure where base_link is relative to the mark. A few cm matter: zone 1 has about 0.2 m spare
  each side.

`frame: start` files (the tests) need none of this. They are relative to wherever the drone
takes off.

---

## 7. Training and refining the policy

This runs on your laptop, never on the drone.

### 7.0 How training and Gazebo fit together

**You do not train inside Gazebo.** Gazebo with PX4 SITL runs at about real time. One course
episode takes about 90 s, and PPO needs hundreds of thousands of episodes. The fast simulator in
`team_mission/rl/` runs thousands of times faster. Gazebo's job is to keep that fast simulator
honest and to check every policy before the drone sees it:

```
 1. train in the fast simulator            python -m team_mission.rl.train --out runs/v1          (7.3)
 2. compare with the baseline              python -m team_mission.rl.evaluate runs/v1/policy.npz   (7.4)
 3. whole mission, no ROS                  python -m team_mission.rl.mission_sim --policy ...     (8)
 4. fly it in Gazebo (real PX4 + flight_interface + this node)                                    (9)
        -> writes ~/team_mission_logs/flight_*.csv
 5. measure Gazebo's drone from those logs python -m team_mission.rl.fit_dynamics <logs> --out gz.yaml  (7.6)
 6. retrain / fine-tune with those ranges  python -m team_mission.rl.train --dynamics gz.yaml --resume runs/v1/best_model.zip --out runs/v2
 7. repeat 2-6 until it beats `carrot` in Gazebo too; then the drone (6.3) and repeat 5-6 with real logs
```

Steps 4–6 are the "training with Gazebo" loop. Gazebo flights give you the real PX4 response:
gains, lag, delay. The fast simulator then trains against that. Start every Gazebo session with
`policy:=carrot`, so you have a baseline log to compare against.

### 7.1 Install

```bash
cd team_ws/src/team_mission
python3 -m venv .venv && source .venv/bin/activate          # Windows: .venv\Scripts\activate
pip install -r requirements-train.txt
python -m pytest test -q                                    # 22 tests should pass
```

### 7.2 What the environment is (`team_mission/rl/env.py`)

One episode is the `FLY` phase:
- **Start:** hovering at the takeoff point, or part-way along the path (50% of episodes, a
  curriculum that practises every segment).
- **Task:** visit the checkpoints in order and stop in a slow hover inside the last one's radius.
- **Not learned:** takeoff and landing. The state machine does those, and the same code runs them
  in `mission_sim.py` (section 8).

**Paths.** Half the episodes use the checkpoint file you train for (default `course_v3.yaml`),
with every checkpoint jittered by about 5 cm, because the real course is never exactly the
published one. The other half use **random zig-zags** (3–8 checkpoints, 0.6–2.5 m legs, turns
up to 120°, random corridors). The policy learns to follow *any* checkpoint list rather than
memorising one.

**The drone model** (`dynamics.py`) is not rotor physics. We never command motors: we send a pose
and PX4 flies to it. The model is PX4's cascade:
- position P gain;
- velocity time constant and integral;
- acceleration limit;
- yaw gain;
- link delay;
- cuVSLAM's errors: noise, drift that grows with distance, and yaw drift;
- a slowly changing push (air).

**Every one of these is drawn from a range each episode** (`config/dynamics_default.yaml`). The
policy has to fly any drone in the ranges, which is how it survives the gap to reality.

**Rewards** (weights in `env.py`, override with `CheckpointEnv(reward={...})`):

| Term | Weight | Purpose |
|---|---|---|
| progress | +2 per metre along the path | the main drive. Only forward progress pays, so hovering earns nothing. |
| checkpoint | +1 each | milestones |
| finish | +5 | slow hover inside the last checkpoint's radius |
| crash | −10, episode ends | left the corridor, altitude outside 0.25–1.8 m, or (with `--course-obj`) hit a wall |
| lateral | −0.05 × (offset / corridor)² per step | stay near the line, more so where it is tight |
| smooth | −0.02 × \|Δaction\|² | no jitter, which cuVSLAM and the motors both prefer |
| heading | −0.01 × (1 − cos(nose vs direction of travel)) | keep the camera looking where it goes |
| overspeed | −0.5 × excess above 1.25 × limit | the action cannot ask for more, but momentum can overshoot |
| time | −0.005 per step | do not dawdle (small: slow is fine) |

Episodes time out after (path length / 0.12 m/s + 20 s).

### 7.3 Train

```bash
python -m team_mission.rl.train --out runs/v1                               # ~1.2M steps
python -m team_mission.rl.train --out runs/v1 --steps 3e6 --n-envs 16       # longer, on a bigger machine
```

What it does:

1. **Behaviour cloning.** The network first copies the `carrot` baseline (300 noisy episodes),
   so PPO starts from something that already flies.
2. **PPO.** Default settings:

   | Setting | Value |
   |---|---|
   | actor | 2×64, tanh |
   | critic | 2×128 |
   | learning rate | 1e-4 |
   | n_steps | 512 × n_envs |
   | batch | 1024 |
   | γ | 0.99 |
   | GAE λ | 0.95 |
   | clip | 0.2 |

3. **Evaluation every 100k steps** on 60 fixed randomised course episodes. The best (by success
   rate, then time) is exported as `policy.npz`.

Outputs in `runs/v1/`:

| File | What it is |
|---|---|
| `policy.npz` | best; this is what you deploy |
| `policy_last.npz` | latest |
| `best_model.zip`, `model.zip` | SB3 models, for `--resume` |
| `eval.csv` | success and time per evaluation |
| `progress.csv` | PPO internals |
| `config.json` | every argument, for reproducing the run |

Useful options:

| Option | Use it when |
|---|---|
| `--resume runs/v1/best_model.zip` | continue training (e.g. with new dynamics ranges) |
| `--dynamics measured.yaml` | train for the drone you measured (7.6) |
| `--course-obj course/dronehacks-course-v3.obj` + `DRONEHACKS_REPO=<repo root>` | crash on the real walls, not only the corridor |
| `--residual-scale 0.3` | learn a correction on top of `carrot` (safer, starts at the baseline) |
| `--random-path-prob 0.8` | generalise more (other courses) / `0.2`: specialise on this one |
| `--checkpoints my_route.yaml` | train for another route |
| `--limits config/mission.yaml` | train with the speeds in the mission config (keep them identical!) |

### 7.4 Evaluate (always against the baseline, same seed)

```bash
python -m team_mission.rl.evaluate carrot --episodes 200
python -m team_mission.rl.evaluate runs/v1/policy.npz --episodes 200 --plot v1.png
python -m team_mission.rl.evaluate runs/v1/policy.npz --nominal          # perfect drone: is the line itself good?
python -m team_mission.rl.evaluate runs/v1/policy.npz --checkpoints config/checkpoints/test_square.yaml
```

The output reports:

- `success_rate`
- `mean_time_s` (successful episodes)
- `p95_max_lateral_m` (how close to the corridor edge)
- `reasons`
- `failed_heading_for` (which checkpoint the failures were heading for)

The same `--seed` gives both policies identical drones, noise and starts.

### 7.5 Deploy a policy

```bash
python -m team_mission.rl.export runs/v1/best_model.zip policies/v1.npz   # also checks numpy == torch
python -m team_mission.rl.mission_sim --checkpoints course_v3.yaml --policy policies/v1.npz --runs 50
```

Then commit `policies/v1.npz`. On the drone, run `team_build.sh` and set `policy:=v1.npz` (or set
it in `mission.yaml`). Fly `test_line` and `test_square` with every new policy before the course.

### 7.6 The refinement loop

```
 fly (sim / Gazebo / drone)  ──►  flight_logs/*.csv  ──►  fit_dynamics  ──►  measured.yaml
        ▲                                                                       │
        │                                                                       ▼
   mission_sim + Gazebo  ◄──  evaluate vs carrot  ◄──  train --dynamics measured.yaml --resume
```

`python -m team_mission.rl.fit_dynamics flight_logs/*.csv --out measured.yaml` uses the FLY and
TAKEOFF rows of real logs (the carrot we sent vs. what cuVSLAM saw). It fits PX4's gain, the
velocity time constant, the link delay and the yaw gain, then writes ranges of ±25% around them.
Treat the numbers as rough: from a simulated flight it recovered the gains to within about 10%,
but split lag between `tau` and `delay` differently from the truth. The fit is good enough to
centre the ranges, not to replace them.

**Things to change, and where:**

| Goal | Change |
|---|---|
| fewer corridor crashes | stronger `lateral` weight; more `mid_start_prob`; train with `--course-obj` |
| smoother flying | stronger `smooth` weight; a smaller `max_yaw_rate` in Limits (then retrain) |
| more robust to bad tracking | widen `drift_per_sqrt_m`, `pos_noise`, `delay_steps` in a dynamics YAML |
| a bit faster later | raise `max_xy_speed` in `mission.yaml` **and** retrain with `--limits config/mission.yaml` |
| a better line through corners | edit `config/checkpoints/course_v3.yaml`, check it with `check_course`, retrain |
| a new observation (e.g. depth / range) | add it in `control.build_observation`, bump `OBS_VERSION`, update `OBS_DIM`, retrain |

**Per-flight replay:** the flight log stores the full observation (`o0..o19`) and action
(`a0..a3`). Feed `o*` through any `.npz` (`MLPPolicy(...).network(obs)`) to see what another
policy *would* have done at that exact moment.

### 7.7 What to expect from your first training run

No trained policy ships with this package: `policies/` is empty until you train one. The
reference points measured while building it, so you know what "normal" looks like:

| Test | `carrot` baseline |
|---|---|
| `evaluate carrot --episodes 60 --seed 12345` (randomised course, corridor only) | 80% success, 65.6 s, p95 lateral 0.214 m |
| `mission_sim --checkpoints course_v3.yaml --course-obj ... --runs 30 --seed 500` (whole mission, real walls) | 27/30 landed and disarmed, 3 wall touches, 88 s mean arm → disarm |

* A **plain PPO** run (BC warm start, 400k steps) stayed at 73–78%. It did not beat `carrot`;
  BC copies it, and early PPO updates slightly degrade the copy.
* A **residual** run (`--residual-scale 0.3`) reached 80% at 100k steps and 78% at 200k, equal to
  the baseline. It was stopped there.

**Conclusion:** with the default ranges, most failures come from air pushes and cuVSLAM drift in
the 0.2 m zone-1 corridors, which no controller can see. Expect RL gains to come from:
- training with `--course-obj` (real walls);
- narrowing the dynamics ranges to your measured drone (7.6);
- many more steps (2–5M) on a machine with more cores (about 230 steps/s on 2 CPU cores here).

**Only deploy a network that beats `carrot` on the same seed.** Until then, fly `policy:=carrot`.

---

## 8. Verify 1: our own simulator (no ROS)

`mission_sim.py` runs **the drone node's state machine** (the same `mission.py`) against
stand-ins for the parts that are not ours. That is the whole of your steps 1–6, on a laptop,
in seconds.

| Stand-in for | What it does |
|---|---|
| the pilot | arms at 2 s, switches to Position at 4 s |
| flight_interface | Offboard only if a team setpoint arrived in the last second; pose floored at 0.3 m; twist capped |
| PX4 | the randomised model; the ground; a land detector; auto-disarm 1 s after landing; disarm if not airborne 10 s after arming |
| cuVSLAM | noise and drift |

```bash
python -m team_mission.rl.mission_sim                                     # test_line, carrot, prints every phase
python -m team_mission.rl.mission_sim --checkpoints test_square.yaml --log /tmp/sq.csv --plot /tmp/sq.png
python -m team_mission.rl.mission_sim --checkpoints course_v3.yaml --policy policies/v1.npz --runs 50
DRONEHACKS_REPO=<repo root> python -m team_mission.rl.mission_sim --checkpoints course_v3.yaml \
    --course-obj course/dronehacks-course-v3.obj --runs 50                 # count real wall touches
```

A good run prints:

```
DORMANT -> ARMED_WAIT -> TAKEOFF -> FLY -> (reached ...) -> DESCEND -> TOUCHDOWN -> DONE
result: landed and disarmed after ~24 s; landed 0.01 m from the last checkpoint
```

The `--log` CSV has the same columns as a real flight log, so `fit_dynamics` and your plots work
on both. `test/test_mission.py` runs these scenarios automatically:

- the full mission, in order
- a slow 1 m takeoff
- tracking loss → hold → resume
- pilot takeover → quiet
- a broken policy → hold
- never climbing without odometry

---

## 9. Verify 2: Gazebo

This runs on your laptop: WSL2 Ubuntu 24.04 with the team's simulator runtime (`~/dronehacks-runtime`:
ROS 2 Jazzy, Gazebo Harmonic, PX4 v1.16 SITL, MicroXRCEAgent, px4_msgs). It is the setup that
`gazebo/README.md` and `sim_stack/` in the repo use. This is the most realistic test before the
drone: the real PX4 flight code, the real `flight_interface`, and our node unchanged. Only the
pilot and (optionally) cuVSLAM are stand-ins.

### 9.1 One-time: a ROS workspace with the three packages

```bash
mkdir -p ~/tm_ws/src && cd ~/tm_ws/src
ln -s ~/dronehacks-runtime/ros_ws/src/px4_msgs .            # the px4_msgs matching PX4 v1.16 (already built there)
cp -r <x.zip>/skybeta/src/skybeta_flight_interface .        # from x.zip (unchanged)
cp -r <repo>/team_ws/src/team_mission .                     # this package (or symlink it while developing)
cd ~/tm_ws && source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
```

You can skip px4_msgs and source `~/dronehacks-runtime/ros_ws/install/setup.bash` before building.
`skybeta_flight_interface` is pure Python and works on Jazzy. It resolves PX4's `_v1` topic
names itself.

### 9.2 Run: empty world first (`test_line`, then `test_square`)

You need four terminals, each with `source /opt/ros/jazzy/setup.bash && source ~/tm_ws/install/setup.bash`.

```bash
# 1  PX4 SITL + Gazebo (x500), with no RC required, like the team's gazebo runner
cd ~/dronehacks-runtime/PX4-Autopilot
PX4_PARAM_COM_RC_IN_MODE=4 PX4_PARAM_NAV_RCL_ACT=0 PX4_PARAM_NAV_DLL_ACT=0 PX4_PARAM_COM_RCL_EXCEPT=4 \
  make px4_sitl gz_x500
#    (if a PX4_PARAM_ value does not stick, type it in the PX4 console: param set COM_RC_IN_MODE 4)

# 2  the uXRCE agent
MicroXRCEAgent udp4 -p 8888

# 3  flight_interface + team_mission + the simulated pilot + PX4-derived odometry
ros2 launch team_mission sim.launch.py odom_source:=px4 pilot:=direct checkpoints:=test_line.yaml

# 4  watch
ros2 topic echo /team_mission/status
```

`pilot:=direct` arms in Hold and asks for Offboard through flight_interface's SITL-only service.
It is the most likely to work with no RC configured. `pilot:=handshake` is the real procedure
(arm in Altitude, switch to Position). It needs PX4 to accept the simulated sticks on
`/fmu/in/manual_control_input`. If PX4 refuses Altitude, use `direct`, or set `pilot:=none` and
act as the pilot in QGroundControl with the virtual joystick on (arm in Altitude, then select
Position).

**What success looks like:**

1. In terminal 3: `DORMANT -> ARMED_WAIT -> TAKEOFF -> FLY -> reached forward_1m ->
   reached back_home -> DESCEND -> TOUCHDOWN -> DONE`.
2. `sim_pilot: disarmed ... Flight over.`
3. In Gazebo the x500 climbs slowly to 1 m, goes 1 m forward and back, sinks, lands and the
   propellers stop.
4. The log is in `~/team_mission_logs/` (`log_dir` defaults there off the drone).

Then do the same with `checkpoints:=test_square.yaml`, and with your trained policy (`policy:=v1.npz`).

### 9.3 Run: the course world

**Option A** (simplest, frame-safe). Use the course world from the repo's README §8
(`deploy/sitl/dronehacks_course_model.sdf`, placed so the world frame equals the course frame) and
spawn on the launch mark:

```bash
PX4_GZ_WORLD=dronehacks PX4_GZ_MODEL_POSE="-2.515,0.914,0.05,0,0,1.5708" \
PX4_PARAM_COM_RC_IN_MODE=4 PX4_PARAM_NAV_RCL_ACT=0 PX4_PARAM_NAV_DLL_ACT=0 PX4_PARAM_COM_RCL_EXCEPT=4 \
  make px4_sitl gz_x500
ros2 launch team_mission sim.launch.py odom_source:=px4 pilot:=direct checkpoints:=course_v3.yaml \
    launch_x:=-2.515 launch_y:=0.914 launch_z:=0.0 launch_yaw_deg:=0
```

> **`launch_yaw_deg:=0` here, not 90.** With `odom_source:=px4` the odometry axes are the
> simulator's world axes (PX4 east/north), not "nose at boot" as with cuVSLAM. The origin is
> still the spawn point. On the real drone it is 90.

**Option B** (closest to the drone). Use the team's Sky Beta world and simulated VIO. The team's
`gazebo/` runner already builds that world and runs `gazebo/vio_emulator.py`, which publishes
launch-frame odometry (FLU, x = course +y, origin on the mark) on `/gazebo/vio_odom`.
`deploy/vio_bridge.py` feeds it to PX4 like `px4_odom_bridge` does on the drone. Point our stack
at it, with the real-drone frame settings:

```bash
ros2 launch team_mission sim.launch.py odom_source:=topic odom_topic:=/gazebo/vio_odom pilot:=direct \
    checkpoints:=course_v3.yaml launch_x:=-2.515 launch_y:=0.914 launch_z:=0.0 launch_yaw_deg:=90
```

`run_course.py` starts the PX4 Offboard policy node itself. For our package, start the world,
PX4, the agent, `vio_emulator` and `vio_bridge` the way `run_course.py` does, but leave out its
mission and policy node.

### 9.4 What to check in Gazebo

| Check | Pass |
|---|---|
| the drone stays on the ground until the pilot arms | `/team_mission/status` phase DORMANT; no pose setpoints (`ros2 topic hz /skybeta/setpoint/pose` silent) |
| climb is slow and stops at about 1 m | ≤0.3 m/s; z 1.0 ± 0.1 before FLY |
| speed stays slow | horizontal speed ≤0.4 m/s (≤0.27 in tight segments) |
| corners | no overshoot beyond the corridor (no HOLD messages) |
| landing | sinks to about 0.3 m, then touches down; PX4 disarms within about 3 s of landing |
| takeover | in QGC, switch mode mid-flight → `ABORTED`, node goes quiet |
| log | `fit_dynamics` on the Gazebo log gives sensible numbers; use them as a sanity check of the training ranges |

---

## 10. Verify 3: the flyx lab

The flyx lab is the organizers' hosted simulator, the "DroneHacks Lab" (`/var/lib/flyx-lab/workspace`).
I assume this is what "our flyx environment" means. Training does not happen there: it runs in
real time, so it would take hours per policy. Training happens in section 7 in seconds per
episode. The lab is a second, independent check of the finished node.

Its own instructions apply:
- the team approves a mission before anything arms;
- **Reset world** before an automated run;
- no keyboard RC during automated runs.

1. **Get the code in.** Copy `skybeta_flight_interface` (from x.zip) and `team_mission` into the
   lab workspace's ROS source folder. In the lab terminal, build them with the lab's ROS 2 Jazzy:
   `colcon build --packages-select skybeta_flight_interface team_mission`. The lab already has
   matching `px4_msgs`. Our code has no version-specific parts.
2. **Inspect the lab's topics first:** `ros2 topic list`. Find the lab's cuVSLAM odometry topic
   (from its `onboard/slam/cuvslam_lib_node.py`) and check that PX4 topics appear (with `_v1`
   suffixes).
3. **First run, frame-safe:** odometry derived from PX4, to test the mission logic alone.
   ```bash
   ros2 launch team_mission sim.launch.py odom_source:=px4 pilot:=direct checkpoints:=test_line.yaml
   ```
4. **Second run, the lab's cuVSLAM** (closest to the drone):
   ```bash
   ros2 launch team_mission sim.launch.py odom_source:=topic odom_topic:=<the lab's cuVSLAM odometry> \
       pilot:=direct checkpoints:=test_line.yaml
   ```
   Check that "1 m forward" really goes 1 m **along the nose**. If it goes sideways or backwards,
   the lab's VIO frame is not the frame its EKF uses. `flight_interface` assumes they match (as on
   the drone), so stay with `odom_source:=px4` there and note it.
5. **Course:** `checkpoints:=course_v3.yaml` with the lab's launch pose (`launch_*`). Use `yaw 0`
   with `odom_source:=px4` (see 9.3).
6. **Pass criteria:** the same as 9.4. Keep the logs and run `fit_dynamics` on them.

---

## 11. Troubleshooting

| Symptom | Likely cause / fix |
|---|---|
| Stays `DORMANT` after arming | `/skybeta/state` not arriving or stale (>1 s): is flight_interface running? `ros2 topic echo /skybeta/state` |
| `ARMED_WAIT` forever, flight_interface says `staying out of Offboard: ...` | Read the reason. Usually: armed **in** Position (arm in Altitude, then switch), or no valid PX4 position (cuVSLAM not tracking). |
| PX4 disarms about 10 s after arming | Offboard never started, so it never took off (`COM_DISARM_PRFLT`). See the row above. |
| `-> HOLD: cuVSLAM not tracking (vo_state=None)` in a simulator | No `/visual_slam/status` there: `require_vslam_status:=false` (sim.launch.py sets it) |
| `-> HOLD: odometry ... s old` | Wrong `odom_topic`, or QoS. `ros2 topic hz <odom_topic>` |
| `-> HOLD: x m off the path` repeatedly | Drift or a bad launch pose (course files). Fly `test_square` to see whether the policy or the frame is the problem. |
| Flies the wrong way on a course file | `launch_yaw_deg` / `launch_x/y` wrong (6.5); with `odom_source:=px4` in simulators, yaw is 0 (9.3) |
| `policy was trained with limits ... flying with ...` | The `.npz` was trained with other speeds: retrain with `--limits`, or change `mission.yaml` back |
| `... trained for obs_version 1 ... this code is 2. Retrain.` | The contract changed; retrain and export |
| Landed but `PX4 has not disarmed` | Check `COM_DISARM_LAND` on the FC (2 s in the fleet files). The pilot disarms. |
| Two nodes fighting over `/skybeta/setpoint/pose` | `square_mission` is still running: stop the old competitor window first |

---

## 12. Design decisions

- **Replace square_mission, keep flight_interface.** It owns the safety envelope and the arming
  handshake; the organizers' guide says to keep it and swap only the mission node.
- **Wake on arm, not on Position.** flight_interface needs a live team setpoint at the moment of
  the switch, so "dormant until Position" could never get Offboard.
- **Pose carrot, not twist.** Safe fallback if our node stalls (section 4); PX4's position loop
  does the low-level work, so the policy learns "where to go next", which transfers better than
  "how to fly".
- **Heading-frame observation.** The policy is independent of where the course is and which way it
  points; random paths in training make it work for any checkpoint list.
- **One contract file.** Training and drone call the same functions, so there is no
  re-implementation to drift apart; the `.npz` carries its contract version and speed limits and
  the node checks both.
- **Numpy inference.** The drone image has numpy, not torch; a 2×64 network is microseconds.
- **Behaviour cloning first.** PPO from scratch spends most of its time learning to not crash; from
  the baseline it only has to improve it, and the baseline remains a fallback (`policy:=carrot`).
- **Domain randomisation of PX4 + cuVSLAM, not rotor physics.** We command positions; what matters
  is how the drone responds to them and how wrong the estimate is.
- **Slow by construction.** Limits live in the contract and in the state machine, not in the
  network, so no policy can fly faster than the YAML allows.
- **Compliant landing.** Twist down + PX4 auto-disarm is the only landing available without
  publishing to `/fmu/in/*`.
