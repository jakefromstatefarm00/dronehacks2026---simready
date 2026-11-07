# flyx_square — a square flight for the FlyX lab's Run button

```bash
python3 flyx_square/fly_square.py          # flies;  --shadow: everything except publishing to PX4
```

One self-contained file (rclpy, px4_msgs, std_msgs). Reset world, start on the launch mark
facing up the first corridor, run it.

```
climb 0.9 m -> forward 0.6 m -> up 0.6 m -> back 0.6 m -> down 0.6 m -> land (native PX4)
```

- **Native PX4**, like the lab's working takeoff/hover/land example: it publishes
  `/fmu/in/offboard_control_mode`, `/fmu/in/trajectory_setpoint`, `/fmu/in/vehicle_command`
  and reads `/fmu/out/vehicle_odometry`, `vehicle_status(_v1)`, `vehicle_land_detected`,
  `vehicle_local_position(_v1)` (version suffix discovered). It is not a skybeta node.
- **Why a vertical square.** A flat square does not fit at the launch mark: the first
  corridor is ~1 m wide and with Sky Beta's 0.28 m radius only its centre line keeps
  >= 0.4 m from the walls (published-map clearance). Straight up the corridor the clearance
  is 0.5 m for ~1.2 m, so the square is forward/up/back/down along it. Top 1.5 m (walls 1.73 m).
- **Frames.** Everything is in PX4's local NED relative to the take-off pose; "forward" is
  the heading PX4 reports on the ground. No course alignment needed.
- **Flow and guards** follow `pathfinding/flyx/mission.py` (FlyX-tested): wait for fresh
  odometry/status/local position, valid estimator, landed + disarmed for `SETTLE_S` (10 s); stream hold
  setpoints 1 s; Offboard; arm; fly. **Message pauses hold, they don't land**: if odometry is
  > 0.4 s old (or local position > 1 s) the last setpoint keeps streaming (PX4 holds on its own
  EKF) and the square's progress freezes until messages return. Lands on a real outage
  (odometry or local position missing > 3 s), status > 3 s, invalid estimator, a corner not
  reached in time (pauses excluded), or 120 s.
  Ages use a monotonic clock (the lab's sim clock jumps). Ctrl-C / lab Stop lands.
  If PX4 leaves Offboard by itself it stops commanding.
- Knobs at the top of the file: `HEIGHT_M`, `SIDE_M`, `SPEED_MPS`, `HOLD_S`, `TOLERANCE_M`.
  `python3 flyx_square/fly_square.py --shadow` runs everything without publishing to PX4.
- Journal: `flyx_square/runs/square-<time>/journal.jsonl` (or `~/flyx-runs`, `/tmp/flyx-runs`).

## Tested

- `test_fly_square.py` (offline, kinematic PX4 stand-in): square completes and lands at the
  take-off point (~22 s); refused arm fails without moving; stale odometry mid-flight lands.
- Real PX4 v1.16 SITL + OpenVINS external vision (EKF2 vision position/yaw) in the Gazebo
  DroneHacks course, Sky Beta airframe, team code on `/fmu/in` exactly as in the lab
  (`sim/gazebo_smoke.py --native-program fly_square.py`):

| run | settle | result |
|---|---|---|
| v122 | 2 s | armed 4 s after the rig set the drone down; OpenVINS lost every feature at liftoff, PX4 dropped `xy_valid`, the script **landed itself** (guard worked) |
| **v123** | 45 s | **square complete, landed + disarmed, exit 0**. Arm → disarm 28.2 s (airborne 21.1 s). Truth: forward 0.64 m (plan 0.6), peak 1.42 m above start (plan 1.5), sideways 0.01 m, touchdown 5 mm from take-off, **0 contacts** while airborne |

| **v124** | 10 s (committed default, no arguments) | **square complete, landed + disarmed, exit 0**. Arm → disarm 27.8 s. Truth: forward 0.64 m, peak 1.42 m, sideways 0.00 m, touchdown 6 mm from take-off |

  Video: `results/gazebo-v123/flight-video/flight.mp4` (top: overhead observer, bottom:
  onboard stereo with VIO tracks). Journal, truth track and contact check alongside.
  Hence the 10 s default settle (the lab's estimator has run since Reset world, so it
  should be healthy well within that).

## FlyX lab run 1 (2026-11-07) and the fix

Armed, climbed ~half a second, then `FLY -> LAND: unhealthy: odometry stale` 2.8 s after
arming. PX4 publishes `vehicle_odometry` continuously from its EKF, so this was the lab's
message stream / simulation pausing (its `/clock` arrives in bursts; teammates' FlyX run 3
saw a 0.69 s `local_position` pause), not a lost estimate. The old guard was 0.3 s of wall
time. Now odometry is stale only after 0.5 s of **both** wall and sim time without a message
(if the sim paused, PX4 paused too), or 1.5 s of wall time regardless. Tests reproduce both
cases. The journal now also logs every stream pause (`gap`), control-loop stalls
(`control_gap`) and a 2 Hz `tel` record (ages, validity, mode, position) - send it over
after the next lab run. Re-flown in Gazebo after the change: v125, square complete, exit 0.

## FlyX lab run 2 (2026-11-07) and the fix

Flew ~20 s (climb, forward leg, part of the climb leg), then `FLY -> LAND: unhealthy: odometry
stale (1.08 s wall, 1.10 s sim)`. Run 1's fix (stale = 0.5 s of both wall and sim time) was not
enough: the lab's PX4 -> ROS stream paused for over a second while sim time ran on. PX4 keeps
estimating and flying through such pauses, so now a pause holds (setpoint frozen, last one
re-streamed) and the square resumes when odometry returns; only a 3 s outage lands. Tests
reproduce run 1 (0.5 s pause after arming) and run 2 (1.2 s pause mid-square): both complete.
Shutdown no longer prints "cannot use Destroyable" (executor joined before the node is destroyed).

Re-flown after the run-2 fix in Gazebo: v126, square complete, landed + disarmed, exit 0.

### What the run-2 journal showed

Pauses are constant in the lab, not occasional: odometry gaps of 0.15-1.4 s every few seconds,
`local_position` gaps up to 2.4 s, and they coincide with the program's own 20 Hz control loop
not running (`control_gap` up to 1.75 s) while sim time keeps advancing (~0.8x real time): the
whole companion process is starved of CPU at times. `test_lab_run2_pause_pattern_completes`
replays those measured gaps (including the stalls, during which no setpoint is sent) and the
current version completes the square. Remaining risk: a stall longer than PX4's Offboard-loss
timeout makes PX4 take over (the script then stops commanding, `TAKEN_OVER`); nothing in the
program can prevent a container-level stall.

## FlyX lab run 3 (2026-11-07) and the fix

Took off and flew ~15 s, then `FLY -> LAND: unhealthy: vehicle_status stale`. The journal shows
vehicle_status, land_detected and local_position all silent for ~6 s at the same moment while
odometry kept arriving and PX4 stayed in Offboard. It had also frozen the square twice on
local_position pauses (1.3 s, 1.5 s) while odometry was fine. Now, in the air, odometry is the
only liveness signal (hold on a > 0.4 s pause, land after 3 s without it); the slow topics are
latest-known state: a fresh "estimator invalid" still lands, PX4 leaving Offboard still stops
commanding, but silence alone only lands after 10 s. Before arming, every topic must still be
fresh. Tests replay runs 1-3 (11 pass).
