# flyx_square — a square flight for the FlyX lab's Run button

`main.py` is self-contained (rclpy, px4_msgs, std_msgs). Put it at the project root the
lab runs, Reset world, start on the launch mark facing up the first corridor, press Run.

```
climb 0.9 m -> forward 0.6 m -> up 0.6 m -> back 0.6 m -> down 0.6 m -> land (native PX4)
```

- **Native PX4**, like the lab's working takeoff/hover/land `main.py`: it publishes
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
  setpoints 1 s; Offboard; arm; fly. Lands on stale odometry (> 0.3 s), stale local position
  (> 2 s; FlyX run 3 saw a 0.69 s gap) or status (> 3 s), invalid estimator, a corner not reached in time, or 120 s.
  Ages use a monotonic clock (the lab's sim clock jumps). Ctrl-C / lab Stop lands.
  If PX4 leaves Offboard by itself it stops commanding.
- Knobs at the top of the file: `HEIGHT_M`, `SIDE_M`, `SPEED_MPS`, `HOLD_S`, `TOLERANCE_M`.
  `python3 main.py --shadow` runs everything without publishing to PX4.
- Journal: `runs/square-<time>/journal.jsonl` (or `~/flyx-runs`, `/tmp/flyx-runs`).

## Tested

- `test_main.py` (offline, kinematic PX4 stand-in): square completes and lands at the
  take-off point (~22 s); refused arm fails without moving; stale odometry mid-flight lands.
- Real PX4 v1.16 SITL + OpenVINS external vision (EKF2 vision position/yaw) in the Gazebo
  DroneHacks course, Sky Beta airframe, team code on `/fmu/in` exactly as in the lab
  (`sim/gazebo_smoke.py --native-program main.py`):

| run | settle | result |
|---|---|---|
| v122 | 2 s | armed 4 s after the rig set the drone down; OpenVINS lost every feature at liftoff, PX4 dropped `xy_valid`, main.py **landed itself** (guard worked) |
| **v123** | 45 s | **square complete, landed + disarmed, exit 0**. Arm → disarm 28.2 s (airborne 21.1 s). Truth: forward 0.64 m (plan 0.6), peak 1.42 m above start (plan 1.5), sideways 0.01 m, touchdown 5 mm from take-off, **0 contacts** while airborne |

  Video: `results/gazebo-v123/flight-video/flight.mp4` (top: overhead observer, bottom:
  onboard stereo with VIO tracks). Journal, truth track and contact check alongside.
  Hence the 10 s default settle (the lab's estimator has run since Reset world, so it
  should be healthy well within that).

Not tested: the FlyX lab itself (its cuVSLAM, its rendering). First lab run: watch the
journal for the WAIT_HEALTH reason if it never arms.
