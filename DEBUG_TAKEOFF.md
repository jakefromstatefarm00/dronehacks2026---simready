# Why the drone will not take off (skybeta-06, 2026-11-08)

Status for anyone helping debug. Nothing here is a code bug in v4c: **PX4 never has a valid local
position**, so no autonomous program can start, and PX4 itself disarms when the pilot asks for a
position-based mode.

## Symptoms seen today

* The pilot can arm (Manual / Altitude need no position estimate).
* Flipping to Position, or anything that needs a position, makes PX4 **disarm on the ground**.
  That is PX4's position failsafe: landed + failsafe = disarm.
* v4c (`jolie_best`) sits in `WAIT_HEALTH` / `WAIT_OFFBOARD` forever. 24 runs today, same picture.

## Evidence (bag `rc-20261108-011337`, 10 s, Jetson `~/team_ws/runs/`; metadata in `drone_logs/jetson_2026-11-08/`)

`/skybeta/state` (the organiser flight interface), every message while armed:

```
"px4_link": true, "armed": true, "landed": true,
"nav_state": 1, "nav_state_name": "ALTCTL",      <- pilot had Position selected; PX4 fell back to Altitude
"odom_enu": [-0.072, 0.027, 0.001],              <- cuVSLAM pose: fine
"local_enu": null                                 <- PX4 vehicle_local_position xy_valid/z_valid FALSE
```

`local_enu: null` was constant from boot to the end of the recording. v4c journals (`flyx-*/journal.jsonl`)
show PX4 in `ALTCTL` or `MANUAL` every time it was armed; it reached `POSCTL` only twice all day
(06:08 and 09:14 local), both while disarmed.

## Why that blocks everything

1. PX4 refuses Position mode (and Offboard position setpoints) without a valid local position.
2. The organiser's `auto_offboard.py` only requests Offboard on an arm -> Position switch, and refuses
   with `no valid local position (is VIO tracking?)` when `vehicle_local_position` is invalid.
3. Both team programs (`upload_to_flyx_v4c_skybeta.py` and the hardcoded `_hard.py`) gate on
   `px4_link and local_enu is not None` (hard: line 2684). They cannot work around 1 and 2.

## What is NOT the problem

* cuVSLAM is tracking: `/visual_slam/status vo_state 1`, `/visual_slam/tracking/odometry` 30 Hz.
* The bridge forwards it: `/fmu/in/vehicle_visual_odometry` ~34 Hz.
* Setpoints flow (v4c publishes `/skybeta/setpoint/pose` at 10-20 Hz; `clamped: ["min_altitude"]` as expected).
* Camera on USB3 (`03e7:f63b`), Jetson disk 135 GB free, stack restarts cleanly.

## ROOT CAUSE (measured live 2026-11-08 03:00, Pixhawk on, drone on the ground)

```
estimator_status_flags:  cs_ev_pos false  cs_ev_yaw false  cs_ev_hgt false   <- EKF2 fuses NO vision
vehicle_local_position:  xy_valid false  dead_reckoning true  xy_reset_counter 11
/fmu/in/vehicle_visual_odometry: pose_frame 2 (ok), quality 0, timestamp - timestamp_sample = 0.29 s (p50), 0.59 s max
```

Where the age comes from (4 s sample, each stage's header stamp vs the node clock):

| stage | age |
|---|---|
| `/oak/imu/data` | 13 ms (fine) |
| `/oak/left/camera_info` (= image stamp) | **273 ms** (259-285) |
| `/visual_slam/tracking/odometry` | 295 ms (cuVSLAM adds ~2 ms) |
| `/fmu/in/vehicle_visual_odometry` timestamp_sample | 290 ms (278-594) |

The camera **images** are stamped ~270 ms before they reach ROS (8 frames at 30 fps: smells like a
full 8-deep host queue in the depthai driver). The FC has `EKF2_EV_DELAY = 70` (organisers' bench
value, confirmed in the ULog), `EKF2_DELAY_MAX = 200`. A sample 290 ms old lands outside EKF2's
buffer and is dropped, so vision is never fused, so no local position, so no Position / Offboard,
and PX4's position failsafe disarms on the ground.

The organisers' own procedure (`~/skybeta/docs/fleet_guide.md` §7, `CLAUDE.md`, `host/run.sh`):
**"EKF2_EV_DELAY = measured, not assumed"** — measure with `tools/latency_probe.py` on *this*
airframe, set the value in QGC (range 0-300 ms), reboot the FC. Bench Orin: 70 ms; drone 16: 60 ms.
This airframe: ~280-290 ms, a 4x outlier ("a drone that is an outlier here is telling you something").

Two fixes, do both:
1. **Now (QGC, organiser-sanctioned):** `EKF2_EV_DELAY` = measured p50 (~290; cap 300), reboot FC.
   Check `cs_ev_pos` becomes true and `/skybeta/state local_enu` is non-null before arming.
2. **Find the 200 ms:** `ev_latency_experiment.sh` compares the probe with the competitor window
   running vs idle. If idle drops it to ~70 ms, our nodes (depth relay / v4c depth subscription)
   starve the camera driver and we fix that on our side; if not, it is the camera driver / Jetson
   and the organisers need to know.

## Also checked (for completeness)

EKF2 is receiving external vision but not using it (or rejecting it). Things to check live with the
Pixhawk powered (ROS CLI inside `docker exec skybeta-stack bash -ic ...`):

```bash
ros2 topic echo --once /fmu/out/vehicle_local_position | grep -E "xy_valid|z_valid|v_xy_valid|dead_reckoning"
ros2 topic echo --once /fmu/out/estimator_status_flags | grep -E "cs_ev_pos|cs_ev_vel|cs_ev_yaw|cs_ev_hgt|cs_tilt_align|cs_yaw_align|reject_ev|cs_in_air"
ros2 topic echo --once /fmu/in/vehicle_visual_odometry      # pose_frame (must be 2 = FRD / NED), timestamp vs PX4 time, quality, variances not NaN
ros2 topic echo --once /fmu/out/timesync_status              # offset between Jetson and PX4 clocks
```

Likely causes, in order (PX4 parameters are read in QGC; **do not change them without the organisers**):

| Check | Expect | Means if wrong |
|---|---|---|
| `EKF2_EV_CTRL` | bit 0 (horizontal pos) and bit 2 (yaw) set (organiser value: 9? confirm) | EV not fused at all |
| `EKF2_HGT_REF` | 3 = vision, or baro with EV_CTRL bit 1 | no height -> `z_valid` false |
| `cs_tilt_align` / `cs_yaw_align` | true | EKF2 never initialised: drone moved during boot, or vibration |
| `vehicle_visual_odometry.timestamp_sample` vs PX4 now | within ~EKF2_EV_DELAY (70 ms) | timesync off -> EV rejected as too old (earlier ULog showed ~210 ms age) |
| `pose_frame` | 2 (FRD) | frame mismatch -> rejected |
| `position_variance` / `orientation_variance` | finite, small | NaN -> rejected |
| `reject_ev_pos` / `reject_ev_yaw` flags | false | innovation too large (EV disagrees with IMU) |

Also from the QGC ULog of the only real launch (log_31, see `drone_logs/qgc_2026-11-06/analysis/`):
`EKF2_MAG_TYPE=0` lets the magnetometer yank heading 126 deg when EV drops; `CA_ROTOR*` geometry is
front-heavy (nose-up at liftoff). Those matter once it flies; they do not cause today's `local_enu: null`.

## Boot program state

* `/etc/skybeta/drone.env`: `COMPETITOR_PKG=jolie_best`, `COMPETITOR_LAUNCH=jolie_best.launch.py`.
* `jolie_best.launch.py` now defaults to `script:=upload_to_flyx_v4c_skybeta_hard.py` (hardcoded route,
  `--start-script hard`); `script:=upload_to_flyx_v4c_skybeta.py` gives the planner v4c back.
* Shadow is never applied automatically (`record` leaves the competitor LIVE; `record --shadow` opts in).
* `path_recorder/` (RC path recording boot program): set `COMPETITOR_PKG=path_recorder`,
  `COMPETITOR_LAUNCH=path_recorder.launch.py` and restart the stack for recording days.
