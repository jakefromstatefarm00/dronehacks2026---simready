# path_recorder: record the path the pilot flies, for an exact replay later

The pilot flies the drone on RC as usual. This package records where it went: **one recording per
arm → disarm**. Each saved point holds exactly what `/skybeta/setpoint/pose` takes, so a later replay
node can loop through the points and publish them.

**It never flies anything.** It publishes nothing to `/skybeta/setpoint/*` or `/fmu/*`. Sky Beta's
`flight_interface` runs beside it, so flipping to Position after arming is refused for Offboard
(`auto offboard: staying out of Offboard: no team setpoint received`). The drone stays in the
pilot's Position mode the whole time.

```
team_ws/src/path_recorder/
├── launch/path_recorder.launch.py   boot program: flight_interface (safe) + recorder
├── path_recorder/node.py            the recorder node                       (recorder)
├── path_recorder/recording.py       recording logic + file format, no ROS
├── path_recorder/dashboard.py       live SSH dashboard                       (dashboard)
├── path_recorder/summary.py         check a saved recording, no ROS needed   (summary)
└── test/test_recording.py           pytest, no ROS
```

---

## 1. Install on the drone (one time)

1. Copy this folder to `~/team_ws/src/path_recorder` with `scp`, `rsync` or `git`. A web uploader
   may drop the empty file `resource/path_recorder`. Check it's there:
   ```bash
   ls ~/team_ws/src/path_recorder/resource/path_recorder
   ```
2. Make it the boot program for recording days. In `/etc/skybeta/drone.env`, change only these two
   lines:
   ```
   COMPETITOR_PKG=path_recorder
   COMPETITOR_LAUNCH=path_recorder.launch.py
   ```
   For autonomy days, set them back to `jolie_best` / `jolie_best.launch.py`.
3. Restart the stack **on the ground, drone still**. This re-zeroes cuVSLAM.
   ```bash
   docker rm -f skybeta-stack
   ```
   The competitor window builds `team_ws` and starts the recorder. From then on, every power-on
   records.

Why replace `jolie_best` at boot: with v4c as the boot program, a pilot who arms and flips to
Position on the ground hands the drone to v4c's autonomy. That's the wrong thing during an RC
recording flight.

## 2. Recording a path

1. **Put the drone on its start spot, facing the way it should start.** Mark the spot and the
   heading on the floor. The replay must start from the same spot facing the same way. Every point
   is stored relative to the pose at arming, so where the drone was powered on doesn't matter.
2. **Arm.** The recording starts, and the competitor window logs `ARMED -> RECORDING path-...`.
3. **Fly the path on RC** in Position mode. Fly at the speed you want replayed, and pause where the
   replay should pause. The `t` column records timing.
4. **Land and disarm.** The file is closed: `DISARM -> SAVED ...: N points, L m, T s`.
5. **Arm again for the next path.** Each arming makes a new folder.

**Points saved:**

- the first pose after arming
- one every **15 cm** of travel
- one every **15°** of heading change, so turns in place are captured
- the final pose before disarming

Hovering adds no points. Points while landed (before liftoff, after touchdown) are kept and
flagged `landed=1`.

## 3. Monitoring over SSH

Run these on the Jetson, inside the stack:

```bash
cd ~/skybeta && host/run.sh shell
source /root/team_ws/install/setup.bash
ros2 run path_recorder dashboard          # live, refreshes every second; Ctrl-C quits (recorder keeps going)
```

```
* RECORDING  path-20261108-002729-001
drone    armed YES   landed False   mode POSCTL   offboard no
health   PX4 link ok   PX4 position valid   odometry 30 Hz (last 0.01 s)   depth /camera/depth/image_raw 15 Hz
clear    left 1.84 m   centre 3.10 m   right 2.25 m
points   41   path 5.62 m   time 24 s   airborne 19 s   max height 0.91 m
pose     x +2.03 fwd   y +1.10 left   z +0.90 up   yaw +88 deg   (relative to where it was armed)
flags    none
last points:  idx  t  x  y  z  yaw  speed  clr_c  reason  flags
...
```

**What to check:**

| Line | Healthy |
|---|---|
| `* RECORDING` | appears when armed, disappears when disarmed |
| `offboard` | always **no** |
| `odometry` | about 30 Hz, steady |
| `pose` | follows the drone: forward = +x, left = +y, up = +z, turning left = +yaw |
| `points` | goes up about 6–7 per metre flown |
| `flags` | stays **none** |
| `clear` | shows distances (if it says "no depth", the clearance columns stay empty) |

`odom_jump` means cuVSLAM reset or relocalised. Trust the points after it less.

**After a flight**, no ROS needed (on the drone host, or a laptop after copying):

```bash
ros2 run path_recorder summary            # newest recording: checks + a top-down sketch
ros2 run path_recorder summary --list     # every recording
```

The competitor window (`docker exec -it skybeta-stack tmux attach -t skybeta`, Ctrl-b w →
`competitor`) also shows the recorder's log: start, save and every warning.

## 4. Where the data is, and what to send

```
~/team_ws/runs/paths/path-<date>-<time>-<n>/
    points.csv     one row per saved point (below)
    meta.json      summary: points, length, duration, airborne time, flags, anchor, frame, columns
    events.jsonl   timeline: mode changes, liftoff/touchdown, odometry gaps/jumps, start/end
```

Copy a recording to your laptop. The files are owned by root, which is readable but not deletable
without `sudo rm -r`.

```bash
scp -r dronehacks@<drone>.local:~/team_ws/runs/paths/path-<...> .
```

To build the replay flight, send the whole `path-...` folder.

Files are written point by point. If the drone loses power mid-flight, everything up to the last
point is kept, and `meta.json` says `"status": "recording"`.

### points.csv columns

| Column | Meaning |
|---|---|
| `idx`, `t` | point number; seconds since the first point |
| `x`, `y`, `z` | **arm frame**: metres forward / left / up from the pose at arming |
| `yaw` | radians, CCW, 0 = heading at arming |
| `qx qy qz qw` | the same heading as a quaternion (the setpoint's orientation) |
| `vx vy vz`, `speed`, `yaw_rate` | velocity over the last 0.2 s (arm frame), horizontal speed, turn rate |
| `landed`, `mode` | PX4 land detector (1 = on the ground); PX4 mode (POSCTL, ALTCTL, …) |
| `clear_left/center/right` | metres to the nearest thing in the forward-left / forward / forward-right third of the depth view (flight-height band, 5th percentile); empty = no depth |
| `depth_age` | how old that depth was, in seconds |
| `odom_x … odom_qw` | the raw cuVSLAM pose of this flight (its own odom frame) |
| `reason` | why this point was saved: `start`, `distance`, `turn`, `end` |
| `flags` | `odom_gap`, `odom_jump`, `depth_stale`, `px4_no_position`, `started_armed` |

**Replay math.** The replay arms on the same spot facing the same way, and reads its own odom
anchor `p0` and `yaw0`. For each point it then publishes:

```
position = p0 + Rz(yaw0)·(x, y) + (0, 0, z)
yaw      = yaw0 + yaw
```

as `PoseStamped` (`frame_id: odom`) on `/skybeta/setpoint/pose`.

## 5. Running it by hand instead of at boot

First stop whatever the competitor window runs (attach, select `competitor`, Ctrl-C). Two flight
interfaces must never run together. Then:

```bash
ros2 launch path_recorder path_recorder.launch.py                     # flight_interface + recorder
ros2 launch path_recorder path_recorder.launch.py interface:=false    # recorder only, if another
                                                                      # flight_interface already runs
```

Options: `spacing_m:=0.15`, `turn_deg:=15.0`, `output_dir:=...`.

## 6. Tested

| Test | Result |
|---|---|
| `pytest test` (Python 3.10, numpy 1.26) | 8 passed: frame maths, 15 cm / 15° sampling, setpoint fields, flags, crash-safe files, depth clearance |
| ROS 2 Jazzy **and Humble**, fake RC flight (odom frame offset 0.5 m and rotated 30°, climb, 2 m forward, 90° turn, 1 m, land, 0.6 s gap, 0.6 m jump, second aborted arming) | 37 points; path correct in the arm frame (2 m forward then 1 m left); gap and jump flagged; aborted arming marked short; depth auto-found on `/oak/stereo/image_raw` |
| Real `flight_interface` + fake PX4: Altitude → arm → Position → disarm | arming started the recording, **Offboard refused** ("no team setpoint received"), disarm saved it, nothing published to `/skybeta/setpoint/pose` |

Not tested: the real drone (cuVSLAM, real depth, the Jetson's CPU).
