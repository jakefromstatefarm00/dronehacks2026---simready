# PX4 flight logs (ULog) of skybeta-06 -- what each one shows

## Session 2 (new camera + cable; logs 68-73 in `drone_logs/qgc_2026-11-08/`, downloaded 2026-11-08 12:26)

| log | length | what happened |
|---|---|---|
| 68 | 2 s | armed, vision fused (`cs_ev_pos` true, EV age ~140 ms + 70), disarmed again |
| **69** | **25 s** | **flew.** Position -> Offboard 1.8 s after arming, liftoff 2.7 s, slow climb to a hover at 0.77-0.86 m (setpoint 0.85), xy drift < 0.1 m, roll/pitch < 2 deg, yaw steady; at 16 s the program starts translating (+y 0.3 m/s), the drone follows 0.3 m behind; **pilot kill at 19.3 s** in a normal hover -> fall, tumble, impact ("Attitude failure (pitch)") 24.2 s, disarm 24.3 s. Vision fused 98 % with test ratio 0.00 until the fall (lost at 20.1 s, after the kill). |
| 70, 71, 72, 73 | 4-6 s each | armed, "Switching to Position is currently not available", **no vision reaching EKF2 at all** (no `estimator_aid_src_ev_pos`), disarmed |

Reading: the new camera + cable gave a clean estimator for the whole flight (the first time EV fusion
held through liftoff and translation). The flight itself was nominal until the kill. After the fall the
vision pipeline was dead for every following attempt -- camera dropped in the impact, or cuVSLAM lost /
diverged -- and the stack was not restarted, so logs 70-73 are the "no vision at all" failure, not the
"too old" one. Rule: **after any kill, crash or camera event, restart the stack and re-run the
preflight before arming again.**

Files: `drone_logs/qgc_2026-11-06/log_24..31_*.ulg` (downloaded with QGC 2026-11-07 23:40; the FC named
them `/fs/microsd/log/2026-11-06/15_43_38.ulg` etc., UTC). **Today's (2026-11-08) flights are still on
the Pixhawk SD card** -- QGC -> Analyze Tools -> Log Download over the FC's USB cable; drop them in
`drone_logs/qgc_2026-11-08/` and rerun the script at the bottom.

Decoded with pyulog (`vehicle_status`, `vehicle_local_position`, `vehicle_land_detected`,
`estimator_status_flags`, `estimator_aid_src_ev_pos`, PX4 text messages).

## Seven logs that armed and never flew (24, 25, 26, 27, 28, 29, 30; 4-10 s each)

```
-0.0s [commander] Armed by RC switch
 2.2s [commander] Switching to Position is currently not available      <- the pilot flipped to Position
 9.5s [commander] Disarmed by RC switch / Kill engaged
```
* nav_state **ALTCTL** the whole time; `xy_valid` **0 %**; `cs_ev_pos` **0 %**; never left the ground.
* **No `estimator_aid_src_ev_pos` dataset at all**: EKF2 received **no vision data whatsoever** in these
  logs (the bridge / stack was not delivering -- stack not up yet, cuVSLAM not tracking, or the TELEM2
  link). This is a *different* failure from 2026-11-08, where vision arrived but was too old.

## The one log that flew: log_31 (11.2 s)

```
 -1.0s  cs_ev_pos=1, cs_ev_yaw=1, xy_valid=1        vision fused ON THE GROUND before arming
 -0.0s  [commander] Armed by RC switch               (ALTCTL)
  2.1s  POSCTL -> OFFBOARD                            pilot flipped to Position; flight_interface took Offboard
  3.0s  [commander] Takeoff detected                  in air 3.0 - 10.2 s, rose +0.33 m
  4.0s  last EV position sample fused                 cuVSLAM output stopped (camera lost at liftoff)
  4.3s  cs_ev_pos=0, cs_ev_yaw=0                      EKF2 dropped vision 1.3 s after liftoff
  4.4s  cs_yaw_align 0->1                             heading re-aligned (to the magnetometer: the 126 deg jump)
  5.2s  [commander] Kill engaged                      pilot killed it (drifting back/left, yaw spin)
  6.5s  [failsafe] Failsafe activated  -> POSCTL
  8.4s  [mc_pos_control] invalid setpoints / Failsafe: blind land -> ALTCTL ; xy_valid=0 at 8.5s
 10.2s  [commander] Disarmed by kill-switch
```
* `estimator_aid_src_ev_pos`: fused **100 %**, innovation_rejected 0 %, age (publish - sample) p50 282 ms,
  which is arrival age ~217 ms + EKF2_EV_DELAY 70 -> inside EKF2's acceptance window (see below) but close
  to its edge (235 ms).

## Matching the Jetson-side journals (pulled 2026-11-08 from `~/broken_team_ws/runs`, copies in `drone_logs/jetson_2026-11-08/runs/`)

The FC clock had no real time (`time_ref_utc=0`); its "2026-11-06 15:43-16:01 UTC" is really
**2026-11-07 ~21:07-21:25 PST**. The team package (`dronehacks_pathplan`) journals from that evening:

| journal (PST) | what the program logged | PX4 log |
|---|---|---|
| pathplan-012453 (17:24), 045809 (20:58) | `READY` <-> `WAIT_HEALTH` cycling: **"PX4 has no valid local position (vision not reaching EKF2?)"**, "no fresh odometry (0.32 s old) - is cuVSLAM tracking?" | logs 24-30: armed, ALTCTL, "Switching to Position is currently not available", no EV data |
| **pathplan-052255 (21:22)** | `READY` -> `ARMED` 21:25:16 -> **`TAKEOFF` "Offboard active"** 21:25:18 -> `NAVIGATE` "at altitude" 21:25:23 | **log_31**: Position -> Offboard 2.1 s after arming, takeoff at 3.0 s, EV lost at 4.3 s, killed 5.2 s |
| pathplan-075710 (23:57), today's 184413 (10:44 PST, new team_ws) | `READY` <-> `WAIT_HEALTH` every few seconds: **"no fresh odometry (0.30-0.34 s old)"** | (today's logs still on the SD card) |

Same code, same drone, same evening: the one run that flew is the one where EKF2 happened to be
fusing vision at the moment of arming. The failed ones say so themselves. The vision age sits on
EKF2's acceptance edge and drifts per boot (measured: 206, 218, 290, ~310 ms), which is why it
"sometimes works".

## What this proves

1. **The chain can work**: with vision fused on the ground, the pilot's arm -> Position flip engaged
   Offboard within 0.1 s and the drone took off under program control (log_31).
2. **Position mode is accepted only when EKF2 is fusing vision.** Every log without `cs_ev_pos` shows
   PX4 refusing Position ("currently not available") and staying in ALTCTL; on 2026-11-08 the same refusal
   plus the position failsafe disarmed it on the ground.
3. **Two distinct ways vision was missing**: Nov 6 logs 24-30 -- nothing reached PX4 (link/stack); Nov 8 --
   samples reached PX4 ~290 ms old and EKF2 discarded them (`EKF2_EV_DELAY` 70, `EKF2_DELAY_MAX` 200:
   a sample is fused iff `age + EV_DELAY < max(DELAY_MAX, EV_DELAY) + 100 ms`, i.e. age < ~235 ms;
   log_31's ~217 ms and the 03:21 boot's 218 ms passed, the 03:00 boot's 290 ms did not).
4. **Vision is lost ~1 s after liftoff** (log_31; Nov 8 03:22 the camera physically dropped off USB3 at
   arming). After that EKF2 falls back to IMU + magnetometer and the heading jumps.

So the fixes: camera USB cable (vibration); treat a high-latency boot as a bad boot and restart the stack
until the driver leg is < 150 ms (`preflight.sh` checks it; 11:10 PST measurement: 36 ms / p50 72 ms and
EKF2 fusing); `EKF2_DELAY_MAX 400` as margin for spikes (`jetson/px4/skybeta06_ekf2_vision_delay.params`,
`EKF2_EV_DELAY` stays 70); and the `fly` / `preflight.sh` gate that only says GO with `cs_ev_pos` true and
a valid local position -- the exact state log_31 started from.

## Rerun

```bash
python -I tools/px4log_summary.py drone_logs/qgc_2026-11-08/*.ulg
```
