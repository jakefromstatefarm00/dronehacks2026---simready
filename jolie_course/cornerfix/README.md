# cornerfix: the first corridor-to-maze corner and VIO divergence

Goal: stop OpenVINS from diverging at the first corner. That is the top of the first corridor,
where the route turns from north to east and then south around maze divider M1. We made targeted
changes to Jolie's FlyX-proven `upload_to_flyx.py` and A/B-tested them in the GB10 Gazebo rig
(real PX4 SITL + OpenVINS). Branch `first-corner-vio`.

| File | What |
|---|---|
| `build_cornerfix.py` | Builder. Reads `../upload_to_flyx.py` (never writes it), makes marked insertions, and writes the four files below. The Sky Beta copies come from `../build_skybeta_copy.py`'s own `build()`, pointed at the variant. |
| `../upload_to_flyx_cornerfix_slow.py`, `../upload_to_flyx_cornerfix_slow_skybeta.py` | Variant **slow** (native PX4 / Sky Beta) |
| `../upload_to_flyx_cornerfix_view.py`, `../upload_to_flyx_cornerfix_view_skybeta.py` | Variant **view** (native PX4 / Sky Beta) |

```
python3 jolie_course/cornerfix/build_cornerfix.py      # regenerates all four
```

Both variants leave everything else in her file unchanged: the yaw cap of 0.25 rad/s, turning in
place beyond 60 deg, the carrot that shrinks while turning, pad centring, depth handling and PX4
I/O. Each embedded module stays byte-identical apart from the marked `cornerfix` insertions:
- a helper block and one `corner_rule(...)` call in `pathplan.planner_process` (the planner child);
- two `MissionConfig` fields and three lines in `Mission._carrot_for` (`flyx.mission`).

Each plan is journalled with `reason` = `corner-slow` / `corner-view` / `corner-view-slow`.
`git diff --no-index upload_to_flyx.py upload_to_flyx_cornerfix_view.py` shows the whole change.

## 1. What happens at the first corner (existing flights)

Data: v200-v214 (her file, 14/15 completed; v209 was the crash) and v128-v142 (suraj_course).
Sources per run: `truth-poses.csv`, `vio-odometry.jsonl` (OpenVINS, aligned to truth at
lift-off), `vio-feature-clouds.jsonl` (SLAM / MSCKF features per update) and the journal.
"What the camera sees" is a ray along the truth heading against the course OBJ plus the net.
Scripts: `corner_analysis.py` (per-run corner features; also used by the A/B rows) and
`kinsim.py` (the offline kinematic check below), both in this folder.
Corner time *tc* = the truth heading crosses 45 deg.

**Kinematics do not separate good from bad passes in her file.** All 15 flights fly the corner
almost identically:
- 0.31-0.35 m/s mean speed in the 3 s before tc;
- yaw rate 0.25 rad/s mean, about 0.30 rad/s peak;
- minimum clearance 0.315-0.33 m;
- at tc the camera faces the net at 0.71-0.83 m (0.56 m in v214).

v209 (the crash) is not an outlier on any of these. Its corner pass was 0.31 m/s, 0.24 rad/s,
0.317 m clearance and a 0.83 m view. Suraj's "yaw while moving" predictor holds for his program,
which yaws at up to 0.8 rad/s. Her capped turn sits below that level in every run, so it cannot
explain v209.

**What separates them is feature starvation while the nose sweeps across the near net.** Over
whole flights, OpenVINS's SLAM feature count falls steadily as the facing surface gets closer:

| camera faces a surface at | > 2 m | 1.4-2 m | 1.0-1.4 m | 0.7-1.0 m |
|---|---|---|---|---|
| SLAM features (net, turning) | 67 | 50 | 37 | 19 |
| SLAM features (wall, turning) | 32 | 32 | 20 | 11 |

At the first corner, the drone spends about 3 s turning with the camera on the net 0.75-0.95 m
away. That is the "0.7-1.0 m" bin: SLAM features fall from about 75 in the corridor to 10-20.
v209 is the one flight where they hit zero:
- SLAM + MSCKF features over [tc-0.5, tc+1.5] s: **0.4 in v209**, 8-23 in the 14 completed flights;
- lowest 1-s SLAM+MSCKF bin while in the corner box: **0.2 in v209**, 5.6-16 in the others.

The VIO error starts growing at tc-1 s, exactly when features vanish. It goes 0.03 m at tc-1,
0.10 m at tc, 0.8 m at tc+3 and 2.5 m at tc+6. That is a smooth 0.23 m/s drift that PX4 follows.
OpenVINS's estimated accelerometer bias runs away at the same time (|ba_y| reaches 0.43 m/s^2
in v209; at most 0.18 elsewhere).

The suraj_course divergers v131 and v134 also lost every SLAM feature at that corner (MSCKF
features 1.7-7). Most of his non-diverging runs had near-zero SLAM features there too, so on his
program this is necessary but not sufficient. v214's 0.21 m wall pass is a different failure: a
0.1 m VIO offset was already present before the corner.

Conclusion: the corner itself (camera sweeping a near, feature-poor surface for about 3 s) puts
OpenVINS at the edge of starvation in every flight. Divergence is the tail of that distribution.
The lever is what the camera faces, not just speed or yaw rate.

Side finding: her yaw limiter runs on the wall clock (`time.monotonic`). At the GB10 real-time
factor of about 0.83, the drone therefore yaws at about 0.30 rad/s of sim time, not 0.25. On the
drone (real time) it is 0.25. Not changed here.

## 2. Variants (stated as rules, no coordinates)

**Corner mode** is on when either holds:
- the planned route turns by more than 45 deg within 1.2 m ahead (direction measured over 0.3 m);
- the nose is still more than 20 deg off the planner's heading.

Corner mode stays on while the route ends within 1.2 m. That end is a checkpoint goal, and the
planner cannot yet see the next leg past it.

**View distance** is measured on the planner's own occupancy map (the published walls plus the
border net): how far the forward camera sees along a heading, capped at 2 m.

- **slow:** in corner mode, if the view along the current or the commanded heading is less than
  1.5 m, the carrot is halved (floor 0.12 m), so the drone flies at about half speed. Heading is
  unchanged.
- **view (perception-aware heading):** in corner mode, while the route turns ahead (not merely
  while the nose is still turning), the commanded heading may move up to 40 deg further into the
  turn, in 10 deg steps:
  - it takes the first heading that sees at least 1.5 m, or else the heading that sees farthest
    (each extra step must gain more than 0.1 m);
  - within one corner the command never swings back against the turn;
  - the 40 deg limit keeps it inside her 60 deg turn-in-place margin;
  - then it slows down as **slow** does if the view is still short.

  In effect, the nose turns toward the next leg early and slowly, while still in the corridor.
  The camera then faces M2 or the far net at about 1.4 m instead of the net corner at 0.75 m.

The yaw-rate cap stays 0.25 rad/s, the maximum.

Offline kinematic check (`kinsim.py`; point mass, no sensing) over the first corner:

| program | camera < 1.0 m from a surface while turning | min view while turning | whole course |
|---|---|---|---|
| base | 3.9 s | 0.80 m | 70 s |
| slow | 2.2 s | 0.77 m | 84 s |
| view | 0.0 s | 1.13 m | 79 s |

Over the whole course, view adds some yaw: 8 yaw-rate reversals instead of 2, and 9 % more total
yaw. It also puts the nose up to about 60 deg off the motion direction for short stretches.

## 3. A/B in Gazebo (v300-v333, GB10 queue, at most 2 of these flights at a time)

**Run order and validity:**
- Arms were interleaved and rotated.
- GB10 was overloaded (load 25-40, from other jobs) during v300-v305. Per the coordinator, flights
  with mean load ≥ 20 are **invalid**: v301, v303, v304, v305.
- v300 and v302 never started (load about 35).
- The feeder then queued only at load below 18 and logged the load every 30 s. Valid flights ran
  at mean load 12-19.5 (the baseline v200-v214 ran at about 14).
- v304 was also the earlier view build, before the "never swing back" rule was added.
- **Three base flights never left the ground** (v309, v318, v324). The estimator was invalid or
  the takeoff timed out, with truth z unchanged. All code before NAVIGATE is identical in the
  three files, so these are rig start-up failures. They are excluded and were re-flown. That all
  three fell on the base arm is probably chance (p of about 0.04 if the rate were equal), but
  watch for it.

| arm | valid airborne flights | completed (95% CI) | VIO divergences (>0.3 m), any / at corner | NAVIGATE s, mean ± sd | worst truth min clearance | camera view at tc | SLAM features [tc-0.5, tc+1.5], mean ± se | lowest 1-s SLAM+MSCKF in corner box, mean (min) |
|---|---|---|---|---|---|---|---|---|
| base v200-v214 (earlier) | 15 | 14/15 (0.70-0.99) | 1 / 1 | 65.8 ± 5.7 | 0.054 m (v209 crash) | 0.76 m | 12.8 ± 1.3 | 8.5 (0.2) |
| **base** (this A/B) | 9 | 9/9 (0.70-1.00) | 0 / 0 | 64.6 ± 4.4 | 0.210 m | 0.78 m | 13.2 ± 0.9 | 9.8 (4.5) |
| **slow** | 8 | 8/8 (0.68-1.00) | 0 / 0 | 76.2 ± 2.1 | 0.283 m | 0.85 m | 30.4 ± 2.2 | 20.3 (11.3) |
| **view** | 8 | 8/8 (0.68-1.00) | 0 / 0 | 71.7 ± 2.7 | 0.229 m | 1.39 m | 39.9 ± 3.1 | 22.6 (14.2) |

Divergence rates, 95 % Wilson intervals:
- base, all 24 airborne flights: 1/24 (0.01-0.20);
- slow: 0/8 (0-0.32);
- view: 0/8 (0-0.32).

The A/B saw no divergence in any arm. **The flights cannot show a lower divergence rate.**
Proving a drop from about 4 % would need on the order of 100+ flights per arm.

What they do show, with large margins:
- **Feature starvation at the corner is gone in both variants.** The lowest feature count in the
  corner roughly doubles (base 9.8 → slow 20.3, view 22.6). The worst flight of each variant
  (11.3, 14.2) is above the best-but-one base flights. v209's 0.2 is the failure mode.
- **view moves the camera off the near net.** The view at tc goes from 0.78 m to 1.39 m in every
  flight, and SLAM features at the corner triple (13 → 40).
- **Cost.** NAVIGATE is +7.1 s (+11 %) for view and +11.6 s (+18 %) for slow, which slows every
  near-wall turn. Both are far inside the 175 s limit.
- **Clearance.** No variant flight went below 0.229 m (base this A/B: 0.210 m). VIO error 6 s
  after the corner: median 13-18 mm in all arms.

Per-flight rows: run, arm, mean load, status, completed, checkpoints, NAVIGATE (sim s),
truth min clearance, max VIO error over the flight, then view (m) / SLAM / MSCKF features at the
corner.

| run | arm | load | status | ok | cp | NAV s | min clr m | VIO max m | corner view / SLAM / MSCKF |
|---|---|---|---|---|---|---|---|---|---|
| v300 | (not started) | - | rig: never airborne | 0 | - | - | - | - | - |
| v301 | slow | 40.5 | overload | 0 | 1 | 20.33 | 0.181 | 1.558 | 0.86 / 36 / 2.8 |
| v302 | (not started) | - | rig: never airborne | 0 | - | - | - | - | - |
| v303 | slow | 26.1 | overload | 0 | 0 | - | 0.391 | - | - |
| v304 | view | 23.1 | overload | 0 | 3 | 168.11 | 0.126 | 0.303 | 1.64 / 66 / 1.4 |
| v305 | base | 25.9 | overload | 1 | 5 | 57.05 | 0.302 | 0.075 | 0.62 / 2 / 4.1 |
| v306 | view | 18.7 | valid | 1 | 5 | 76.57 | 0.308 | 0.076 | 1.37 / 30 / 1.4 |
| v307 | base | 19.5 | valid | 1 | 5 | 58.16 | 0.301 | 0.071 | 0.83 / 14 / 2.2 |
| v308 | slow | 19.4 | valid | 1 | 5 | 75.8 | 0.297 | 0.066 | 0.77 / 21 / 3.7 |
| v309 | base | 20.1 | rig: never airborne | 0 | 0 | 17.28 | - | - | - |
| v310 | slow | 13.4 | valid | 1 | 5 | 77.35 | 0.293 | 0.07 | 0.86 / 34 / 3.2 |
| v311 | view | 13.6 | valid | 1 | 5 | 73.29 | 0.293 | 0.06 | 1.34 / 39 / 1.8 |
| v312 | slow | 12.3 | valid | 1 | 5 | 72.46 | 0.299 | 0.048 | 0.86 / 31 / 1.5 |
| v313 | view | 13.5 | valid | 1 | 5 | 67.27 | 0.303 | 0.062 | 1.37 / 39 / 1.3 |
| v314 | base | 15.2 | valid | 1 | 5 | 63.06 | 0.304 | 0.06 | 0.80 / 13 / 2.6 |
| v315 | view | 17.8 | valid | 1 | 5 | 69.51 | 0.304 | 0.066 | 1.40 / 40 / 1.7 |
| v316 | base | 16.1 | valid | 1 | 5 | 63.1 | 0.318 | 0.078 | 0.77 / 14 / 2.8 |
| v317 | slow | 17.2 | valid | 1 | 5 | 75.62 | 0.294 | 0.051 | 0.83 / 28 / 3.6 |
| v318 | base | 18.1 | rig: never airborne | 0 | 0 | - | - | - | - |
| v319 | slow | 19.1 | valid | 1 | 5 | 74.49 | 0.301 | 0.041 | 0.89 / 33 / 2.4 |
| v320 | view | 17.6 | valid | 1 | 5 | 73.03 | 0.29 | 0.065 | 1.34 / 33 / 1.5 |
| v321 | slow | 18.8 | valid | 1 | 5 | 78.47 | 0.299 | 0.045 | 0.83 / 23 / 4.4 |
| v322 | view | 16.1 | valid | 1 | 5 | 71.25 | 0.229 | 0.062 | 1.46 / 58 / 2.6 |
| v323 | base | 17.1 | valid | 1 | 5 | 73.94 | 0.32 | 0.072 | 0.80 / 12 / 4.7 |
| v324 | base | 15.8 | rig: never airborne | 0 | 0 | 17.36 | - | - | - |
| v325 | slow | 13.0 | valid | 1 | 5 | 78.75 | 0.29 | 0.046 | 0.92 / 33 / 1.9 |
| v326 | view | 15.1 | valid | 1 | 5 | 71.35 | 0.295 | 0.047 | 1.43 / 46 / 1.6 |
| v327 | base | 16.9 | valid | 1 | 5 | 63.96 | 0.21 | 0.067 | 0.71 / 10 / 3.2 |
| v328 | base | 16.1 | valid | 1 | 5 | 62.94 | 0.28 | 0.075 | 0.77 / 12 / 4.2 |
| v329 | slow | 12.5 | valid | 1 | 5 | 76.73 | 0.283 | 0.09 | 0.83 / 40 / 2.9 |
| v330 | view | 14.1 | valid | 1 | 5 | 71.69 | 0.293 | 0.06 | 1.43 / 34 / 2.2 |
| v331 | base | 14.0 | valid | 1 | 5 | 62.39 | 0.29 | 0.062 | 0.77 / 10 / 3.6 |
| v332 | base | 17.2 | valid | 1 | 5 | 66.12 | 0.298 | 0.051 | 0.80 / 17 / 2.2 |
| v333 | base | - | valid | 1 | 5 | 67.72 | 0.315 | 0.061 | 0.74 / 17 / 4.7 |

v333's load was not logged (the logger stopped with the plan); load was 17.4 when it was queued.
Scoring: `jolie_course/gazebo/score_run.py` (branch `gazebo-jolie`), plus the corner analysis
scripts used for section 1.

## 4. Recommendation

**Adopt `view` as the candidate fix** and keep `slow` as the simpler fallback. Do not claim yet
that either one lowers the divergence rate.

- **Why view:**
  - It targets the measured mechanism: the camera on a near, feature-poor surface during the turn.
  - It removes the starvation that preceded the only crash of her file in Gazebo: the corner
    feature floor goes from about 10 (minimum 0.2) to about 23 (minimum 14).
  - It costs less time than slow (+7 s versus +12 s).
  - It is a map-based rule with no hard-coded coordinates.
- **Uncertainty:**
  - 0 divergences in 8 view flights is compatible with anything from 0 to 32 %. The base rate is
    about 4 % (1/24).
  - The evidence for a benefit is the strong proxy (feature floor, view distance), not the outcome.
- **Before hardware:**
  - **Real camera and VIO.** The drone runs cuVSLAM (stereo) and sees real textures. Check its
    feature or track counts at this corner with base versus view in FlyX.
  - **Depth.** These Gazebo runs had no depth camera (depth frames 0). The view rule points the
    nose up to 40 deg into the turn (briefly about 60 deg off the motion direction), so the depth
    camera sees less of the path ahead. Re-check with the depth-camera rig.
  - **Yaw.** It adds some yaw reversals (all capped at 0.25 rad/s).
  - **Approval.** Team approval before arming, as always.
