# Sweep results: Jolie's FlyX-proven planner (offline, 2026-11-07)

Everything below comes from the offline closed-loop sim in this folder ([README](README.md)). It
runs the exact embedded mission and planner code from `jolie_course/upload_to_flyx.py`. **None of
these candidates has flown.** Each one must pass Gazebo or FlyX shadow and real-physics runs, and
the team must approve it, before it arms.

About 6,000 runs on GB10, in four stages:
- calibration;
- coarse: 128 Sobol configs plus the base, on levels L0, L2 and L3, 5 seeds each;
- refine: 10 Pareto/ranked configs with 5 jitters each, on C0, C2 and C3, 8 seeds;
- finals: 8 configs on C0, C1, C2, L1 and L2, 40 seeds each, with common random numbers.

Raw data is in `results/*_runs.csv`. The finals table is `results/final_table.md` and the failure
map is `results/failmap_table.md`.

## Calibration check

| Setting | Base NAVIGATE (mean) | Reference |
|---|---|---|
| L2 (FlyX-like stalls/pauses) on the lab clock, rtf 0.8 | **106.6 s** wall (24 seeds) | FlyX: **120.9 s** lab wall clock (= ~97 s sim time) |
| L0, rtf 0.8 | 103.5 s | within the 100-150 s target |
| C0, calibrated dynamics (Kp 0.952, tau 0.83, vel_i 0.66, delay 0) | **77.5 s** (40 seeds, 100 % complete) | Gazebo real-physics baseline: 65.8 s mean, 14/15 complete |

The sim runs about 12 % faster than FlyX at FlyX-like conditions, and about 18 % slower than
Gazebo at the calibrated nominal. It sits between the two references, so I made no speed
correction. The FlyX number is lab wall clock: the program and its 5 Hz planner run on the wall
clock, while physics runs at about 0.8x. This is modelled with `Level.rtf`.

**One sim fix was found on the way.** With an exactly noise-free estimate, the flown planner can
freeze. The drone settles perfectly still on a degenerate plan and gets the same carrot forever:
it never moves, so it never replans. This happened in 20/20 runs at tau 0.45. Gazebo-level jitter
(0.6 mm) removes it, so C0 now carries that jitter. On the real drone, noise makes the freeze
unlikely, but the latent case exists in the code.

## Results: base vs candidates (40 seeds per level, same worlds and noise for every config)

Completion = reached the pad without a crash (true wall clearance < 0.25 m) within the flight
limit. Time = NAVIGATE mean for completed runs. "VIO ev" = simulated visual-odometry jump or loss
events per run.

| config | C0 compl / time | C1 compl / time | C2 compl | L1 compl / time | L2 compl / time | C0 min wall / p5 clear (m) | L2 VIO ev |
|---|---|---|---|---|---|---|---|
| **base (proven)** | 100 % / 77.5 s | 98 % / 84.7 s | 20 % | 95 % / 81.4 s | 58 % / 92.5 s | 0.368 / 0.391 | 0.15 |
| **fast** (s048) | 100 % / **51.8 s** | 98 % / 56.0 s | 32 % | 98 % / 58.3 s | 60 % / 67.7 s | 0.355 / 0.402 | 0.72 |
| **safe** (s086) | 100 % / 80.2 s | 98 % / 89.0 s | 28 % | 98 % / 86.2 s | **72 %** / 95.1 s | 0.377 / 0.407 | 0.15 |
| **balanced** (s048r3) | 100 % / **49.5 s** | 92 % / 56.5 s | 30 % | **100 %** / 57.7 s | **72 %** / 67.4 s | 0.371 / 0.413 | 0.10 |
| **clearance** (s029r2) | 100 % / 72.1 s | 92 % / 76.0 s | **35 %** | **100 %** / 76.1 s | 68 % / 85.5 s | **0.392 / 0.411** | 0.12 |

- **fast** and **safe** have completion ≥ base at **every** level.
- **balanced** and **clearance** are better at four levels but lose 2 runs out of 40 at C1. That
  is within noise, but they do not strictly meet the ≥-base rule.
- Clean runs (no prop-tip touch < 0.28 m) at C1: base 82 %, fast 88 %, safe 90 %, balanced 85 %.
- Rejected: s019r2. It is the fastest (43.6 s), but it touches the net in 40/40 C0 runs and has
  0.5 VIO events per run.

| knob | proven | fast | safe | balanced | clearance |
|---|---|---|---|---|---|
| carrot_m | 0.5 | 0.695 | 0.423 | 0.69 | 0.423 |
| slow_turn_deg | 60 | 71.4 | 72.0 | 68.4 | 116.4 |
| min_carrot_frac | 0.3 | 0.53 | 0.386 | 0.519 | 0.599 |
| yaw_rate_max | 0.25 | 0.321 | 0.219 | 0.292 | 0.21 |
| TURN_IN_PLACE_DEG | 60 | 76.9 | 70.8 | 86.2 | 92.1 |
| lookahead_m | 0.7 | 1.184 | 0.977 | 1.132 | 1.173 |
| commit_margin | 0.10 | 0.24 | 0.169 | 0.238 | 0.179 |
| replan_every_m | 0.5 | 0.768 | 0.274 | 0.697 | 0.789 |
| turn_step_deg | 15 | 25.3 | 19.5 | 23.6 | 21.4 |
| tracking_margin | 0.08 | 0.083 | 0.043 | 0.068 | 0.046 |
| soft_margin / soft_weight | 0.15 / 4 | 0.124 / 6.16 | 0.298 / 9.42 | 0.162 / 6.0 | 0.30 / 4.68 |
| WALL_SNAP_M | 0.25 | 0.148 | 0.115 | 0.177 | 0.212 |
| HIT_CONFIRM | 3 | 6 | 3 | 6 | 6 |
| suspect_penalty | 8 | 9.9 | 6.5 | 8.75 | 5.6 |
| MAX_TURN_RATE | 0.5 | 0.328 | 0.418 | 0.308 | 0.574 |
| pad_centre_m | 0.08 | 0.092 | 0.067 | 0.085 | 0.114 |

What drives the results (Spearman rank correlation over the 128 coarse configs):
- `tracking_margin` dominates. A larger margin gives lower completion (ρ = -0.72) and a slower
  course (+0.59), but more clearance when the run does finish (+0.62). At 0.08 the M1 passage
  leaves about a 0.2 m free band, and the drone loops on "escape". A thin hard margin plus a wide
  soft margin centres the drone instead (safe and clearance use this).
- `carrot_m` is the speed knob (ρ = -0.46 on time).
- A larger `WALL_SNAP_M` helps completion (+0.20).
- The fast configs share: a long look-ahead (about 1.1-1.2 m), a high minimum carrot fraction
  (about 0.5), `HIT_CONFIRM` 6, and `MAX_TURN_RATE` about 0.3.

## Failure map: base, on the calibrated nominal C0, one axis at a time (20 seeds; C0 row 40)

| Perturbation | Holds up to | Breaks at | How it breaks |
|---|---|---|---|
| Wall shifts (±, along and across) | 0.10 m: 100 % | 0.15-0.20 m: 95 %, clean 70-80 %; 0.25 m: 85 % | touches and crashes in the maze before Gate A |
| Unmapped poles near the route | 2 poles: 95-100 % | 3: 65 %; 4: 50 % | crashes and "planner failed: no path" |
| cuVSLAM drift (m/√m, plus noise) | 0.01: 100 % | 0.02: 90 %; 0.04: 45 %; 0.06: 0 % | crashes in the maze and chicane; **the most damaging axis** |
| Link delay | 9 steps (0.45 s): 100 % | none | only slower (+25 s at 9 steps) |
| Program stalls | 1.5/s, 15 % long ≤3 s: 100 % | none | slower (+21 s) |
| Odometry pauses | every 5 s 0.3-1.4 s, or every 10 s 1-2.5 s: 100 % | none | slightly slower |
| PX4 velocity lag tau | 1.05 s: 100 % | 1.3 s: 100 % but clean 20 %; 1.6 s: 45 % | overshoot into walls (fails after cp 1 and cp 3) |
| PX4 Kp | 0.7 to 1.2: 100 % | none | Kp 1.2 is 8 s faster |
| VIO hazard (yaw-rate × speed model) | x2: 100 % | x4 and x8: 95 % | a loss over 3 s means "odometry stale", then land |
| Depth noise σ = k·z² | k 0.02: 100 % | k 0.04: 10 % | phantom obstacles: "no goal cell", flight time limit |
| Depth/pose time lag | 4 ticks (0.2 s): 100 % | none | none |

Combined levels break the base harder than any single axis does:
- C2 (the YAML's full ranges, including 2x drift, up to 0.054 m/√m): 20 %.
- L2 (FlyX-like): 58 %.

Every candidate improves C2 and L2, but nothing survives the C2 drift reliably. The weak spot is
the **first corridor to the maze to Gate A** (most failures are "after cp 1"), then the chicane
("after cp 3").

## Caveats: what the offline sim cannot capture

- **Kinematic world, 2-D slab.** Contact is a disc-centre clearance against a 1 cm grid (about
  1 cm accuracy). There is no aerodynamics, no wall wash, and no tilt-dependent footprint. The net
  is analytic and touching it does not end a run.
- **PX4 is `team_mission.rl.dynamics`**: a linear P/PI loop with saturation. It does not model the
  real jerk-limited position controller, failsafe modes during stalls over 1 s (the sim holds the
  last setpoint), or estimator resets.
- **VIO loss is a hazard model.** The yaw-rate × speed form and its rates come from 4/15 Gazebo
  divergences, not from cuVSLAM itself. The fast config raises the yaw cap to 0.321 rad/s and has
  more VIO events under the yaw-rate-only model (0.72 per run at L2), so **check VIO health at the
  first 90-degree corner first** when it flies.
- **The depth camera is a 640x400 render** against an extruded slab: no stereo artefacts, no
  textureless dropouts, floor and wall returns only. Planner compute latency is a fixed model.
- **The base crashes more in the sim than in reality.** At C0 it is 100 %, matching Gazebo's
  14/15. At C1 and L2 the base already loses runs, so the absolute rates at C2 and L2 are
  pessimistic. **Trust the ranking more than the absolute numbers.**
- Knob values are the exact tested values (3 decimals), not rounded.

## Candidate files (built from upload_to_flyx.py; only these constants change)

`python3 jolie_course/candidates/build_candidates.py` reads `sweep/results/candidates.json`. It
checks that the embedded sources equal what the sweep ran, and that no other line changed.

- `jolie_course/candidates/upload_to_flyx_fast.py`: 17 constants
- `jolie_course/candidates/upload_to_flyx_safe.py`: 16 constants
- `jolie_course/candidates/upload_to_flyx_balanced.py`: 17 constants
- `jolie_course/candidates/upload_to_flyx_clearance.py`: 17 constants

Copies are in `~/dronehacks-2026-gb10/jolie_flyx/` on GB10. They are not queued: pick free run
numbers, then append lines such as `NNN jolie_flyx/upload_to_flyx_fast.py` to
`jolie_flyx/queue.txt`.
