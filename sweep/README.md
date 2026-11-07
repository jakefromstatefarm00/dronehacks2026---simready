# sweep/: offline tuning and failure map for Jolie's FlyX-proven planner

This folder tunes the program in `jolie_course/upload_to_flyx.py`, the planner that completed the
FlyX course (NAVIGATE 120.9 s lab wall clock, pad 0.07 m from centre). It also maps where that
planner breaks when the world is perturbed. Everything here is offline: a kinematic course,
a PX4 and cuVSLAM response model, and simulated depth. Nothing here flies. Candidate files must
go through Gazebo or FlyX shadow runs, and the team must approve them, before anything arms.

Results and the tables are in [`RESULTS.md`](RESULTS.md).

## What is simulated, and what runs unchanged

| Part | Source |
|---|---|
| Mission state machine: carrot, slow-turn carrot, yaw cap, odometry-pause hold, pad centring, flight limit | `flyx.mission` embedded in `upload_to_flyx.py`, **exact**: `Mission.tick()` at 20 Hz, `Mission.plan()` at 5 Hz |
| Planner process: depth votes, `WALL_SNAP_M`, `HIT_CONFIRM`, `drop_misregistered`, `safe_carrot_path`, `TURN_IN_PLACE_DEG` | `pathplan.planner_process.planner_process()` **itself**, run in a thread through a queue pipe (signal/nice stubbed) |
| Theta\* replanning, belief map, look-ahead, commitment, look-around, escape, doubt-the-map | `pathplan.online/belief/grid_search/world`, **exact** |
| Depth projection | `fastdepth.py`: same output as the embedded `project_depth`, bit for bit, about 5x faster. Each worker checks it against the original at start-up |
| PX4 position and velocity loop, yaw loop, link delay, cuVSLAM drift and noise | `team_mission.rl.dynamics.DroneSim`, with parameters drawn per level |
| Depth camera | a 640x400 32FC1 image (72 deg HFOV) rendered from the **true** pose against the **true** perturbed course (walls from 0 to 1.727 m, floor). The planner projects it with the **estimated** pose, so drift, noise and stale poses misregister depth the way they do on the drone. Frames are dropped while turning faster than `MAX_TURN_RATE` and during odometry gaps, as `on_depth` does |
| Program stalls | the whole program freezes: no ticks and no planning. PX4 holds the last setpoint |
| Odometry pauses | no odometry for 0.3-1.4 s. The mission holds the last setpoint and the planner sees a stale pose |
| Visual-odometry loss risk | a hazard driven by the **true** yaw rate: 0.20/s while above 0.40 rad/s, ramping in from 0.30 rad/s. Turning in place adds 0.012/s (speed < 0.1 m/s, yaw rate > 0.1 rad/s), and there is a 0.001/s floor, all scaled per level. Each event is an estimate jump (N(0, jump_m) per axis, 3 deg yaw) plus a 0.3-1 s pause. 20 % of events are losses: a 1.5-4 s pause, and above 3 s the mission lands |
| Planner compute latency | deterministic: 20 ms, plus 80 ms per depth frame, plus 30 ms per replan. This is about 1.3x the flown code's time measured on GB10 with `calibrate_latency.py` |
| Evaluation | true clearance of the disc centre, from a KD-tree on a 1 cm truth grid, accurate to about 1 cm. A **touch** is less than 0.28 m (the Sky Beta prop-tip radius); touches are counted. A **crash** is less than 0.25 m and ends the run. The net is analytic: a net touch is counted but does not end the run, because the FlyX run was not stopped by it |

Every knob is applied as an **exact text substitution** of one constant in the embedded source
(`embedded.KNOBS`). The candidate builder applies the same table, so a candidate file flies
exactly the code that was simulated.

## Levels (common random numbers: every config sees the same worlds and noise per seed)

| Level | Walls | Clutter | PX4 lag tau / Kp / delay | cuVSLAM drift | Stalls | Odometry pauses | VIO hazard x |
|---|---|---|---|---|---|---|---|
| L0 nominal | published | none | 0.5 s / 0.95 / 1 step | none | none | none | 0 |
| L1 slight | ±5 cm | 0-1 | 0.35-0.6 / 0.9-1.0 / 1-2 | ≤0.005 m/√m | 0.3/s, ≤1 s | 1 per 60 s, 0.3-0.6 s | 0.5 |
| L2 FlyX-like | ±10 cm | 1-2 | 0.35-1.0 / 0.85-1.05 / 1-3 | ≤0.01 | 0.63/s (86 per 137 s), 3 % long ≤1.95 s | 1 per 25 s, 0.3-1.4 s | 1 |
| L3 heavy | ±15 cm | 1-3 | 0.5-1.2 / 0.8-1.1 / 2-4 | 0.005-0.02 | 0.8/s, 5 % ≤2.5 s | 1 per 12 s, 0.3-2 s | 2 |
| L4 severe | ±20 cm | 2-4 (larger) | 0.6-1.4 / 0.75-1.15 / 3-5 | 0.01-0.03 | 1/s, 8 % ≤3 s | 1 per 8 s, 0.3-2.5 s | 3 |

The failure map (`failmap.py`) starts from L0 and raises **one** axis at a time.

## Knob ranges (why)

| Knob (where) | Proven | Range | Why this range |
|---|---|---|---|
| `carrot_m` (mission + `--carrot`) | 0.5 | 0.30-0.90 | Speed ≈ Kp x carrot: 0.28-0.85 m/s. Below 0.3 the drone crawls. Above 0.9 `safe_carrot_path` pulls it back in most corridors anyway |
| `slow_turn_deg` (mission) | 60 | 30-120 | The heading error at which the carrot reaches its minimum. 30 brakes on every bend; 120 barely slows |
| `min_carrot_frac` (mission `_carrot_for`, the literal `.3`) | 0.3 | 0.15-0.70 | How hard to brake in a turn |
| `yaw_rate_max` (mission) | 0.25 | 0.15-0.40 | VIO evidence: 30 of 40 losses above 0.35 rad/s, and 3/3 Gazebo completions at 0.25 |
| `turn_in_place_deg` (planner_process) | 60 | 30-100 | 30 was the old planner value (21-26 stops per flight). Above 90 the drone moves sideways or backwards relative to its nose |
| `lookahead_m` (PlannerConfig) | 0.7 | 0.4-1.2 | Where the nose points, and so what the camera sees and what turn-in-place triggers on |
| `commit_margin` | 0.10 | 0-0.30 | Route flip-flop against sticking to a worse route |
| `replan_every_m` | 0.5 | 0.25-1.0 | Replan cadence |
| `turn_step_deg` | 15 | 8-30 | Turn step for look-before-moving and look-around |
| `tracking_margin` (Envelope) | 0.08 | 0.04-0.14 | Hard keep-out = 0.28 + this. The M1 passage leaves about 0.2 m of free band at 0.08 |
| `soft_margin`, `soft_weight` (Envelope) | 0.15, 4 | 0.05-0.30, 1-10 | How strongly routes are pushed off walls |
| `wall_snap_m` (planner_process) | 0.25 | 0.10-0.35 | Depth returns this close to a published wall count as that wall |
| `hit_confirm` (planner_process) | 3 | 2-6 | Votes before a new obstacle exists: 2 means one frame, 4 means two in a row, 5-6 means three |
| `suspect_penalty` | 8 | 2-16 | Cost behind unexpected hits |
| `max_turn_rate` (planner_process) | 0.5 | 0.3-0.8 | Depth frames are dropped while turning faster than this |
| `pad_centre_m` (mission) | 0.08 | 0.05-0.15 | Pad centring tolerance before Land |

## Running

On GB10, under `~/dronehacks-2026-gb10/sweep-jolie/`, with at most 12 workers under `nice -n 10`:

```
cd sweep
nice -n 10 python3 run_sweep.py calib   --seeds 24 --out results/calib      # base, L0/L2 at rtf 1.0 and 0.8
nice -n 10 python3 run_sweep.py coarse  --n 160 --levels L0 L2 L3 --seeds 6 --out results/coarse
python3 select_configs.py results/coarse/runs.csv --configs results/coarse/configs.json --refine 12 --per 8 --out results/refine_configs.json
nice -n 10 python3 run_sweep.py configs --configs results/refine_configs.json --levels L0 L2 L3 --seeds 12 --out results/refine
nice -n 10 python3 run_sweep.py configs --configs results/finalists.json --levels L0 L1 L2 L3 L4 --seeds 50 --out results/final
nice -n 10 python3 run_sweep.py failmap --seeds 30 --out results/failmap
python3 analyze.py results/final/runs.csv
python3 sim.py --level L2 --seed 3 --knob carrot_m .6          # one run, JSON out
```

`runs.csv` is append-only and resumable. Truth worlds are cached in `cache/` (git-ignored).
Candidates: `python3 jolie_course/candidates/build_candidates.py` reads
`sweep/results/candidates.json`.
