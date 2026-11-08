# Map alignment (VIO drift correction): offline A/B and decision

Branch `drift-eval`. This compares Jolie's FlyX-proven planner (`jolie-flyx-sources` e1ac88d,
= `jolie_course/upload_to_flyx.py`) with her map alignment on top of it (`jolie-drift-correction`
f7b97d3). Everything here is new. Her branches and the frozen files were not touched.

## How the alignment works

Each depth frame (13 Hz) is projected from the reported pose. The hit points within 0.35 m of a
published wall then go through a Gauss-Newton fit on the signed distance field of the published
walls, sampled bilinearly, for at most 5 iterations. The fit corrects a direction only if the walls
constrain it (Hessian eigenvalue > max(10, 2 % of the hits)), so a straight corridor corrects only
sideways. It refuses frames with too few hits (< 40), no rms gain (< 3 mm), or a shift > 0.3 m.
At each 5 Hz planning step, `fly_pathplan` moves the course frame's origin by 0.6 x the measured
shift. Each step moves it at most 4 cm, and the total stops at 1.5 m. Planning, setpoints and the
next frames' projection all use the moved frame. The method is a proportional observer that pulls
the frame until depth sits on the published walls.

Failure modes, by construction:
* **Moved / unmapped geometry:** it aligns to the published map. If the walls in view moved
  coherently (or one wall is all it sees), a moved wall looks exactly like drift. The frame then
  takes on that wall's offset (up to the wall shift) until other walls pull it back. The planner's
  `drop_misregistered` already ignores depth within 0.25 m of a published wall, so neither arm
  sees a wall that moved < 25 cm.
* **Noisy depth:** single-frame shifts are noisy. The 4 cm / step clip limits each kick, but kicks
  that take the error past the capture range (0.35 m band, 0.3 m refusal) lock onto the wrong wall.
  The error then grows to the 1.5 m cap.
* **Degenerate views:** in a long corridor the along-corridor direction is never corrected. That is
  correct, but the error there keeps growing until a cross wall is seen. Grazing side walls give
  little evidence.
* **Mount offset / mount yaw / heading drift:** a wrong camera offset makes a constant bias, which
  the frame absorbs (a few cm). Heading error is not modelled: it shows up as a translation that
  changes as the view changes.

## The offline A/B (`ab_sim.py`)

* Runs each branch's own `flyx.mission.Mission` (state machine, carrot, yaw cap, pad centring) and
  `pathplan.planner_process.planner_process` (depth projection, wall snap, votes, theta*, and
  `align_to_map` on the drift branch), on a thread with an in-memory pipe.
* `fly_pathplan`'s glue (pose at the frame stamp, turn-rate filter, 5 Hz planning, 20 Hz control,
  `apply_alignment`) is re-implemented line for line. The alignment lands 0.1 s after the plan
  starts, the way the real planning thread does.
* PX4: a P position loop with a velocity lag (0.3 s), 1.5 m/s and 3 m/s², flying the estimated
  pose.
* VIO drift (estimate - truth), as a function of distance flown with common random numbers:
  * a bias of `drift` m/m in a slowly wandering direction, plus white noise;
  * optional heading drift (deg/m) and jumps (N(0, 8 cm) per axis, 1 per 10 m).
* Depth: `fake_px4.render_depth` (640x400, FlyX intrinsics), cast from the TRUE pose on the TRUE
  (perturbed) course. Noise options:
  * noisy: σ = 1 cm + 1.5 %·z², in 8x8-pixel blocks, 5 % dropout, 0.5 % outliers;
  * bad: σ = 2 cm + 3 %·z², 15 % dropout, 2 % outliers.
* Course variants (`pathplan/variants.py`): walls (maze, chicane and approach walls moved ±15 cm),
  walls_big (±30 cm), clutter (1-3 unmapped posts by the route).
* Metrics are on TRUE geometry:
  * contact steps (50 Hz, clearance < 0.28 m);
  * min clearance;
  * completion: landed through the planner's pad logic, all 4 lines crossed by the true track, and
    landed inside the ring;
  * course time;
  * correction applied vs drift, and the residual pose error.

The fake-PX4 kinematics are faster than FlyX: courses here take ~65 s, where FlyX took 120.9 s.

## Results

We ran 1688 simulated flights on the GB10 (3 cores) and the laptop: base vs drift on common random
numbers, then the safeguard candidates. All flights are in
[`results/all_results.jsonl.gz`](results/all_results.jsonl.gz). To regenerate a table:
`python mdtable.py results/all_results.jsonl.gz base,drift[,S7]`.

How to read the cells: `done %` is completion. `contact runs` counts the runs with at least one
50 Hz step closer than 0.28 m to TRUE geometry. `mean min clear (worst)` is the true minimum
clearance in m. `time s` is the course time of the completed runs. `worse / better` counts paired
seeds where the arm had more / fewer contact steps than base, or lost / gained completion.

### Base vs drift (Jolie's f7b97d3), every seed run

| cell | n | base: done % / contact runs / mean min clear (worst) / time s | drift (Jolie): done % / contact runs / mean min clear (worst) / time s | drift (Jolie) vs base: worse / better | correction / drift at end, residual (drift (Jolie)) |
|---|---|---|---|---|---|
| 0 cm/m, nominal | 3 | 100 / 0 / 0.38 (0.38) / 67 | 100 / 0 / 0.38 (0.38) / 64 | 0 / 0 | 0.01 / 0.00, 0.014 |
| 1 cm/m, nominal | 8 | 100 / 4 / 0.29 (0.23) / 67 | 100 / 0 / 0.37 (0.37) / 64 | 0 / 4 | 0.16 / 0.16, 0.015 |
| 2 cm/m, nominal | 30 | 53 / 23 / 0.20 (0.00) / 70 | 100 / 0 / 0.37 (0.37) / 64 | 0 / 23 | 0.31 / 0.31, 0.018 |
| 4 cm/m, nominal | 30 | 0 / 30 / 0.11 (0.00) / - | 100 / 1 / 0.36 (0.27) / 64 | 0 / 30 | 0.63 / 0.62, 0.027 |
| 6 cm/m, nominal | 8 | 0 / 8 / 0.12 (0.00) / - | 100 / 2 / 0.35 (0.26) / 65 | 0 / 8 | 0.97 / 0.98, 0.018 |
| 8 cm/m, nominal | 8 | 0 / 8 / 0.05 (0.00) / - | 88 / 2 / 0.29 (0.00) / 63 | 0 / 8 | 1.33 / 1.32, 0.242 (capped 2) |
| 2 cm/m +jumps 1.0/10m, nominal | 15 | 60 / 12 / 0.17 (0.00) / 69 | 100 / 0 / 0.37 (0.33) / 64 | 0 / 12 | 0.33 / 0.33, 0.043 |
| 2 cm/m +0.1 deg/m, nominal | 9 | 44 / 6 / 0.17 (0.00) / 71 | 100 / 0 / 0.36 (0.34) / 66 | 0 / 6 | 0.33 / 0.34, 0.030 |
| 0 cm/m, nominal, mount +5 cm | 7 | 100 / 0 / 0.38 (0.38) / 64 | 100 / 0 / 0.34 (0.34) / 76 | 0 / 0 | 0.04 / 0.00, 0.035 |
| 0 cm/m, nominal, pose lag 50 ms | 8 | 100 / 0 / 0.38 (0.38) / 71 | 100 / 0 / 0.38 (0.38) / 68 | 0 / 0 | 0.00 / 0.00, 0.003 |
| 4 cm/m, nominal, mount +5 cm | 8 | 0 / 8 / 0.10 (0.00) / - | 100 / 0 / 0.34 (0.33) / 72 | 0 / 8 | 0.64 / 0.64, 0.038 |
| 0 cm/m, nominal, mount yaw +2 deg | 7 | 100 / 0 / 0.37 (0.37) / 63 | 100 / 0 / 0.34 (0.34) / 61 | 0 / 0 | 0.03 / 0.00, 0.033 |
| 4 cm/m, nominal, mount yaw +2 deg | 8 | 0 / 8 / 0.07 (0.00) / - | 100 / 0 / 0.33 (0.33) / 62 | 0 / 8 | 0.65 / 0.65, 0.026 |
| 0 cm/m, nominal, noisy depth | 15 | 100 / 0 / 0.37 (0.37) / 79 | 93 / 1 / 0.29 (0.00) / 92 | 1 / 0 | 0.31 / 0.00, 0.313 |
| 0 cm/m, nominal, bad depth | 10 | 90 / 0 / 0.36 (0.36) / 114 | 0 / 9 / 0.05 (0.00) / - | 10 / 0 | 1.14 / 0.00, 1.139 (capped 4) |
| 4 cm/m, nominal, noisy depth | 15 | 0 / 15 / 0.10 (0.00) / - | 80 / 1 / 0.32 (0.27) / 95 | 0 / 15 | 0.65 / 0.69, 0.228 |
| 0 cm/m, clutter | 30 | 97 / 1 / 0.37 (0.27) / 71 | 100 / 2 / 0.36 (0.19) / 66 | 1 / 2 | 0.01 / 0.00, 0.012 |
| 0 cm/m, walls | 30 | 100 / 15 / 0.28 (0.19) / 67 | 100 / 20 / 0.26 (0.15) / 68 | 18 / 4 | 0.01 / 0.00, 0.012 |
| 0 cm/m, walls_big | 15 | 87 / 11 / 0.24 (0.12) / 67 | 100 / 10 / 0.28 (0.15) / 70 | 4 / 9 | 0.03 / 0.00, 0.030 |
| 1 cm/m, clutter | 9 | 100 / 6 / 0.26 (0.23) / 70 | 100 / 1 / 0.36 (0.24) / 67 | 0 / 6 | 0.17 / 0.17, 0.017 |
| 1 cm/m, walls | 9 | 100 / 7 / 0.27 (0.23) / 68 | 100 / 6 / 0.27 (0.20) / 69 | 4 / 3 | 0.16 / 0.16, 0.017 |
| 2 cm/m, clutter | 30 | 60 / 23 / 0.22 (0.00) / 72 | 100 / 2 / 0.36 (0.19) / 66 | 1 / 22 | 0.32 / 0.32, 0.018 |
| 2 cm/m, walls | 30 | 57 / 25 / 0.20 (0.00) / 71 | 100 / 23 / 0.26 (0.13) / 67 | 8 / 17 | 0.31 / 0.31, 0.019 |
| 2 cm/m, walls_big | 15 | 40 / 15 / 0.12 (0.00) / 75 | 100 / 9 / 0.27 (0.15) / 70 | 1 / 14 | 0.33 / 0.32, 0.034 |
| 4 cm/m, clutter | 30 | 0 / 30 / 0.11 (0.00) / - | 100 / 4 / 0.34 (0.15) / 66 | 0 / 29 | 0.65 / 0.65, 0.022 |
| 4 cm/m, walls | 30 | 0 / 29 / 0.11 (0.00) / - | 100 / 21 / 0.25 (0.15) / 66 | 0 / 28 | 0.64 / 0.63, 0.030 |
| 6 cm/m, clutter | 8 | 0 / 8 / 0.14 (0.00) / - | 100 / 5 / 0.28 (0.17) / 72 | 0 / 8 | 1.01 / 1.01, 0.023 |
| 6 cm/m, walls | 8 | 0 / 8 / 0.13 (0.00) / - | 100 / 5 / 0.28 (0.23) / 68 | 0 / 8 | 1.00 / 0.99, 0.030 |
| 8 cm/m, clutter | 9 | 0 / 9 / 0.04 (0.00) / - | 89 / 2 / 0.29 (0.00) / 66 | 0 / 9 | 1.35 / 1.37, 0.238 (capped 3) |
| 8 cm/m, walls | 8 | 0 / 8 / 0.06 (0.00) / - | 75 / 5 / 0.21 (0.00) / 75 | 1 / 7 | 1.23 / 1.33, 0.568 (capped 1) |
| 2 cm/m +0.1 deg/m, walls | 9 | 33 / 8 / 0.15 (0.00) / 82 | 100 / 5 / 0.27 (0.17) / 71 | 2 / 5 | 0.35 / 0.36, 0.035 |
| 0 cm/m, walls, noisy depth | 15 | 100 / 8 / 0.29 (0.24) / 73 | 67 / 10 / 0.26 (0.19) / 91 | 10 / 3 | 0.32 / 0.00, 0.316 |
| 4 cm/m, walls, noisy depth | 15 | 0 / 15 / 0.10 (0.00) / - | 73 / 12 / 0.25 (0.17) / 102 | 0 / 14 | 0.71 / 0.69, 0.351 |

### Base vs drift vs safe, on the seeds the safe arm ran (10 per cell, 6 at 1/6/8 cm/m)

| cell | n | base: done % / contact runs / mean min clear (worst) / time s | drift (Jolie): done % / contact runs / mean min clear (worst) / time s | safe: done % / contact runs / mean min clear (worst) / time s | drift (Jolie) vs base: worse / better | safe vs base: worse / better | correction / drift at end, residual (drift (Jolie)) |
|---|---|---|---|---|---|---|---|
| 0 cm/m, nominal | 3 | 100 / 0 / 0.38 (0.38) / 67 | 100 / 0 / 0.38 (0.38) / 64 | 100 / 0 / 0.38 (0.38) / 64 | 0 / 0 | 0 / 0 | 0.01 / 0.00, 0.014 |
| 1 cm/m, nominal | 6 | 100 / 3 / 0.29 (0.23) / 67 | 100 / 0 / 0.37 (0.37) / 64 | 100 / 0 / 0.37 (0.37) / 65 | 0 / 3 | 0 / 3 | 0.16 / 0.16, 0.016 |
| 2 cm/m, nominal | 10 | 40 / 9 / 0.16 (0.00) / 68 | 100 / 0 / 0.37 (0.37) / 63 | 100 / 0 / 0.37 (0.37) / 63 | 0 / 9 | 0 / 9 | 0.32 / 0.32, 0.021 |
| 4 cm/m, nominal | 10 | 0 / 10 / 0.06 (0.00) / - | 100 / 0 / 0.37 (0.33) / 64 | 100 / 1 / 0.36 (0.26) / 65 | 0 / 10 | 0 / 10 | 0.64 / 0.65, 0.020 |
| 6 cm/m, nominal | 6 | 0 / 6 / 0.16 (0.05) / - | 100 / 2 / 0.34 (0.26) / 65 | 100 / 1 / 0.34 (0.26) / 63 | 0 / 6 | 0 / 6 | 0.97 / 0.97, 0.018 |
| 8 cm/m, nominal | 6 | 0 / 6 / 0.03 (0.00) / - | 83 / 2 / 0.28 (0.00) / 63 | 100 / 2 / 0.33 (0.26) / 64 | 0 / 6 | 0 / 6 | 1.31 / 1.30, 0.310 (capped 2) |
| 2 cm/m +jumps 1.0/10m, nominal | 10 | 60 / 9 / 0.17 (0.00) / 68 | 100 / 0 / 0.36 (0.33) / 65 | 100 / 0 / 0.36 (0.29) / 64 | 0 / 9 | 0 / 9 | 0.34 / 0.34, 0.041 |
| 2 cm/m +0.1 deg/m, nominal | 8 | 38 / 6 / 0.15 (0.00) / 69 | 100 / 0 / 0.36 (0.34) / 66 | 100 / 0 / 0.36 (0.34) / 66 | 0 / 6 | 0 / 6 | 0.33 / 0.34, 0.030 |
| 0 cm/m, nominal, mount +5 cm | 7 | 100 / 0 / 0.38 (0.38) / 64 | 100 / 0 / 0.34 (0.34) / 76 | 100 / 0 / 0.34 (0.34) / 72 | 0 / 0 | 0 / 0 | 0.04 / 0.00, 0.035 |
| 0 cm/m, nominal, pose lag 50 ms | 8 | 100 / 0 / 0.38 (0.38) / 71 | 100 / 0 / 0.38 (0.38) / 68 | 100 / 0 / 0.38 (0.38) / 67 | 0 / 0 | 0 / 0 | 0.00 / 0.00, 0.003 |
| 4 cm/m, nominal, mount +5 cm | 8 | 0 / 8 / 0.10 (0.00) / - | 100 / 0 / 0.34 (0.33) / 72 | 100 / 0 / 0.34 (0.33) / 74 | 0 / 8 | 0 / 8 | 0.64 / 0.64, 0.038 |
| 0 cm/m, nominal, mount yaw +2 deg | 7 | 100 / 0 / 0.37 (0.37) / 63 | 100 / 0 / 0.34 (0.34) / 61 | 100 / 0 / 0.34 (0.34) / 62 | 0 / 0 | 0 / 0 | 0.03 / 0.00, 0.033 |
| 4 cm/m, nominal, mount yaw +2 deg | 8 | 0 / 8 / 0.07 (0.00) / - | 100 / 0 / 0.33 (0.33) / 62 | 100 / 0 / 0.34 (0.33) / 62 | 0 / 8 | 0 / 8 | 0.65 / 0.65, 0.026 |
| 0 cm/m, nominal, noisy depth | 10 | 100 / 0 / 0.37 (0.37) / 80 | 100 / 0 / 0.31 (0.29) / 91 | 100 / 0 / 0.34 (0.33) / 77 | 0 / 0 | 0 / 0 | 0.33 / 0.00, 0.330 |
| 0 cm/m, nominal, bad depth | 10 | 90 / 0 / 0.36 (0.36) / 114 | 0 / 9 / 0.05 (0.00) / - | 80 / 2 / 0.31 (0.26) / 118 | 10 / 0 | 3 / 1 | 1.14 / 0.00, 1.139 (capped 4) |
| 4 cm/m, nominal, noisy depth | 10 | 0 / 10 / 0.13 (0.00) / - | 70 / 0 / 0.32 (0.29) / 102 | 100 / 1 / 0.33 (0.26) / 72 | 0 / 10 | 0 / 10 | 0.60 / 0.70, 0.213 |
| 0 cm/m, clutter | 10 | 90 / 1 / 0.36 (0.27) / 71 | 100 / 1 / 0.36 (0.27) / 66 | 100 / 1 / 0.36 (0.27) / 66 | 0 / 2 | 0 / 2 | 0.01 / 0.00, 0.012 |
| 0 cm/m, walls | 10 | 100 / 4 / 0.30 (0.27) / 67 | 100 / 5 / 0.28 (0.20) / 70 | 100 / 5 / 0.28 (0.18) / 67 | 4 / 2 | 4 / 2 | 0.01 / 0.00, 0.012 |
| 0 cm/m, walls_big | 10 | 80 / 8 / 0.21 (0.12) / 66 | 100 / 6 / 0.27 (0.15) / 70 | 100 / 6 / 0.26 (0.15) / 68 | 2 / 7 | 2 / 7 | 0.03 / 0.00, 0.026 |
| 1 cm/m, clutter | 6 | 100 / 3 / 0.28 (0.23) / 70 | 100 / 0 / 0.37 (0.36) / 67 | 100 / 0 / 0.36 (0.31) / 66 | 0 / 3 | 0 / 3 | 0.16 / 0.17, 0.015 |
| 1 cm/m, walls | 6 | 100 / 4 / 0.27 (0.23) / 69 | 100 / 3 / 0.28 (0.20) / 69 | 100 / 3 / 0.28 (0.20) / 68 | 2 / 2 | 2 / 2 | 0.16 / 0.16, 0.016 |
| 2 cm/m, clutter | 10 | 40 / 9 / 0.18 (0.00) / 70 | 100 / 1 / 0.35 (0.23) / 66 | 100 / 1 / 0.34 (0.15) / 66 | 0 / 9 | 0 / 8 | 0.34 / 0.34, 0.018 |
| 2 cm/m, walls | 10 | 40 / 9 / 0.18 (0.00) / 78 | 100 / 6 / 0.28 (0.23) / 69 | 100 / 6 / 0.28 (0.23) / 67 | 3 / 7 | 3 / 7 | 0.33 / 0.33, 0.022 |
| 2 cm/m, walls_big | 10 | 40 / 10 / 0.09 (0.00) / 79 | 100 / 5 / 0.27 (0.15) / 69 | 100 / 6 / 0.27 (0.13) / 67 | 1 / 9 | 0 / 10 | 0.32 / 0.33, 0.029 |
| 4 cm/m, clutter | 10 | 0 / 10 / 0.06 (0.00) / - | 100 / 2 / 0.34 (0.23) / 66 | 100 / 2 / 0.34 (0.24) / 67 | 0 / 10 | 0 / 10 | 0.68 / 0.67, 0.027 |
| 4 cm/m, walls | 10 | 0 / 10 / 0.08 (0.00) / - | 100 / 6 / 0.27 (0.20) / 68 | 100 / 5 / 0.28 (0.20) / 68 | 0 / 9 | 0 / 9 | 0.66 / 0.65, 0.032 |
| 6 cm/m, clutter | 6 | 0 / 6 / 0.16 (0.03) / - | 100 / 3 / 0.31 (0.23) / 68 | 100 / 1 / 0.33 (0.26) / 66 | 0 / 6 | 0 / 6 | 0.99 / 1.00, 0.016 |
| 6 cm/m, walls | 6 | 0 / 6 / 0.14 (0.00) / - | 100 / 3 / 0.29 (0.23) / 69 | 100 / 3 / 0.29 (0.20) / 68 | 0 / 6 | 0 / 6 | 0.99 / 0.98, 0.035 |
| 8 cm/m, clutter | 6 | 0 / 6 / 0.03 (0.00) / - | 83 / 2 / 0.28 (0.00) / 66 | 100 / 2 / 0.32 (0.26) / 66 | 0 / 6 | 0 / 6 | 1.34 / 1.33, 0.287 (capped 2) |
| 8 cm/m, walls | 6 | 0 / 6 / 0.02 (0.00) / - | 100 / 3 / 0.28 (0.23) / 75 | 83 / 3 / 0.29 (0.23) / 73 | 0 / 6 | 0 / 6 | 1.32 / 1.31, 0.025 |
| 2 cm/m +0.1 deg/m, walls | 7 | 29 / 6 / 0.13 (0.00) / 90 | 100 / 3 / 0.28 (0.20) / 71 | 100 / 3 / 0.29 (0.23) / 71 | 1 / 5 | 1 / 5 | 0.34 / 0.35, 0.032 |
| 0 cm/m, walls, noisy depth | 10 | 100 / 6 / 0.29 (0.24) / 73 | 60 / 6 / 0.25 (0.19) / 100 | 100 / 6 / 0.27 (0.23) / 77 | 7 / 2 | 5 / 3 | 0.32 / 0.00, 0.317 |
| 4 cm/m, walls, noisy depth | 10 | 0 / 10 / 0.09 (0.00) / - | 60 / 7 / 0.26 (0.17) / 100 | 90 / 6 / 0.25 (0.00) / 83 | 0 / 9 | 1 / 8 | 0.71 / 0.68, 0.391 |

### False corrections (zero drift)

| depth | arm | runs | correction left at the end, mean | largest correction during the run | runs that hit the 1.5 m cap |
|---|---|---|---|---|---|
| clean, all variants | drift | 195 | 0.016 m | 0.29 m (walls ±30 cm) | 0 |
| clean, published course | drift | 13 | 0.006 m | 0.024 m | 0 |
| clean, all variants | safe | 69 | 0.020 m | 0.29 m | 0 |
| noisy | drift | 31 | **0.317 m** | 0.55 m | 0 |
| noisy | safe | 27 | 0.095 m | 0.18 m | 0 |
| bad | drift | 10 | **1.139 m** | 1.50 m | **4** |
| bad | safe | 11 | 0.121 m | 0.30 m | 0 |

Clean depth on the published course gives Jolie's result: about 1-2 cm, no false correction. Of
that, ~1.3 cm per frame is a ray-step and raster bias that a real camera would also see from the
map's 5 cm cells. Moved walls pull the frame by up to the wall shift. Noisy stereo is where the
unguarded version breaks.

### Choosing the safeguard (seeds 0-5, `done % / mean contact steps`)

| cell (seeds 0-5) | base | drift | S1 | S2 | S3 | S4 | S6 | S7 |
|---|---|---|---|---|---|---|---|---|
| 4 cm/m, nominal (6) | 0% / 754 | 100% / 0 | 100% / 0 | 67% / 16 | 67% / 12 | 100% / 0 | 100% / 0 | 100% / 7 |
| 2 cm/m +jumps 1.0/10m, nominal (6) | 50% / 312 | 100% / 0 | 100% / 15 | 83% / 32 | 100% / 10 | 100% / 1 | 100% / 1 | 100% / 0 |
| 0 cm/m, nominal, noisy depth (6) | 100% / 0 | 100% / 0 | 100% / 0 | 100% / 0 | 100% / 0 | 67% / 0 | 100% / 1 | 100% / 0 |
| 0 cm/m, nominal, bad depth (6) | 83% / 0 | 0% / 2822 | 100% / 0 | 100% / 0 | 100% / 0 | 0% / 1518 | 0% / 2197 | 100% / 0 |
| 4 cm/m, nominal, noisy depth (6) | 0% / 504 | 67% / 0 | 83% / 7 | 67% / 193 | 50% / 122 | 83% / 7 | 67% / 7 | 100% / 8 |
| 0 cm/m, walls (10) | 100% / 20 | 100% / 54 | 100% / 29 | 100% / 34 | 100% / 35 | 100% / 58 | 100% / 58 | 100% / 56 |
| 0 cm/m, walls_big (6) | 83% / 426 | 100% / 51 | 100% / 149 | 100% / 130 | 100% / 126 | 83% / 390 | 100% / 95 | 100% / 56 |
| 4 cm/m, walls (6) | 0% / 682 | 100% / 58 | 50% / 91 | 50% / 90 | 67% / 58 | 100% / 40 | 100% / 58 | 100% / 64 |
| 8 cm/m, walls (6) | 0% / 2992 | 100% / 60 | 17% / 1066 | 0% / 772 | 0% / 475 | 83% / 391 | 100% / 88 | 83% / 44 |
| 0 cm/m, walls, noisy depth (6) | 100% / 30 | 50% / 122 | 100% / 56 | 100% / 61 | 100% / 42 | 83% / 54 | 67% / 226 | 100% / 56 |

* S1: Tukey-robust fit (c 8 cm) + evidence 30.
* S2: S1 + gain 0.3.
* S3: S1 + median of 3.
* S4: median of 3 consecutive shifts only.
* S6: the coordinator's suggestion, a post-fit inlier gate (60 % of hits within 5 cm) + median of 3.
* **S7: consensus gate.** Jolie's fit runs unchanged. A Tukey-robust refit (c 8 cm), started from
  her answer, must move < 3 cm, or the frame is refused.

Robust-only fits (S1-S3) fix noisy depth. But they lose high-drift tracking (8 cm/m + walls: 0-17 %
done), because the Tukey window shrinks the capture range. S4 and S6 keep the tracking but do not
fix bad depth (0 %). Only S7 does both. A "two distinct wall faces must agree" gate was also
tried: it halved the accepted frames and still let moved-wall shifts through, so it was dropped.

## Decision

**Adopt with safeguards: `upload_to_flyx_drift_safe*.py` (S7). Do not adopt Jolie's version as it
is.**

* **Where it helps (both versions):** any systematic drift. At ≥ 2 cm/m the base fails 40-100 %
  of flights (4 cm/m: 0 % done), and the drift and safe arms complete 100 %. At 2 cm/m on the
  published course: base 53 %, drift 100 %, n = 30. It also holds under heading drift, VIO jumps,
  camera mount offset / yaw error, 50 ms pose lag, clutter and moved walls. The correction tracks
  the true drift to 2-4 cm residual, up to 6 cm/m. At 8 cm/m the 1.5 m cap is reached in some runs.
* **Zero drift + moved walls is worse on contacts, the same on completion.** This confirms the
  residual agent's finding on contacts and clearance. Walls ±15 cm, n = 30, 100 % vs 100 % done:
  * the drift arm had more contact steps on 18 seeds, fewer on 4;
  * mean contact steps 85 vs 42, mean minimum clearance 0.260 vs 0.283 m;
  * cause: the frame takes up to 15 cm of the moved wall's offset.

  We did not reproduce a completion loss from moved walls alone. With walls ±30 cm, the drift arm
  finishes more often (100 % vs 87 %). The completion losses come when moved walls are combined
  with noisy depth: drift 60 % vs base 100 %, and the safe arm restores 100 %. No safeguard tested
  removes the moved-wall contact cost. To a fit against the map, a coherently moved wall looks the
  same as drift.
* **Jolie's version fails with noisy stereo.** Bad depth, zero drift: 0 % done vs 90 %, false
  correction 1.1 m, 4 runs capped. Noisy depth + walls: 60 % vs 100 %. The safe version: 80 % vs 90 %
  (bad), 100 % vs 100 % (walls + noisy).
* **"Never worse" does not hold for either version.** Safe is never worse on completion in any
  cell, except bad depth at zero drift (80 % vs 90 %, 10 seeds) and 8 cm/m + walls (83 % vs Jolie
  100 %; base 0 %). It costs contact margin with moved walls at zero drift.

**For the FlyX lab:** the proven base already completed there, so FlyX drift is small. Keep
`upload_to_flyx.py` as the competition file. Fly `upload_to_flyx_drift_safe.py` once (Reset
world, team approval) as the A/B. Switch if its course time, clearance and pad error are no worse.
**For the drone** (cuVSLAM stereo-only drift, real stereo noise, a hall that may differ from the
map), the safe Sky Beta file is the better bet.

## Files

| file | what |
|---|---|
| `jolie_course/upload_to_flyx_drift.py` | Jolie's f7b97d3 native-PX4 FlyX file, verbatim (equal to a rebuild of her sources; only the commit stamp e1ac88d -> f7b97d3 corrected) |
| `jolie_course/upload_to_flyx_drift_skybeta.py` | the same on the Sky Beta contract (`build_skybeta_copy.py`'s swap + `skybeta_main.py.in`). Its main has the same `apply_alignment`, copied verbatim, plus the `on_plan` call and node state |
| `jolie_course/upload_to_flyx_drift_safe.py` | native FlyX, consensus gate (S7). Jolie's `align_to_map` kept verbatim as `_align_plain` |
| `jolie_course/upload_to_flyx_drift_safe_skybeta.py` | the safe file on the Sky Beta contract |
| `drift/build_drift_files.py` | builds the last three from `upload_to_flyx_drift.py` |
| `drift/check_files.py` | the embedded `align_to_map` == the harness's (0 mismatches on 400 clouds), Sky Beta mains have no `/fmu` and apply the alignment |
| `drift/ab_sim.py`, `run_grid.py`, `make_plan.py`, `analyze.py`, `mdtable.py`, `probe_align.py`, `align_safe.py` | the harness |

## What only a FlyX run (or the drone) can confirm

* How much FlyX odometry actually drifts. The base completed there, so probably < 1-2 cm/m.
* What the FlyX OAK-D's depth noise really looks like. Our noisy / bad models are guesses, and
  they decide the outcome.
* The real planning-thread latency and frame timing on a loaded lab machine. The sim assumes the
  shift lands 0.1 s after planning starts. More latency double-counts frames.
* Whether the FlyX world matches the published map to within a few cm.
* PX4's response to ≤ 4 cm / 0.2 s frame shifts.
* Course time. The sim's fake PX4 is ~2x faster than FlyX.
* For the drone: the real cuVSLAM drift, camera extrinsics, and how much the hall's walls differ
  from the published map.

