# Learned residuals on Jolie's planner: work-in-progress snapshot (2026-11-07 ~14:50)

**Experimental: training is still running, and neither residual has passed its acceptance gate
yet. Simulation only; nothing here has flown in Gazebo or FlyX.** Your proven
`jolie_course/upload_to_flyx.py` and `upload_to_flyx_skybeta.py` are untouched.

## What a residual is here

`residual/README.md` has the full design.

The residual is a 20 → 16 → 3 tanh MLP with 387 numbers, run in numpy only (no torch at runtime).
It runs at each 5 Hz planning step inside your `Mission.plan()` / `_limit_yaw` and nudges three
things:

| Output | Range | Neutral |
|---|---|---|
| carrot length × s | 0.6–1.3 | 1.0 |
| lateral carrot offset | ±0.08 m, only on a side the planner process checked | 0 |
| yaw-rate cap × y | 0.6–1.0 (it can only turn *slower* than 0.25 rad/s) | 1.0 |

Your code stays in charge of everything else: route, turn-in-place, checkpoints, pad centring,
landing and every guard.

The residual outputs **exactly zero** (your behaviour) in any of these cases:
- odometry is older than 0.3 s;
- there was a control stall in the last 0.5 s;
- planner info is missing;
- any number is non-finite;
- outside NAVIGATE.

`--no-residual` reproduces your file exactly; there is a test for that.

## Training setup

- **Simulator:** closed loop around the embedded code that flew, with the dynamics calibrated from
  15 real PX4 ulogs in Gazebo (held-out replay error 1.8 cm RMS over 2 s).
- **Speed:** tuned toward your FlyX NAVIGATE time.
- **Disturbances:** program stalls and odometry pauses like the lab, shifted walls and clutter, and
  graded estimator noise and drift.
- **Optimiser:** evolution strategies, with a curriculum from L0 (nominal) upward.

## Where it is (held-out seeds, latest gate check)

| Base | Level | Base: complete / time / contacts | Base + residual: complete / time / contacts | Clearance p5 | Gate |
|---|---|---|---|---|---|
| proven (`upload_to_flyx.py`) | L2 | 68.8 % / 97.6 s / 5 | 75.0 % / **78.5 s** / 4 | 0.278 → 0.279 m | not passed |
| map alignment (`jolie-drift-correction` f7b97d3) | L1 | 91.7 % / 91.8 s / 1 | 91.7 % / **77.7 s** / 1 | 0.288 → 0.299 m | not passed |

The residual flies the same courses about 15–20 % faster, with no more contacts. It has **not**
met the rule: never worse than the base at any level, and at least 95 % completion. L2 also adds
disturbances under which the base itself completes only 69 %, which says as much about the
simulated conditions as about the residual.

## Files

- `residual/policy.json`: the current best residual for the proven base (`runs/proven/policy_best.json`).
- `residual/policy_drift.json`: the current best residual for the map-alignment base.
- `jolie_course/upload_to_flyx_residual.py`: your FlyX file + residual. Native PX4, FlyX only.
- `jolie_course/upload_to_flyx_residual_skybeta.py`: the same on the Sky Beta contract (drone / `run.sh`).
- `jolie_course/upload_to_flyx_drift_residual.py`: the map-alignment build + its residual.
- `residual/runs/*/train_log.jsonl`: the learning curves, including every gate check.
- `jolie_course/build_residual_copy.py`: rebuilds the three files from your files + the policies.

## Try it offline

```
python -m pytest -q residual/tests
python -m residual.evaluate --levels L0 L1 --seeds 1000-1015          # base vs base + residual
```

## Known gaps before any lab use

- **Vision loss:** in 15 Gazebo runs, every visual-odometry divergence started at the first
  corridor-to-maze corner, after yawing 0.46–0.8 rad/s *while moving* at 0.4–0.5 m/s. A separate
  risk event for this is being added to the simulator.
- **Lab pace:** the lab flew your planner about 2× slower than Gazebo in sim time (NAVIGATE ≈ 97 s
  vs 50 s). The cause is on the navigation side (stalls, replanning or the yaw cap). The lab run's
  `journal.jsonl` would show which, so please upload
  `/home/flyx/flyx-runs/flyx-20261107-205352/`.
