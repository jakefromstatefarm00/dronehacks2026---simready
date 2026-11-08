# residual/: a small trained correction on top of Jolie's FlyX-proven planner

The base is `jolie_course/upload_to_flyx.py`, the file that completed the whole course in FlyX.
It is **never modified**. The residual is a 20 → 16 → 3 tanh MLP (387 numbers, numpy-only at
runtime). At each 5 Hz planning step it nudges three things in her `Mission.plan()` /
`_limit_yaw`:

| output | range | 0 = | why this and not more |
|---|---|---|---|
| carrot length × s | 0.6 … 1.3 | 1.0 | PX4 flies straight at a pose setpoint with speed ≈ P × distance. Carrot length is speed, and it is the one knob her loop already uses (0.5 m, shorter while turning). Lengthening only uses a point the planner process checked against the keep-out (`ext`). |
| lateral carrot offset | ±0.08 m | 0 | wall clearance. Applied only to a side whose straight line the planner process checked. 8 cm equals her tracking margin, so it can never undo the keep-out. |
| yaw-rate cap × y | 0.6 … 1.0 | 1.0 | VIO loss rises with yaw rate. The residual may only turn *slower* than her 0.25 rad/s. |

The base stays in charge. The route, turn-in-place, checkpoints, pad centring, landing and every
guard are still her code. The residual gives **zero correction** (her exact target object and
cap) when odometry is older than 0.3 s, when a control stall happened in the last 0.5 s, when
planner info is missing, when any number is non-finite, or in any state other than NAVIGATE.

## Files

| file | what |
|---|---|
| `runtime.py` | the residual itself (numpy only): observation, MLP, bounds, safe application. Pasted verbatim into the generated flight files. |
| `sim.py` | closed-loop simulator around the **embedded** code that flew (see below) |
| `train.py` | curriculum + antithetic evolution strategies |
| `evaluate.py` | base vs base+residual on held-out seeds, acceptance rule, `results/results.md` |
| `policy.json` | exported residual for the proven base |
| `policy_drift.json` | exported residual for the map-alignment base |
| `bases/upload_to_flyx_drift.py` | Jolie's drift-correction single file (origin/jolie-drift-correction f7b97d3, rebuilt with its `tools/build_single_file.py`, identical to the one committed there) |
| `tests/test_residual.py` | bounds, zero-on-stale, zero-policy == base in closed loop, generated files == originals with `--no-residual` |
| `../jolie_course/build_residual_copy.py` | writes `upload_to_flyx_residual.py`, `upload_to_flyx_residual_skybeta.py`, `upload_to_flyx_drift_residual.py` |

## Run

```
python -m pytest -q residual/tests                              # ~1 min
python -m residual.evaluate --base-only --levels L0 --seeds 1000-1015   # base sanity
python -m residual.train --minutes 55 --workers 15 --out residual/runs/proven
python -m residual.train --variant drift --levels L0 L1 L2 D1 L3 L4 D2 L5 --out residual/runs/drift
cp residual/runs/proven/policy_best.json residual/policy.json   # (and drift -> policy_drift.json)
python -m residual.evaluate --variants proven drift --levels L0 L1 L2 L3 L4 L5 D1 D2 --seeds 1000-1023
python jolie_course/build_residual_copy.py
```

In FlyX, upload or run the generated file exactly as you would the original: Reset world, start
on the mark. `--no-residual` flies the original program unchanged.

```
python3 jolie_course/upload_to_flyx_residual.py                 # native PX4, FlyX lab
python3 jolie_course/upload_to_flyx_residual.py --no-residual   # = upload_to_flyx.py
bash flyx_skybeta/run.sh ...  /  python3 jolie_course/upload_to_flyx_residual_skybeta.py   # Sky Beta contract
python3 jolie_course/upload_to_flyx_drift_residual.py           # map-alignment base + its residual
```

At the end the terminal prints `residual: acted N planning steps, zero M (reasons)`, and the
journal gets a `{"kind": "residual", ...}` record.

## The simulator (what it models)

What runs is the code that flew, exec'd from the `_SOURCES` embedded in the single file:
- her `Mission` state machine, every step from WAIT_HEALTH to DONE;
- her planner child loop `planner_process` (run unmodified on a thread with an in-memory pipe);
- `project_depth`, `drop_misregistered`, hit votes, `OnlinePlanner` with TURN_IN_PLACE_DEG 60, and `safe_carrot_path`;
- the embedded map.

For the drift base, her node's `apply_alignment` (0.6 × shift, ≤ 4 cm/step, ≤ 1.5 m) is reproduced
in the loop, because it lives in the ROS node, not in an embedded module.

Around it:
- **PX4 + airframe + cuVSLAM**: `team_mission/rl/dynamics.py` `DroneSim` (P position loop →
  lagged, saturated velocity loop, yaw loop, link delay, estimator noise/drift, disturbance).
  Added on top is a PX4 mode model: Offboard needs a stream, arm needs Offboard; no setpoint for
  COM_OF_LOSS_T = 1 s → failsafe hold, which returns 0.3 s after setpoints resume; Land descends.
- **Lab clock**: the program's monotonic clock and timers run on lab wall time, and physics on
  sim time = 0.8 × wall (FlyX runs at ~0.8× real time).
- **Calibration to Jolie's completed FlyX run** (coordinator's timings): NAVIGATE 120.85 s wall
  ≈ 97 s sim, TAKEOFF 7.0 s wall ≈ 5.6 s sim. PX4's nominal MPC_XY_P 0.95 with a 0.35 s lag gave
  NAVIGATE 71 s sim, which is too fast. An *effective* P of 0.7 with a 0.5 s velocity lag (kp_z 0.35)
  gives 96–101 s sim at L0 (≈ 120–126 s wall). The FlyX run is matched by an effective P gain,
  not by a measured cause; see risks.
- **Depth**: a synthetic 32FC1 image (320 × 40, which `project_depth` samples at 40 × 5) is
  ray-cast from the TRUE pose on the TRUE course: walls are vertical extrusions, below-horizon rows
  return the floor. It is then projected with the ESTIMATED pose, so estimator error misplaces depth
  as it does on the drone.
- **Lab starvation**: program stalls (nothing ticks, PX4 flies the last setpoint, then fails safe),
  odometry pauses 0.3–1.4 s (her hold), slow topics silent 2–6 s.
- **VIO loss**: hazard per second `0.0015·exp((|ω|−0.25)/0.05)`, ×3 when turning in place
  (speed < 8 cm/s), from the true yaw rate. 75 % of events are a drift jump of 5–25 cm with a
  0.3–0.8 s pause; 25 % are a loss of 0.6–2.5 s (dead reckoning; after 1 s PX4 reports xy invalid
  → her guard lands). This is calibrated only qualitatively to "30/40 losses above 0.35 rad/s".
- **Systematic drift** (D levels): 0–8 cm per metre flown in a random direction, plus 0–0.5°/m of
  heading drift.
- **Truth**: contact = exact distance from the drone's slab to the perturbed course < 0.28 m (Sky
  Beta prop-tip radius; rounded box corners, posts, tube shell, net). Checkpoints are scored from
  the true trajectory, and the pad must hold the whole footprint.

Not modelled:
- attitude, rotor or battery physics;
- 3-D obstacles above or below the slab (boulders sit below it);
- textureless-depth dropout;
- real stereo artefacts;
- the planner's own compute latency (each plan is applied the same tick);
- the GIL coupling between planner and control;
- RC or a pilot;
- Sky Beta's z ≥ 0.3 m clamp (irrelevant at 0.9 m).

The residual never sees truth. Its observation is:
- estimated velocity (along / across the chord);
- heading error to the route;
- base carrot fraction and safe-chord length, and whether a safe extension exists;
- distance to the next checkpoint and the checkpoint index;
- turn mode;
- 5 wall probes on the drone's belief map (0°, ±45°, ±90°, to 1.2 m);
- odometry age and time since the last stall;
- its previous action.

## Curriculum and training

| level | adds |
|---|---|
| L0 | published course, calibrated nominal dynamics, 3 mm pose jitter |
| L1 | maze / chicane walls shifted ≤ 8 cm |
| L2 | walls ≤ 15 cm + 1–3 unmapped posts (`variants.perturb("mixed")`) |
| L3 | + half of dynamics.py's estimator noise / drift / link delay / disturbance, VIO-loss events, launch error 1.5 cm, depth noise |
| L4 | + stalls 0.2/s ≤ 1.2 s, odometry pauses, slow-topic silences |
| L5 | + full dynamics.py ranges, randomised PX4 gains / lag, stalls 0.4/s ≤ 2 s (FlyX run 7), VIO × 1.5, launch error 3 cm |
| D1 | L2 + systematic VIO drift 0–8 cm/m and heading drift 0–0.5°/m |
| D2 | L4 + the D1 drift |

The trainer is antithetic ES on common random numbers. θ + ε and θ − ε fly the same world,
dynamics draw and event schedule, and the output layer starts at zero, so iteration 0 *is* the
base. Every 4 iterations the mean policy flies held-out gate seeds (2000+). The level advances
only if the residual's completion is ≥ 95 % **and** ≥ the base's, with no more contacts. 25 %
of each batch replays lower levels.

The fitness uses truth for scoring only:
- +40 for completion and +8 per checkpoint;
- +0.5 per second of NAVIGATE saved below 180 s;
- −80 for a contact;
- −100 per m·s of clearance deficit below 0.30 m;
- −30 per expected VIO event (integrated hazard) and −5 per realised event;
- a small smoothness term.

Seeds < 1000 train, 2000+ gate, 1000–1999 are the final acceptance set.

## Results

Final acceptance run: held-out seeds 1000-1015, 16 runs per cell, 640 episodes
(`python -m residual.evaluate --variants proven drift --levels L0 L1 L2 L3 L4 L5 D1 D2 C0 C1 --seeds 1000-1015`).
Policies: `policy.json` = proven run iteration 22 (the curriculum reached L2), `policy_drift.json` = drift run
iteration 28 (reached L1). Time is NAVIGATE in sim seconds (lab wall = sim / 0.8; FlyX was 120.85 s wall = 96.7 s sim).
Raw rows: `results/episodes.jsonl`; training logs: `results/train_log_*.jsonl`.

**Summary**
- **Proven base + residual, L0-L5 (the curriculum it trained on):** never worse on completion or contacts at any level.
  - NAVIGATE drops 15-19 s at L0-L4. L0 goes from 96.3 to 80.1 s sim, which is about 120 to 100 s lab wall, under the 120 s target.
  - L0 p5 clearance improves from 0.340 to 0.392 m.
  - Completion is higher under estimator noise and lab stalls: L3 62 -> 75 %, L4 44 -> 62 %, L5 12 -> 25 %.
- **Formal acceptance (every level): not met for either residual.**
  - Proven residual: D1 (drift) completion 19 -> 12 % (3 -> 2 runs, contacts equal), C0 38 -> 25 % (6 -> 4 runs), C1 19 -> 12 %.
  - Drift residual: worse at L1/L2 (one and three more contacts) and at D1.
  - With 16 seeds, a difference of 1-2 runs is within noise, but the rule is the rule.
  - Neither residual was trained on drift (D) or calibrated (C) conditions.
- **The drift (map-alignment) base itself** is worse than the proven base on shifted walls: L1 94 % vs 100 %, L2 62 % vs 100 %. It is better under drift: D1 31 % vs 19 %. Alignment to published walls is pulled by walls that moved.
- **Calibrated levels C0/C1** (dynamics-calib YAML: P 0.95, lag up to 1.05 s, delay up to 6 ticks, drift up to 5.4 cm/sqrt(m)): the base completes only 38 % / 19 %. Contacts come from drift and lag; the residual does not change them.

| level | arm | runs | completion | time mean / p90 (s) | min clearance min / p5 (m) | time < 0.30 m (s) | contacts | VIO risk (exp. events) | VIO events |
|---|---|---|---|---|---|---|---|---|---|
| L0 | proven base | 16 | 100 % | 96.3 / 100.4 | 0.337 / 0.340 | 0.0 | 0 | 0.136 | 0 |
| L0 | proven base + residual | 16 | 100 % | 80.1 / 82.7 | 0.388 / 0.392 | 0.0 | 0 | 0.244 | 0 |
| L0 | drift base | 16 | 100 % | 92.8 / 96.7 | 0.365 / 0.367 | 0.0 | 0 | 0.139 | 0 |
| L0 | drift base + residual | 16 | 100 % | 80.9 / 84.7 | 0.378 / 0.382 | 0.0 | 0 | 0.232 | 0 |
| L1 | proven base | 16 | 100 % | 96.2 / 100.0 | 0.328 / 0.332 | 0.0 | 0 | 0.138 | 0 |
| L1 | proven base + residual | 16 | 100 % | 78.8 / 82.4 | 0.310 / 0.323 | 0.0 | 0 | 0.231 | 0 |
| L1 | drift base | 16 | 94 % | 92.5 / 95.1 | 0.278 / 0.295 | 0.0 | 1 | 0.144 | 0 |
| L1 | drift base + residual | 16 | 88 % | 75.4 / 77.3 | 0.279 / 0.280 | 0.0 | 2 | 0.199 | 0 |
| L2 | proven base | 16 | 100 % | 100.6 / 104.6 | 0.292 / 0.299 | 0.1 | 0 | 0.139 | 0 |
| L2 | proven base + residual | 16 | 100 % | 81.1 / 85.7 | 0.292 / 0.293 | 0.1 | 0 | 0.242 | 0 |
| L2 | drift base | 16 | 62 % | 102.0 / 106.4 | 0.275 / 0.276 | 0.3 | 5 | 0.135 | 0 |
| L2 | drift base + residual | 16 | 44 % | 77.9 / 80.8 | 0.276 / 0.277 | 0.3 | 8 | 0.168 | 0 |
| L3 | proven base | 16 | 62 % | 95.7 / 101.5 | 0.275 / 0.278 | 0.6 | 6 | 0.140 | 1 |
| L3 | proven base + residual | 16 | 75 % | 81.7 / 91.0 | 0.277 / 0.278 | 0.3 | 4 | 0.215 | 4 |
| L3 | drift base | 16 | 44 % | 93.9 / 101.3 | 0.276 / 0.277 | 0.3 | 9 | 0.119 | 1 |
| L3 | drift base + residual | 16 | 50 % | 77.1 / 82.8 | 0.278 / 0.278 | 0.5 | 7 | 0.155 | 4 |
| L4 | proven base | 16 | 44 % | 96.0 / 103.8 | 0.277 / 0.278 | 0.5 | 7 | 0.111 | 1 |
| L4 | proven base + residual | 16 | 62 % | 82.7 / 91.0 | 0.276 / 0.278 | 0.3 | 4 | 0.129 | 1 |
| L4 | drift base | 16 | 6 % | 94.4 / 94.4 | 0.276 / 0.277 | 0.6 | 12 | 0.057 | 2 |
| L4 | drift base + residual | 16 | 19 % | 86.7 / 97.4 | 0.277 / 0.277 | 0.4 | 10 | 0.095 | 0 |
| L5 | proven base | 16 | 12 % | 90.5 / 98.9 | 0.208 / 0.253 | 0.4 | 12 | 0.064 | 3 |
| L5 | proven base + residual | 16 | 25 % | 94.1 / 99.3 | 0.208 / 0.252 | 0.3 | 10 | 0.062 | 1 |
| L5 | drift base | 16 | 6 % | 96.3 / 96.3 | 0.208 / 0.251 | 0.5 | 12 | 0.049 | 1 |
| L5 | drift base + residual | 16 | 6 % | 73.8 / 73.8 | 0.208 / 0.252 | 0.4 | 12 | 0.055 | 2 |
| D1 | proven base | 16 | 19 % | 100.7 / 103.7 | 0.274 / 0.275 | 0.4 | 13 | 0.085 | 0 |
| D1 | proven base + residual | 16 | 12 % | 79.6 / 80.3 | 0.277 / 0.278 | 2.9 | 13 | 0.197 | 0 |
| D1 | drift base | 16 | 31 % | 104.6 / 110.3 | 0.271 / 0.275 | 0.4 | 11 | 0.114 | 0 |
| D1 | drift base + residual | 16 | 25 % | 82.6 / 88.8 | 0.271 / 0.274 | 0.3 | 12 | 0.175 | 0 |
| D2 | proven base | 16 | 0 % | nan / nan | 0.273 / 0.276 | 0.5 | 11 | 0.059 | 2 |
| D2 | proven base + residual | 16 | 6 % | 82.6 / 82.6 | 0.276 / 0.276 | 0.5 | 11 | 0.089 | 1 |
| D2 | drift base | 16 | 6 % | 93.5 / 93.5 | 0.272 / 0.275 | 0.3 | 12 | 0.063 | 2 |
| D2 | drift base + residual | 16 | 12 % | 83.1 / 87.8 | 0.275 / 0.276 | 0.4 | 12 | 0.079 | 0 |
| C0 | proven base | 16 | 38 % | 71.6 / 80.4 | 0.188 / 0.243 | 0.6 | 8 | 0.003 | 0 |
| C0 | proven base + residual | 16 | 25 % | 71.9 / 78.5 | 0.188 / 0.243 | 0.5 | 9 | 0.014 | 0 |
| C0 | drift base | 16 | 38 % | 70.1 / 81.7 | 0.188 / 0.245 | 0.4 | 8 | 0.003 | 0 |
| C0 | drift base + residual | 16 | 44 % | 67.0 / 75.8 | 0.188 / 0.245 | 0.4 | 7 | 0.017 | 0 |
| C1 | proven base | 16 | 19 % | 68.5 / 77.5 | 0.188 / 0.246 | 0.5 | 11 | 0.003 | 0 |
| C1 | proven base + residual | 16 | 12 % | 77.1 / 82.9 | 0.188 / 0.245 | 0.8 | 12 | 0.016 | 0 |
| C1 | drift base | 16 | 19 % | 65.3 / 72.2 | 0.188 / 0.245 | 0.4 | 11 | 0.003 | 0 |
| C1 | drift base + residual | 16 | 44 % | 66.8 / 73.0 | 0.188 / 0.245 | 0.4 | 7 | 0.010 | 0 |

- **L0**: published course, nominal dynamics
- **L1**: maze/chicane walls shifted up to 8 cm
- **L2**: walls up to 15 cm + 1-3 unmapped posts
- **L3**: + half of dynamics.py's cuVSLAM noise/drift/delay/disturbance ranges, VIO-loss hazard, launch error 1.5 cm, depth noise
- **L4**: + lab starvation: stalls <= 1.2 s, odometry pauses 0.3-1.4 s, slow topics silent 2-6 s
- **L5**: + full dynamics.py estimator ranges, randomised PX4 gains/lag, FlyX-run-7 stalls (0.4/s, <= 2 s), VIO x1.5, launch error 3 cm
- **D1**: L2 + systematic VIO drift 0-8 cm/m and heading drift 0-0.5 deg/m
- **D2**: L4 + the D1 drift
- **C0**: published course, calibrated dynamics YAML (dynamics-calib 4964e62), lab starvation as L4, corner VIO divergence
- **C1**: C0 on the L2 course perturbations (walls <= 15 cm + posts)

**proven residual vs proven base: accepted = False**
- L0: not worse = True, faster = True, wider p5 clearance = True
- L1: not worse = True, faster = True, wider p5 clearance = False
- L2: not worse = True, faster = True, wider p5 clearance = False
- L3: not worse = True, faster = True, wider p5 clearance = False
- L4: not worse = True, faster = True, wider p5 clearance = False
- L5: not worse = True, faster = False, wider p5 clearance = False
- D1: not worse = False, faster = True, wider p5 clearance = False
- D2: not worse = True, faster = False, wider p5 clearance = False
- C0: not worse = False, faster = False, wider p5 clearance = False
- C1: not worse = False, faster = False, wider p5 clearance = False

**drift residual vs drift base: accepted = False**
- L0: not worse = True, faster = True, wider p5 clearance = True
- L1: not worse = False, faster = True, wider p5 clearance = False
- L2: not worse = False, faster = True, wider p5 clearance = False
- L3: not worse = True, faster = True, wider p5 clearance = False
- L4: not worse = True, faster = True, wider p5 clearance = False
- L5: not worse = True, faster = True, wider p5 clearance = False
- D1: not worse = False, faster = True, wider p5 clearance = False
- D2: not worse = True, faster = True, wider p5 clearance = False
- C0: not worse = True, faster = True, wider p5 clearance = False
- C1: not worse = True, faster = False, wider p5 clearance = False

seeds 1000-1015, 640 episodes, 1121 s wall

