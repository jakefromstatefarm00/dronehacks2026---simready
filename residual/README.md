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

RESULTS_PLACEHOLDER
