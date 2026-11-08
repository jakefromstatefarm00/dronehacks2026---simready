# Serial, unloaded Gazebo A/B: Jolie's planner with vs without the residual (v600-v609)

Program: `jolie_flyx/upload_to_flyx_residual.py` (residual ON) vs the same file with `--no-residual`
(byte-for-byte base behaviour). GB10 Gazebo + real PX4 SITL + OpenVINS, `jolie_flyx/launch.sh NNN <program>`,
no DEPTH, no other args. Runner `jolie_flyx/serial_ab.sh` (copied here): exactly one flight at a time, arms
alternating (even = base, odd = residual), each flight's slot torn down before the next launch. 2026-11-08
21:15-21:52 GB10 time. Before the block the queue runner was stopped via `jolie_flyx/STOP` (queue/active
empty). Host load: ~2.0 baseline (two stale QGC processes from Nov 7 at ~100% CPU each), 3.7-6.7 (1-min) during
the flights on 20 cores; see `serial_ab_load.log`. The earlier v500-v509 residual runs were flown 3-in-parallel,
so they are confounded; v200-v214 (base, also queue-parallel) are re-scored for reference.

Scored with `python -m residual.score_gazebo` via `summarize_ab.py` (same scorer, plus per-arm aggregation).
yv = |yaw rate| x speed from Gazebo truth (rad m/s^2); corner = within 1 m of the first corridor-to-maze corner.

## Per-arm summary

| arm | runs | completed | NAVIGATE s (completed) | checkpoints | land reasons | corner yv peak | yv peak (all) | yv>0.10 s | diverged_at_s |
|---|---|---|---|---|---|---|---|---|---|
| serial base | v600..v608 (n=5) | 5/5 | 67.1 +/- 6.1 | 5 5 5 5 5 | landing pad reached (0.08 m from its centre) x2; landing pad reached (0.07 m from its centre) x3 | 0.114 +/- 0.002 | 0.124 +/- 0.003 | 4.4 +/- 0.4 | - |
| serial residual | v601..v609 (n=5) | 4/5 | 70.7 +/- 6.4 | 5 1 5 5 5 | landing pad reached (0.07 m from its centre) x2; flight time limit x1; landing pad reached (0.08 m from its centre) x2 | 0.157 +/- 0.030 | 0.163 +/- 0.026 | 8.7 +/- 3.5 | 5.3 |
| ref base (parallel) | v200..v214 (n=15) | 14/15 | 65.8 +/- 5.7 | 5 5 5 5 5 5 5 5 5 2 5 5 5 5 5 | landing pad reached (0.08 m from its centre) x8; landing pad reached (0.07 m from its centre) x6; unhealthy: estimator invalid (xy_valid=False, x1 | 0.115 +/- 0.007 | 0.137 +/- 0.054 | 3.4 +/- 1.0 | 12.8 |
| ref residual (parallel) | v500..v509 (n=10) | 0/10 | - | 0 1 0 0 1 0 0 0 0 0 | unhealthy: estimator invalid (xy_valid=False, x3; DONE - landed and disarmed x2; LAND - unhealthy: estimator invalid (xy_valid x2; NAVIGATE - near altitude after 10 s (0.25 m o x1; TAKEOFF x2 | 0.184 +/- 0.049 | 0.127 +/- 0.075 | 0.4 +/- 0.9 | 11.0 |

## Per-run rows

| run | arm | residual | complete | checkpoints | navigate_s | land_reason | diverged_at_s | corner_yv_peak | yv_peak | yv_above_0p10_s | yaw_rate_peak | speed_mean |
|---|---|---|---|---|---|---|---|---|---|---|---|---|
| v600 | serial base | off | True | 5 | 68.1 | landing pad reached (0.08 m from its centre) | None | 0.116 | 0.122 | 3.83 | 0.336 | 0.247 |
| v602 | serial base | off | True | 5 | 62.1 | landing pad reached (0.07 m from its centre) | None | 0.113 | 0.123 | 4.26 | 0.36 | 0.266 |
| v604 | serial base | off | True | 5 | 62.2 | landing pad reached (0.07 m from its centre) | None | 0.113 | 0.126 | 4.78 | 0.349 | 0.267 |
| v606 | serial base | off | True | 5 | 66.0 | landing pad reached (0.07 m from its centre) | None | 0.116 | 0.122 | 4.5 | 0.365 | 0.251 |
| v608 | serial base | off | True | 5 | 77.0 | landing pad reached (0.08 m from its centre) | None | 0.113 | 0.128 | 4.57 | 0.351 | 0.222 |
| v601 | serial residual | ON: acted 396 planning steps, zero 23 ({'not navigatin | True | 5 | 63.9 | landing pad reached (0.07 m from its centre) | None | 0.15 | 0.15 | 8.88 | 0.377 | 0.27 |
| v603 | serial residual | ON: acted 1040 planning steps, zero 18 ({'not navigati | False | 1 | 167.0 | flight time limit | 5.3 | 0.209 | 0.209 | 2.65 | 0.835 | 0.383 |
| v605 | serial residual | ON: acted 420 planning steps, zero 23 ({'not navigatin | True | 5 | 67.7 | landing pad reached (0.08 m from its centre) | None | 0.142 | 0.151 | 10.02 | 0.355 | 0.261 |
| v607 | serial residual | ON: acted 490 planning steps, zero 24 ({'not navigatin | True | 5 | 78.8 | landing pad reached (0.07 m from its centre) | None | 0.135 | 0.151 | 10.07 | 0.352 | 0.227 |
| v609 | serial residual | ON: acted 452 planning steps, zero 24 ({'not navigatin | True | 5 | 72.5 | landing pad reached (0.08 m from its centre) | None | 0.148 | 0.156 | 11.8 | 0.356 | 0.247 |
| v200 | ref base (parallel) | off | True | 5 | 65.0 | landing pad reached (0.08 m from its centre) | None | 0.11 | 0.121 | 2.58 | 0.336 | 0.257 |
| v201 | ref base (parallel) | off | True | 5 | 61.1 | landing pad reached (0.08 m from its centre) | None | 0.113 | 0.124 | 3.79 | 0.349 | 0.271 |
| v202 | ref base (parallel) | off | True | 5 | 70.2 | landing pad reached (0.08 m from its centre) | None | 0.114 | 0.117 | 3.03 | 0.331 | 0.236 |
| v203 | ref base (parallel) | off | True | 5 | 60.1 | landing pad reached (0.07 m from its centre) | None | 0.114 | 0.13 | 4.82 | 0.366 | 0.275 |
| v204 | ref base (parallel) | off | True | 5 | 63.9 | landing pad reached (0.07 m from its centre) | None | 0.117 | 0.128 | 3.36 | 0.372 | 0.264 |
| v205 | ref base (parallel) | off | True | 5 | 66.3 | landing pad reached (0.08 m from its centre) | None | 0.114 | 0.12 | 3.52 | 0.362 | 0.251 |
| v206 | ref base (parallel) | off | True | 5 | 61.1 | landing pad reached (0.07 m from its centre) | None | 0.108 | 0.125 | 4.65 | 0.365 | 0.272 |
| v207 | ref base (parallel) | off | True | 5 | 65.1 | landing pad reached (0.07 m from its centre) | None | 0.113 | 0.134 | 3.99 | 0.388 | 0.258 |
| v208 | ref base (parallel) | off | True | 5 | 60.9 | landing pad reached (0.08 m from its centre) | None | 0.127 | 0.127 | 4.26 | 0.358 | 0.272 |
| v209 | ref base (parallel) | off | False | 2 | 25.0 | unhealthy: estimator invalid (xy_valid=False, z_valid=True) | 12.8 | 0.13 | 0.332 | 1.75 | 0.865 | 0.294 |
| v210 | ref base (parallel) | off | True | 5 | 67.9 | landing pad reached (0.07 m from its centre) | None | 0.111 | 0.113 | 2.25 | 0.323 | 0.247 |
| v211 | ref base (parallel) | off | True | 5 | 61.6 | landing pad reached (0.07 m from its centre) | None | 0.105 | 0.119 | 2.51 | 0.34 | 0.269 |
| v212 | ref base (parallel) | off | True | 5 | 64.6 | landing pad reached (0.08 m from its centre) | None | 0.11 | 0.122 | 4.61 | 0.361 | 0.26 |
| v213 | ref base (parallel) | off | True | 5 | 81.2 | landing pad reached (0.08 m from its centre) | None | 0.119 | 0.126 | 3.11 | 0.36 | 0.208 |
| v214 | ref base (parallel) | off | True | 5 | 72.3 | landing pad reached (0.08 m from its centre) | None | 0.114 | 0.114 | 2.58 | 0.38 | 0.238 |
| v500 | ref residual (parallel) | ON: acted 31 planning steps, zero 3 ({'not navigating' | False | 0 | 2.6 | unhealthy: estimator invalid (xy_valid=False, z_valid=True) | None | None | None | 0.0 | None | None |
| v501 | ref residual (parallel) | ON: acted 147 planning steps, zero 11 ({'not navigatin | False | 1 | 20.8 | unhealthy: estimator invalid (xy_valid=False, z_valid=True) | 11.0 | 0.149 | 0.149 | 1.47 | 0.834 | 0.195 |
| v502 | ref residual (parallel) | ON: acted 0 planning steps, zero 0 ({}) | False | 0 | None |  | None | None | None | 0.0 | None | None |
| v503 | ref residual (parallel) | ON: acted 0 planning steps, zero 0 ({}) | False | 0 | None |  | None | None | 0.097 | 0.0 | 0.319 | 0.02 |
| v504 | ref residual (parallel) | ON: acted 84 planning steps, zero 9 ({'not navigating' | False | 1 | None |  | None | 0.219 | 0.219 | 2.72 | 0.59 | 0.236 |
| v505 | ref residual (parallel) | ON: acted 0 planning steps, zero 0 ({}) | False | 0 | None |  | None | None | None | 0.0 | None | None |
| v506 | ref residual (parallel) | ON: acted 0 planning steps, zero 0 ({}) | False | 0 | None |  | None | None | None | 0.0 | None | None |
| v507 | ref residual (parallel) | ON: acted 96 planning steps, zero 18 ({'not navigating | False | 0 | 12.5 | unhealthy: estimator invalid (xy_valid=False, z_valid=True) | None | None | None | 0.0 | None | None |
| v508 | ref residual (parallel) | ON: acted 0 planning steps, zero 0 ({}) | False | 0 | None |  | None | None | None | 0.0 | None | None |
| v509 | ref residual (parallel) | ON: acted 0 planning steps, zero 0 ({}) | False | 0 | None |  | None | None | 0.044 | 0.0 | 0.99 | 0.033 |

## Reading

1. Unloaded and serial, the base is solid: 5/5 complete, NAVIGATE 67.1 +/- 6.1 s, no VIO divergence.
2. The residual completed 4/5; v603 cleared checkpoint 1, then looped on "escape" at Gate A (maze exit)
   until the flight time limit (NAVIGATE 167 s journal), with |yaw rate| 0.84 rad/s in truth early in NAVIGATE.
3. Completed residual runs took 70.7 +/- 6.4 s vs 67.1 +/- 6.1 s base: no speed gain (diff within 1 sd).
4. Corner yv peak is higher with the residual (0.157 +/- 0.030 vs 0.114 +/- 0.002), and time above
   yv 0.10 doubles (8.7 vs 4.4 s): it worsens the very quantity round 2 tried to penalise.
5. Nothing diverged in the 9 completed flights, so the higher yv did not trip VIO here, but the margin shrank.
6. The parallel v500-v509 0/10 (most never reached NAVIGATE) was dominated by host load; the serial block
   removes that confound: residual ON flies the full course 4/5 times.
7. Base serial matches the earlier base block v200-v214 (14/15, 65.8 +/- 5.7 s), so the host is comparable.
8. With n=5 per arm, 4/5 vs 5/5 is not statistically separable, but every metric points the same way.
9. Verdict: in the unloaded serial setting the residual is the same-to-worse than its own `--no-residual` base
   (no time gain, higher corner yaw-rate x speed, one maze-exit failure). Do not adopt it for flight.
10. Harness notes (same for both arms, also seen in earlier blocks): launch log ends with the smoke-test
    assertion "no advancing image streams observed"; post_flight fails on missing course-mission.json after
    writing truth-poses.csv, so no videos were encoded. Neither affects the scoring inputs.

Files: `gazebo_ab_serial/vNNN/` hold each run's native-program.log, journal, result.json, post.log
(ULogs, bags, images not copied; full dirs remain on GB10 `~/dronehacks-2026-gb10/artifacts/`).
