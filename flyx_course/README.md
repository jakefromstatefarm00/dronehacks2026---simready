# flyx_course: the full course with the friend's waypoint controller

**Default: the Sky Beta contract, the same as on the drone.** Run it in FlyX through
`flyx_skybeta/run.sh course`; see `flyx_skybeta/README.md`. It reads
`/visual_slam/tracking/odometry` and `/skybeta/state`, publishes `/skybeta/setpoint/pose`, and
never arms or lands: the pilot does that, or `flyx_skybeta/sim_pilot.py` in sim.

`--native` is the old FlyX-only mode. It commands PX4 directly on `/fmu/in`, and is kept only as
a fallback until the Sky Beta mode has flown in FlyX. **Never use it on the drone.**

This flies the waypoint controller from
[machmoon/dronehacks-sim](https://github.com/machmoon/dronehacks-sim) (`mission/core.py`,
`CourseController`, unchanged) on their default route (`mission/course.json`), at their
validated speed scale of 0.65. The route has 15 waypoints: corridor, maze, Gate A, tunnel,
chicane, window, then the landing pad. `fly_course.py` is **generated**: edit
`fly_course_template.py`, then run
`python3 flyx_course/build_fly_course.py <path to machmoon/dronehacks-sim>`.

## Which interface: FlyX lab only

The script publishes `/fmu/in/offboard_control_mode`, `/fmu/in/trajectory_setpoint` and
`/fmu/in/vehicle_command`. That is the FlyX lab's contract: native PX4, where team code is the
only setpoint publisher.

It is **not** for the Sky Beta drone or the skybeta sim stack. There `skybeta_flight_interface`
owns `/fmu/in/*`, and team code sends `/skybeta/setpoint/pose` instead ("Do not publish to
`/fmu/in/*` yourself", `skybeta/docs/competitor_guide.md`).

As a guard, the script counts the other publishers on `/fmu/in/offboard_control_mode` and
`/fmu/in/trajectory_setpoint` every second. If any exist (for example, a running flight
interface), it **refuses to arm**. If one appears in flight, it lands.

## How it flies

- **Reference.** The friend's controller steers a virtual reference point along the route. The
  controller sees the reference's own position and velocity, so it plans a clean path.
- **PX4 target.** PX4 is sent a position 0.6 s ahead of the reference, never past the next
  waypoint, plus 40 % of the reference velocity as feedforward. If this program stalls (FlyX:
  up to 1.8 s), PX4 settles near a point on the route instead of flying on. A pure velocity
  command would overshoot by about 0.5 m.
- **Leash.** If the drone falls more than 0.25 m behind, the reference waits for it.
- **Corners.** Within 0.5 m of a turn sharper than 45°, the reference brakes at 0.3 m/s²,
  never below 0.2 m/s, with no lead and no feedforward. Without this, PX4 swung 0.22–0.27 m wide
  at the first maze corner (Gazebo v128, v129). The drone keeps moving through the turn: a full
  stop turns the heading in place, and visual odometry was lost right after one (v131).
- **Divergence.** If the drone is ever more than 0.8 m from the reference, the script lands at
  once. It does not wait for PX4 to flag the estimate.
- **Clock.** The controller runs on its own clock, stepped by at most 0.1 s per tick. Without
  this, a program stall trips its `control_clock_discontinuity` fault.
- **Landing.** At the handoff point over the pad (the friend's `done`), the script sends a
  native PX4 Land.

The route keeps at least 0.405 m from every obstacle on the published map. The Sky Beta
footprint proxy has a radius of 0.28 m.

## Lab robustness (same as flyx_square)

- **Before arming.** Fresh telemetry, then a 10 s settle. Offboard is requested first, then arm.
- **Odometry pauses.** In the air, odometry is the only liveness signal. A pause over 0.4 s
  freezes the reference and holds; over 3 s, the script lands.
- **Slow topics.** Status, land-detected and local position are treated as latest-known state.
  A fresh "estimator invalid" report lands; silence lands only after 10 s.
- **Leaving Offboard.** If PX4 leaves Offboard, the script stops commanding (`TAKEN_OVER`).
- **Stopping.** Ctrl-C or the lab's Stop button makes the script land.
- **Journal.** Written to `flyx_course/runs/course-<time>/journal.jsonl` (else `~/flyx-runs`).
  It contains a `waypoint` event for each waypoint, plus holds, topic gaps and control stalls.

## Tested

- **Offline.** `python3 -m pytest -q flyx_course/test_fly_course.py` runs 15 tests against a
  kinematic PX4 stand-in. They cover:
  - the full course, with every waypoint, the window inside its aperture, and a landing on the
    pad;
  - FlyX square run 2's measured odometry, local-position and program stalls, repeated across
    the whole course;
  - a 1.8 s stall anywhere along the route (stays within 0.2 m of it);
  - run 3's 6 s slow-topic silence;
  - odometry outage, invalid estimate, estimate divergence, leaving Offboard, and the
    foreign-publisher guard.
- **Gazebo, GB10.** Native PX4 v1.16, OpenVINS vision, no truth fed to the estimator, flight
  code on `/fmu/in` as in the lab. Wall clearance is measured on the truth track against the
  course geometry; the footprint proxy is 0.28 m.

| Run | Version | Result | Min wall clearance | Pad error |
|---|---|---|---|---|
| v128 | no corner handling | complete, 15/15 waypoints | 0.235 m | 0.04 m |
| v129 | reference stops at corners | complete | 0.252 m | 0.08 m |
| v130 | — | PX4 refused to arm (sim start-up); stayed on the ground | — | — |
| v131 | stop + slow approach | visual odometry lost in the maze; landed on "estimator invalid" | 0.128 m | — |
| v132 | corner braking, no stop, yaw ≤ 0.5 rad/s | complete | 0.302 m | 0.04 m |
| v133 | `--corner-deg 180` (no corner braking) | complete | 0.184 m | 0.09 m |
| v134 | corner braking, yaw ≤ 0.5 rad/s | visual odometry lost on the same maze leg as v131; the divergence guard landed (truth 3.8 m from the pad) | — | — |
| v135 | corner braking, yaw ≤ 0.5 rad/s | complete | 0.298 m | 0.05 m |
| v136 | + yaw ≤ 0.25 rad/s | **complete** | **0.297 m** | **0.05 m** |
| v137 | yaw ≤ 0.25 | complete (ran 0.15 m low through the window: height estimate) | 0.154 m | 0.05 m |
| v138 | yaw ≤ 0.25 | complete (visual odometry drifted 0.9 m at the maze corner, then recovered) | 0.181 m | 0.05 m |
| v139 | pathfinding/flyx `upload_to_flyx.py`, for comparison | complete | 0.336 m | 0.25 m |
| **v140** | **current: lookahead heading, position-only targets, leash 0.35 m** | **complete** | **0.334 m** | **0.03 m** |
| **v141** | **current** | **complete** | **0.317 m** | **0.02 m** |

**Current version: borrows pathfinding/flyx's smoothness.**
- The nose faces a point 0.7 m ahead on the route, at ≤ 0.25 rad/s, so turns start before
  corners.
- PX4 gets position-only targets, leading the reference by v × 1.0 s and clamped at the next
  waypoint.
- The leash is 0.35 m.

Gazebo v140 and v141 both completed, with ≥ 0.317 m from every wall and landing 0.02–0.03 m
from the pad. Earlier versions lost visual odometry right after 90° corner turns at 0.5 rad/s
(v131, v134, FlyX runs 1–2).

The divergence guard was added after v131. In that run the estimate ran away for about 8 s
before PX4 flagged it.

Options: `--speed-scale` (default 0.65), `--yaw-rate` (default 0.25 rad/s), `--corner-deg` (default 45; 180 turns corner braking
off), `--settle-s`, `--shadow`.
