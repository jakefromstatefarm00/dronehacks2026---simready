# flyx_course: the full course with the friend's waypoint controller

**Sky Beta contract only, the same interface as on the drone. No `/fmu` topics.**

```
bash flyx_skybeta/run.sh course     # FlyX / sim: organizer's flight interface + this + sim pilot
python3 flyx_course/fly_course.py   # on the drone (the interface is already running; the pilot arms)
```

| | Topic |
|---|---|
| reads | `/visual_slam/tracking/odometry` (cuVSLAM, ENU `odom`), `/skybeta/state` |
| writes | `/skybeta/setpoint/pose` (ENU `odom`), `~/status` (JSON: `HOLD` means done, the pilot lands) |

The program never arms, never changes mode and never lands. The safety pilot arms, and the
organizer's `skybeta_flight_interface` enters Offboard and owns PX4. At the end the program holds
over the pad. On any fault it holds where it is. Either way it reports `HOLD`, and the pilot lands.
In sim, `flyx_skybeta/sim_pilot.py` plays the pilot (see `flyx_skybeta/README.md`).

Start on the launch mark, facing up the first corridor (course +y). The course frame is fixed from
the cuVSLAM pose at that moment.

## The algorithm

The controller is [machmoon/dronehacks-sim](https://github.com/machmoon/dronehacks-sim)'s
`CourseController` (`mission/core.py`), embedded verbatim, on their default route
(`mission/course.json`) at their validated speed scale of 0.65. The route has 15 waypoints:
corridor, maze, Gate A, tunnel, chicane, window, then the landing pad. Per-segment speeds,
braking into each waypoint, and an 8 cm arrival radius are theirs.

What this script adds, each change backed by a measured failure:
- **Moving reference.** The controller drives a virtual reference. The pose setpoint leads it by
  v × 1.0 s, never past the next waypoint. If the program stalls (FlyX: up to 1.8 s), the drone
  stops on the route.
- **Heading.** The nose faces the route 0.7 m ahead, at ≤ 0.25 rad/s. Visual odometry was lost
  right after fast 90° corner turns, in Gazebo v131 and v134 and in FlyX runs 1 and 2.
- **Corner braking.** Turns sharper than 45° are braked into at 0.3 m/s², never below 0.2 m/s.
  Without this, PX4 swung 0.22–0.27 m wide at the first maze corner.
- **Leash and divergence.** The reference waits if the drone lags more than 0.35 m. If the drone
  is more than 0.8 m away (the estimate diverged), the program holds.
- **Health.** Odometry paused for more than 0.4 s freezes the reference; after 3 s the program
  holds. If `/skybeta/state` is silent for 10 s, or there is no valid local position, it holds.
  If Offboard ends (pilot takeover), it stops commanding.

`fly_course.py` is **generated**: edit `fly_course_template.py`, then run
`python3 flyx_course/build_fly_course.py <path to machmoon/dronehacks-sim>`.
Options: `--shadow`, `--speed-scale` (0.65), `--yaw-rate` (0.25), `--corner-deg` (45),
`--odometry-topic`. Journal: `flyx_course/runs/course-<time>/journal.jsonl`.

## Tested

- **Offline:** `python3 -m pytest -q flyx_course/test_fly_course.py` runs 15 tests against a fake
  vehicle, flight interface and pilot. They cover:
  - the full route, the window inside its aperture, and holding over the pad;
  - not moving before the pilot arms;
  - FlyX run 2's measured odometry, state and program stalls, repeated across the course;
  - a 1.8 s stall anywhere along the route;
  - odometry outage, invalid local position, pilot takeover, and estimate divergence;
  - no `/fmu` in the program.
- **Gazebo, earlier native version** (same flight core, PX4 driven directly): v140 and v141
  completed, ≥ 0.317 m from every wall (truth), pad error 0.02–0.03 m. The full history from
  v128 is in git log.
- **Sky Beta path:** on the laptop's Isaac + cuVSLAM, the interface linked to PX4 and the program
  received odometry and state. Not yet flown end to end; the first FlyX run is the test.
