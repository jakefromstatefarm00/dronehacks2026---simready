# DroneHacks 2026: team code

All flight code meant for the drone uses the **Sky Beta contract**, the same interface as on the drone. It reads
`/visual_slam/tracking/odometry` and `/skybeta/state` and publishes `/skybeta/setpoint/pose`. Team
code never touches `/fmu`: the organizer's `skybeta_flight_interface` owns PX4, and the safety
pilot arms and lands. Each of those is the same file in sim and on the drone. The one exception is
`jolie_course/upload_to_flyx.py`: the native-PX4 FlyX file that completed the course. It is FlyX-only
(it commands `/fmu/in` itself) and is kept as is.

| Folder | What | Run (FlyX / sim, from this folder) |
|---|---|---|
| `suraj_course/` | Suraj's course flight: machmoon/dronehacks-sim's CourseController on a 15-waypoint route, with a moving reference, corner braking, look-ahead heading and VIO-safe yaw | `bash flyx_skybeta/run.sh suraj` |
| `jolie_course/` (code in `pathfinding/`) | Jolie's online path planner: Theta\* on the published map, corrected by the depth camera. **`upload_to_flyx.py` completed the whole course in FlyX** (native PX4, FlyX only); `jolie_course.py` is the same planner on Sky Beta | `python3 jolie_course/upload_to_flyx.py` (FlyX) or `bash flyx_skybeta/run.sh jolie` |
| `team_ws/src/team_mission/` | Jake's `team_mission` ROS package: a trainable-policy mission, plus the sim pilot used by `run.sh` | `ros2 launch team_mission sim.launch.py` (see its README) |
| `flyx_skybeta/` | sim launcher: the organizer's flight interface + a program + the sim pilot | |
| `skybeta/` | the organizer's stack, unmodified | |

On the drone, the interface starts at power-up and the pilot arms. You only run the program,
e.g. `python3 suraj_course/suraj_course.py`. `flyx_skybeta/` and the sim pilot are sim-only:
they stand in for the power-up stack and the human pilot.

Before any automated run in FlyX: **Reset world**, put the vehicle on the launch mark facing up the
first corridor, and get the team's approval.

Tests: `python3 -m pytest -q suraj_course`, `cd pathfinding && python3 -m pytest -q tests`,
`cd team_ws/src/team_mission && python3 -m pytest -q test`.
