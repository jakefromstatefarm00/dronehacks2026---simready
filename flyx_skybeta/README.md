# flyx_skybeta: run a course in FlyX on the drone's own interface

On the drone, the organizer's `skybeta_flight_interface` is the only program on `/fmu`. Team
code reads `/visual_slam/tracking/odometry` and `/skybeta/state` and publishes
`/skybeta/setpoint/pose`. The safety pilot arms and lands (`skybeta/docs/competitor_guide.md`).
`run.sh` builds the same arrangement in the FlyX lab, so the flight code you test is the code that
flies the drone.

```
cd <repo root>                  # the folder that has skybeta/, suraj_course/, jolie_course/, team_ws/
bash flyx_skybeta/run.sh suraj  # suraj_course: Suraj's course flight
bash flyx_skybeta/run.sh jolie  # jolie_course: Jolie's online path planner (+ FlyX depth camera)
```

Before each run: **Reset world**, with the vehicle on the launch mark facing up the first
corridor, and get the team's OK. Ctrl-C makes the sim pilot land.

`run.sh` starts three things:

1. **`skybeta/src/skybeta_flight_interface`**, the organizer's code, unmodified, run from source.
   It uses the drone's setting, `auto_offboard_on_arm`: arming then switching to Position starts
   Offboard. It also enables `allow_mode_request` (SITL only) for `PILOT=direct`.
2. **The program,** on the Sky Beta contract. It never arms, never changes mode and never lands.
   At the end it holds over the pad and reports `HOLD` (jolie: once centred, `ready_to_land`).
3. **`team_ws/src/team_mission/team_mission/sim/sim_pilot.py`** (Jake's). It is **SIM ONLY**
   and refuses to start on the drone. It does the safety pilot's job, the real way: arm in
   Altitude, then switch to Position. The interface then enters Offboard on its own. When the
   program reports `HOLD` or `LAND`, or after 240 s, the pilot lands.

   If PX4 refuses the Altitude handshake, run `PILOT=direct bash flyx_skybeta/run.sh ...`
   instead: arm in Hold, then call `/skybeta/request_offboard`.

On the drone, none of this folder is used: the interface starts at power-up, the human pilot arms
and lands, and you run the program directly (`python3 suraj_course/suraj_course.py`).

Logs go to `flyx_skybeta/runs/<program>-<time>/`: `flight_interface.log`, `program.log`,
`pilot.log`. Each program also writes its own journal (`suraj_course/runs/`, `jolie_course/runs/`).

Needs ROS 2 Jazzy with `px4_msgs`, `geometry_msgs`, `nav_msgs` and `std_srvs`, as in the lab.
`jolie` also needs numpy and scipy.

**Status:** offline tests pass (suraj_course 15, pathfinding 168, team_mission 22). The Sky Beta
chain started on the laptop's Isaac (the interface linked to PX4; the program received odometry and
state), but has not yet flown end to end. The first FlyX run is the test.
