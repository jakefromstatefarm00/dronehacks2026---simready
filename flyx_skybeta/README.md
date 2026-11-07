# flyx_skybeta: fly in FlyX on the drone's own interface (no team code on /fmu)

On the drone, the organizer's `skybeta_flight_interface` is the only program on `/fmu/in`. Team
code reads `/visual_slam/tracking/odometry` and `/skybeta/state` and publishes
`/skybeta/setpoint/pose`. The safety pilot arms and lands (`skybeta/docs/competitor_guide.md`).
This folder runs that same arrangement in the FlyX lab, so the flight code you test is the code
that flies the drone.

```
cd <repo root>                       # the folder that has flyx_course/, flyx_skybeta/, skybeta/, pathfinding/
bash flyx_skybeta/run.sh course      # friend's route: flyx_course/fly_course.py (Sky Beta mode)
bash flyx_skybeta/run.sh pathfinder  # the planner: pathfinding/flyx/pathplan_mission.py
```

Before each run: **Reset world**, with the vehicle on the launch mark facing up the first
corridor. Get the team's OK. Ctrl-C makes the sim pilot land.

`run.sh` starts three things:

1. **`skybeta/src/skybeta_flight_interface`**, the organizer's code, unmodified, run straight
   from source. It owns `/fmu/in`, as on the drone. `allow_mode_request:=true` turns on the
   sim-only `/skybeta/request_offboard` service.
2. **The program,** on the Sky Beta contract. It never arms, never changes mode and never
   lands. It holds over the pad at the end (status `HOLD`). On a fault it holds where it is.
3. **`sim_pilot.py`, SIM ONLY.** It does the safety pilot's job: once the program streams, it
   requests Offboard and arms; when the program reports `HOLD`, `PAUSED` or `TAKEN_OVER`, or
   after 200 s, or on Ctrl-C, it lands. It uses `/fmu/in/vehicle_command`, like `commander arm`
   at the PX4 shell in the organizer's sim, and nothing else. **On the drone the human pilot
   does this, and this script is never run.**

Logs go to `flyx_skybeta/runs/<program>-<time>/`: `flight_interface.log`, `program.log`,
`pilot.log`. The `fly_course` journal is in `flyx_course/runs/`.

Needs ROS 2 Jazzy with `px4_msgs`, `geometry_msgs`, `nav_msgs` and `std_srvs`, as in the lab.
The pathfinder also needs numpy and scipy.

## Status

- **Offline:** 15 tests (`python3 -m pytest -q flyx_course`), all on the Sky Beta contract with a
  fake pilot. It flies the full route and holds over the pad; it does not move before the pilot
  arms; it holds on an odometry outage; it stops commanding when the pilot takes over.
- **Laptop Isaac + cuVSLAM:** the full chain started. The flight interface linked to PX4, and
  the program received odometry and `/skybeta/state`. The laptop sim's cuVSLAM then ran at about
  3 Hz and PX4 dropped its position estimate, so it never armed. That is a laptop performance
  limit.
- **Not yet flown end to end.** The first FlyX run is the test.
