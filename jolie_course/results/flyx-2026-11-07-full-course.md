# FlyX full-course run: jolie_course/upload_to_flyx.py (2026-11-07)

This run completed the whole course: all checkpoints, then a landing on the pad. The journal
is `/home/flyx/flyx-runs/flyx-20261107-205352/journal.jsonl` in the lab.

| Phase | Lab wall clock |
|---|---|
| WAIT_HEALTH -> STREAM -> WAIT_OFFBOARD -> WAIT_ARM -> TAKEOFF | 3.9 s |
| TAKEOFF -> NAVIGATE (at altitude) | 7.0 s |
| **NAVIGATE (the course)** | **120.9 s** |
| LAND ("landing pad reached, 0.07 m from its centre") -> DONE (landed and disarmed) | 9.3 s |
| **arm -> disarm** | **137.1 s** |

The lab simulates at about 0.8x real time (earlier FlyX journals).

```
[INFO] [1794084833.118250559] [flyx_pathplan]: flyx_pathplan up: EXECUTE; journal /home/flyx/flyx-runs/flyx-20261107-205352
[INFO] [1794084833.130988309] [flyx_pathplan]: state None -> WAIT_HEALTH waiting for fresh pose, valid estimator, landed + disarmed
[INFO] [1794084833.182118469] [flyx_pathplan]: odom: /fmu/out/vehicle_odometry
[INFO] [1794084833.182865259] [flyx_pathplan]: status: /fmu/out/vehicle_status_v1
[INFO] [1794084833.183534509] [flyx_pathplan]: land: /fmu/out/vehicle_land_detected
[INFO] [1794084833.184234769] [flyx_pathplan]: local: /fmu/out/vehicle_local_position_v1
[INFO] [1794084833.184899949] [flyx_pathplan]: failsafe_flags: /fmu/out/failsafe_flags
[INFO] [1794084835.681235408] [flyx_pathplan]: state WAIT_HEALTH -> STREAM streaming hold setpoints
[INFO] [1794084836.681098515] [flyx_pathplan]: state STREAM -> WAIT_OFFBOARD requesting Offboard
[INFO] [1794084836.731033684] [flyx_pathplan]: state WAIT_OFFBOARD -> WAIT_ARM Offboard active, requesting arm
[INFO] [1794084836.981164101] [flyx_pathplan]: state WAIT_ARM -> TAKEOFF takeoff
[INFO] [1794084843.981171537] [flyx_pathplan]: state TAKEOFF -> NAVIGATE at altitude
[INFO] [1794084964.831161872] [flyx_pathplan]: state NAVIGATE -> LAND landing pad reached (0.07 m from its centre)
[INFO] [1794084974.081171643] [flyx_pathplan]: state LAND -> DONE landed and disarmed
```

The journal file (`journal.jsonl`) has not been uploaded yet. It holds the setpoints, poses,
stalls and planner events.
