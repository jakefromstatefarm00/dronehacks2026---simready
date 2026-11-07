# isaac_sim/ — the organizer's skybeta sim profile, actually run (Isaac Sim 5.1 + Pegasus + PX4 1.16.2 + cuVSLAM)

The organizer's `skybeta/sim/` (README: *"a spike… Nothing here has been executed"*) brought
up end to end on the team GB10 (DGX Spark, aarch64), headless, recorded, and flying the
organizer's own `competitor_example/square_mission.py`.

## Which simulator is which (read this first)

| | **This folder (organizer skybeta profile)** | **FlyX / DroneHacks Lab** |
|---|---|---|
| What it models | the **real drone's** stack: cuVSLAM → `px4_odom_bridge` → PX4, team code talks only to `skybeta_flight_interface` | the cloud lab in `AI_CONTEXT.md`: native PX4, team code publishes `/fmu/in/*` itself |
| Team code interface | `/skybeta/setpoint/pose|twist`, `/skybeta/state`, `/visual_slam/tracking/odometry` | `/fmu/in/offboard_control_mode`, `trajectory_setpoint`, `vehicle_command`; `/fmu/out/*_v1`; `/camera/depth/image_raw` |
| Runs | `square_mission.py`, `pathfinding/flyx/pathplan_mission.py`, `isaac_sim/flyx/waypoint_mission.py` | `pathfinding/flyx/upload_to_flyx.py` ("Run saved file") |

Skybeta-contract nodes **wait for Offboard forever in FlyX** (no `/skybeta/state` there; see
`pathfinding/flyx/README.md`). FlyX-native files publish `/fmu/in/*`, which the organizer
rules forbid on the drone. Keep the two apart.

### The command for FlyX

Upload `pathfinding/flyx/upload_to_flyx.py` with **Run saved file** (no arguments: it flies;
Reset world first, launch mark facing up the first corridor). From a lab terminal instead:

```bash
python3 pathfinding/flyx/upload_to_flyx.py --shadow   # plans only, sends nothing
python3 pathfinding/flyx/upload_to_flyx.py            # flies
```

There is no FlyX command for `square_mission.py`: it is a skybeta node. FlyX's equivalent
smoke test is the lab's own `onboard/examples/takeoff_land.py`.

### The command for this sim (on the real drone stack, the organizer's procedure)

```bash
ros2 launch competitor_example competitor.launch.py arrival_radius_m:=0.6
```

`arrival_radius_m:=0.6` is the only change from the organizer's example, through its own
launch argument (see "Findings"). On the drone the pilot arms on Altitude and switches to
Position; in this sim `fly_mission.sh` plays the pilot.

## Result: square_mission completes (run square4)

`results/square4/video/flight.mp4` — left: the onboard left camera cuVSLAM used; right:
top-down ground truth (blue), cuVSLAM (orange), commanded corners (grey).

| | |
|---|---|
| Sequence | hover 1.5 m 5 s → 4 corners of a 5 m square → "square complete — holding" → pilot lands |
| Arm → touchdown | 62.4 s (airborne 56.0 s) |
| Corner error (truth) | 0.01, 0.07, 0.13, 0.00 m |
| PX4 parameters | exactly `skybeta/sim/px4_sitl.env` (magnetometer on, baro height, EV_CTRL 9, EV_DELAY 50) |

## Findings (things the organizer's untested spike gets wrong, and what we did)

1. **Cameras look backwards.** Pegasus `MonocularCamera.start()` hands scipy's `(x,y,z,w)`
   quaternion to Isaac, which reads `(w,x,y,z)`. The organizer's orientation `[0,0,0]` becomes a
   180° turn: the stereo pair stares at the Iris, cuVSLAM locks onto the airframe and reports
   (0,0,0) while the vehicle drifts (run square2). Pegasus's own default `[0,0,180]` is the one
   that comes out forward and upright — the value their README names. `isaac/pegasus_skybeta_course.py`
   adds 180° to the camera **view** only (`SKYBETA_CAMERA_VIEW_ROLL_DEG`); positions and
   cuVSLAM's TFs are untouched. Their `SKYBETA_CAMERA_ROLL` can't be used: it also rotates the
   baseline offset (swaps left/right) and must match the launch TF.
2. **Arming order.** README step 7 arms first, then requests Offboard. PX4 refuses
   ("Preflight Fail: heading estimate not stable"): with an FRD vision frame EKF2 never sets
   `yaw_align`, which blocks arming in the boot mode (AUTO_LOITER). Arming in Offboard is
   accepted. So the sim pilot calls `/skybeta/request_offboard`, then `commander arm`. No PX4
   parameter changed. (The real pilot arms in Altitude with RC; the sim has no transmitter.)
3. **cuVSLAM height ≠ PX4 height.** EKF2_EV_CTRL 9 fuses no vision height; PX4 holds height
   on the barometer, cuVSLAM's z read ~0.55 m low at 1.5 m. `square_mission` checks arrival in
   cuVSLAM's 3-D frame with a 0.3 m radius, so it never "reached" corner 1 (run square3, truth
   was 0.01 m from it). Any skybeta node that checks 3-D arrival (pathplan_mission's takeoff
   uses 0.15 m) needs to tolerate this or check horizontal distance.
4. **Course appearance.** The organizer's sim uses NVIDIA's "Curved Gridroom", not the course.
   `isaac/convert_course.py` imports the published OBJ (z-up metres, already the course frame)
   with triangle colliders. Rendered from its MTL every wall is flat dark grey and cuVSLAM
   diverged once the view was all wall (run course-pathplan1). `isaac/texture_course.py` makes
   an **estimated** textured variant (our Gazebo arena's procedural textures, labelled as such).
5. **GB10-only:** Isaac ROS 4.0's arm64 (Jetson Thor) image converter crashed with
   `cudaErrorIllegalAddress` in the textured run (course-pathplan2). The organizer's x86
   container (Isaac ROS 3.2) should not have this; an x86 RTX machine is the better host.

## Deviations from the organizer sim (all sim-side or sim-pilot; none touch the drone)

- PX4 v1.16.2 SITL built from source; parameters applied via rcS `PX4_PARAM_*` (same values).
- ROS 2 Jazzy + Isaac ROS 4.0 arm64 (`fetch_isaac_ros.py`: unpacked, no sudo; VPI 4.0.5 from
  the Isaac ROS 4.2 repo) instead of Humble + Isaac ROS 3.2 x86 container. Same nodes/params.
- `mirror_oak_topics:=false` (topic_tools not installed; nothing here reads `/oak/*`).
- Camera view fix (finding 1). Sim pilot: Offboard-then-arm (finding 2), `commander land` at the end.
- `ROS_DOMAIN_ID=55` (shared machine).

## Files

| File | Role |
|---|---|
| `env.sh` | paths (`ORGANIZER_SIM`, `SKYBETA_REPO` = `../skybeta`, `SKYBETA_WS`, `COMPETITOR_WS`, `XRCE_AGENT_DIR`) |
| `run_px4.sh`, `run_isaac.sh`, `run_stack.sh` | README terminals T2/T1/T3 |
| `run_all.sh <run-dir>` / `stop_all.sh <run-dir>` | bring-up in README order / stop exactly those processes |
| `check_stack.sh` | README checklist steps 3–6 as one report |
| `fly_mission.sh <run-dir> square` / `... flyx <node.py>` | step 7 with a sim pilot, recorded |
| `record_flight.py`, `encode_video.py` | truth/estimate/setpoint/events + onboard frames → MP4 (offline; no live viewer) |
| `isaac/pegasus_skybeta_course.py` | runs the organizer's `pegasus_skybeta.py` with the course + view fix inserted |
| `isaac/convert_course.py`, `isaac/texture_course.py` | course OBJ → USD (+ estimated textures) |
| `fetch_isaac_ros.py`, `isaac_ros_env.sh` | Isaac ROS 4.0 arm64 into a private prefix |
| `flyx/waypoint_mission.py` | machmoon/dronehacks-sim waypoint controller as a **skybeta** node (built by `tools/build_flyx_waypoint_node.py`); not run yet |

## Install (Ubuntu 24.04, NVIDIA driver ≥ 580; done once under `$ORGANIZER_SIM`)

1. Isaac Sim 5.1.0 standalone zip (aarch64 or x86_64) → `$ORGANIZER_SIM/isaacsim`.
2. `PegasusSimulator` @ v5.1.0; `pegasus_env.sh` = the `isaac_run` block from Pegasus's
   installation docs with `ISAACSIM_PATH` pointed at step 1; `$ISAACSIM_PYTHON -m pip install -e extensions/pegasus.simulator`.
3. PX4-Autopilot v1.16.2: `make px4_sitl_default none` (stop the PX4 it launches afterwards).
4. A colcon workspace with the organizer's `skybeta_flight_interface`, `oakd_vslam_bringup`,
   `px4_msgs` (release/1.16) → `SKYBETA_WS`; one with `competitor_example` → `COMPETITOR_WS`.
5. aarch64: `python3 fetch_isaac_ros.py --prefix $ORGANIZER_SIM/isaac_ros ros-jazzy-isaac-ros-visual-slam ros-jazzy-isaac-ros-image-proc`
   (+ VPI 4.0.5 arm64 from the Isaac ROS 4.2 repo). x86: the organizer's `docker/Dockerfile.sim`.

```bash
bash isaac_sim/run_all.sh ~/runs/square
bash isaac_sim/fly_mission.sh ~/runs/square square        # run with SQUARE_ARGS="arrival_radius_m:=0.6"
python3 isaac_sim/encode_video.py ~/runs/square/flight-square-*/
bash isaac_sim/stop_all.sh ~/runs/square
```
