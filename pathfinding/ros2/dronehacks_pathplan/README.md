# dronehacks_pathplan — our planner as a skybeta competitor node

Fork of the organisers' `competitor_example` (`square_mission.py`): **same contract**
(`/skybeta/state` + `/visual_slam/tracking/odometry` in, `/skybeta/setpoint/pose` out,
nothing on `/fmu/*`, never arms) — the square is replaced by the online Theta\* planner
from `pathfinding/pathplan` (13/13 perturbed courses offline; see `pathfinding/README.md`).

```
WAIT_FOR_OFFBOARD  send the current position (lets flight_interface enter Offboard)
TAKEOFF            straight up to flight_altitude_m (0.9 m above the floor)
NAVIGATE           planner through the 5 checkpoints; carrot 0.5 m ahead, look-ahead yaw
HOLD               over the landing pad at the envelope floor (no land command exists: operator lands)
PAUSED             Offboard lost: hold, stop advancing
```

**Tested:** `colcon build` + `ros2 run` on ROS 2 Jazzy against a fake flight interface
(`test/fake_skybeta.py`): waited for Offboard → took off to 0.9 m → all 5 checkpoints →
held 0.00 m from the pad centre. **Not yet run in FlyX** (real dynamics, cuVSLAM, depth).

## Run it in FlyX / on the drone

The node imports the planner and the published course from the same repo checkout
(`pathfinding/`), so clone the **whole repo** into the team workspace (colcon finds this
package in it). If you only need FlyX, use the single file `pathfinding/flyx/pathplan_mission.py`.

```bash
cd ~/team_ws/src && git clone https://github.com/jakefromstatefarm00/dronehacks2026---simready.git
team_build.sh --packages-select dronehacks_pathplan        # or: colcon build --symlink-install
python3 -c "import numpy, scipy, yaml"                       # needed; see below if one is missing

# 1. shadow first: plans and publishes ~/path, sends NO setpoints (Offboard will not start)
ros2 launch dronehacks_pathplan pathplan.launch.py shadow:=true
# 2. fly (sim: allow_mode_request:=true lets you request Offboard without a pilot)
ros2 launch dronehacks_pathplan pathplan.launch.py
ros2 topic echo /dronehacks_pathplan/status                  # state, next checkpoint, cleared
```

On the drone, the stack's `competitor` window launches `COMPETITOR_PKG`/`COMPETITOR_LAUNCH`
from `/etc/skybeta/drone.env`: set them to `dronehacks_pathplan` / `pathplan.launch.py`.
Missing Python packages go in a team image (`host/run.sh build --init`), not `pip` in a
running container (competitor guide §5).

| Launch argument | Default | |
|---|---|---|
| `depth_topic`, `camera_info_topic` | empty | a depth `sensor_msgs/Image` (16UC1/32FC1); empty = published map only |
| `flight_altitude_m` | `0.9` | above the floor; walls are 1.73 m (rule 7.7) |
| `carrot_m` | `0.5` | pose setpoints are not speed-limited: a short carrot keeps it slow |
| `planner` | `theta` | `astar`, or `static` (no sensing, baseline) |
| `shadow` | `false` | `true` = plan only |
| `pathfinding_path` | auto | `<repo>/pathfinding` if not found from the checkout |
| flight_interface args | as `competitor.launch.py` | `max_altitude_m`, `min_altitude_m`, `allow_mode_request` (SIM ONLY), ... |

## Must be true / check first

- **Start on the launch mark facing up the first corridor (course +y).** cuVSLAM's `odom`
  origin and axes are the startup pose; `launch_xy_m` / `launch_heading_course_rad`
  (node parameters) turn it into the published map's frame. A different start = wrong map.
- **Depth:** the stock drone camera config publishes no depth (cuVSLAM stereo only); find
  out whether FlyX has a depth topic (`ros2 topic list | grep -i depth`). Without one the
  node flies the published map only.
- **Camera offset** `camera_offset_flu_m` (default Sky Beta CAD `0.115 0 0.029`): use
  `CAMERA_X/Y/Z` from `drone.env` on hardware.
- **Landing:** `flight_interface` has no land command and floors setpoints at 0.3 m, so the
  node ends holding over the pad; checkpoint 5 (land + disarm) needs the operator/pilot.
