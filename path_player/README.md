# path_player

**The inverse of `path_recorder` for Sky Beta.**

`path_player` reads path recordings produced by `path_recorder` (`points.csv` and `meta.json`) and generates setpoint commands published to `/skybeta/setpoint/pose` in accordance with the Sky Beta competitor flight contract.

---

## How It Works

### Coordinate Frame Inversion

`path_recorder` saved each waypoint in the **arm frame**, relative to the drone's pose when it was armed on the ground:
* $x$: metres forward (along the drone's heading at arming)
* $y$: metres left
* $z$: metres up ($0 = \text{ground height when armed}$)
* $\text{yaw}$: radians CCW ($0 = \text{heading at arming}$)

When replaying on the drone, `path_player` reads the drone's current odometry pose at arming $(p_0, \text{yaw}_0)$ and computes the ENU `odom` setpoint for every point or interpolated step:

$$\begin{pmatrix} x_{\text{odom}} \\ y_{\text{odom}} \\ z_{\text{odom}} \end{pmatrix} = \begin{pmatrix} p_{0,x} \\ p_{0,y} \\ p_{0,z} \end{pmatrix} + \begin{pmatrix} \cos(\text{yaw}_0) & -\sin(\text{yaw}_0) & 0 \\ \sin(\text{yaw}_0) & \cos(\text{yaw}_0) & 0 \\ 0 & 0 & 1 \end{pmatrix} \begin{pmatrix} x \\ y \\ z \end{pmatrix}$$

$$\text{yaw}_{\text{odom}} = \text{wrap}(\text{yaw}_0 + \text{yaw})$$

$$q = \left(0.0,\; 0.0,\; \sin\left(\frac{\text{yaw}_{\text{odom}}}{2}\right),\; \cos\left(\frac{\text{yaw}_{\text{odom}}}{2}\right)\right)$$

This is published as a `geometry_msgs/PoseStamped` on `/skybeta/setpoint/pose` with `header.frame_id = 'odom'`.

---

## Safety & Flight Contract

`path_player` fully adheres to the Sky Beta competitor rules:
1. **Never touches `/fmu/*`**: Only publishes to `/skybeta/setpoint/pose`. `skybeta_flight_interface` handles NED conversion, rate limits, and safety envelope clamping.
2. **Safe Pre-Offboard Streaming**: While waiting for Offboard mode, the node streams the drone's current position at 20 Hz. This provides the continuous setpoint stream PX4 requires to enter Offboard mode without any sudden jumps.
3. **Arm & Switch Handshake**:
   - Pilot arms the drone on the starting pad $\to$ node latches arm anchor $(p_0, \text{yaw}_0)$.
   - Pilot switches RC switch to **Position** $\to$ `flight_interface` enters **Offboard** mode (`offboard_active: true`).
   - Node detects `offboard_active: true` and begins trajectory replay.
4. **Pilot Override**: If the safety pilot takes control at any moment (switching modes or moving the sticks), `offboard_active` drops to `false`. Replay immediately **pauses** and stops advancing.
5. **End of Path**: Once all waypoints are flown, the node transitions to `HOLD_END`, holding the final waypoint stably until the pilot takes over or lands.

---

## Playback Modes

Configured via the `mode` parameter:

| Mode | Description |
| :--- | :--- |
| **`adaptive`** *(default)* | Follows recorded timestamps $t$, smoothly interpolating between points. If the drone lags behind target by $> \text{max\_lag\_m}$ (e.g. fighting wind or climbing), trajectory clock automatically pauses until the drone catches up. |
| **`time`** | Exact time pacing following recorded timestamps $t$, scaled by `speed_factor`. |
| **`waypoint`** | Advances to waypoint $i+1$ only when the drone arrives within `arrival_radius_m` of waypoint $i$. |

---

## ROS Parameters

| Parameter | Default | Description |
| :--- | :--- | :--- |
| `recording_path` | `""` | Path to recording folder. Empty string auto-selects newest recording in runs directory. |
| `mode` | `"adaptive"` | Playback mode: `"adaptive"`, `"time"`, or `"waypoint"`. |
| `speed_factor` | `1.0` | Speed multiplier (e.g., `0.8` for 80% speed replay, `1.2` for 120%). |
| `arrival_radius_m`| `0.25` | Waypoint arrival tolerance in metres (for `waypoint` mode). |
| `max_lag_m` | `0.5` | Maximum allowed lag in metres before adaptive playback pauses time. |
| `command_yaw` | `true` | Whether to command yaw orientation from the recording. |
| `anchor_mode` | `"armed"` | Anchor source: `"armed"` (when armed), `"offboard"` (when Offboard starts), `"recorded"` (use meta.json anchor), or `"fixed"`. |
| `hold_at_end` | `true` | Maintain setpoint hold at final waypoint when complete. |
| `publish_rate_hz` | `20.0` | Setpoint publication frequency. |

---

## Usage

### 1. Inspect a Recording Before Flight
Inspect any recording to see duration, distance, waypoint count, and flags:

```bash
ros2 run path_player inspect
# or specify a path:
ros2 run path_player inspect ~/team_ws/runs/paths/path-20261108-112030-001
```

### 2. Manual Launch
Launch with default settings (auto-plays latest recording at 1.0x speed):
```bash
ros2 launch path_player path_player.launch.py
```

Launch with custom speed factor and mode:
```bash
ros2 launch path_player path_player.launch.py speed_factor:=0.8 mode:=adaptive
```

Launch specifying an explicit recording path:
```bash
ros2 launch path_player path_player.launch.py recording_path:=/home/dronehacks/team_ws/runs/paths/path-20261108-112030-001
```

If `skybeta_flight_interface` is already running in another terminal / launch:
```bash
ros2 launch path_player path_player.launch.py interface:=false
```

### 3. Autostart on Drone Boot
To fly a recorded path automatically on drone boot:
In `/etc/skybeta/drone.env`:
```bash
COMPETITOR_PKG=path_player
COMPETITOR_LAUNCH=path_player.launch.py
```

### 4. Monitor Playback Telemetry
The node publishes real-time status at 2 Hz on `/path_player/status`:
```bash
ros2 topic echo /path_player/status
```
Example JSON payload:
```json
{
  "state": "PLAYING",
  "folder": "/home/dronehacks/team_ws/runs/paths/path-20261108-112030-001",
  "total_points": 142,
  "current_idx": 45,
  "progress_pct": 31.7,
  "rem_distance_m": 8.42,
  "lag_m": 0.12,
  "speed_factor": 1.0,
  "offboard_active": true,
  "armed": true,
  "mode": "OFFBOARD",
  "anchor_pos": [1.25, 0.42, 0.08],
  "anchor_yaw_deg": 42.1,
  "target_pos": [2.15, 1.22, 1.20],
  "drone_pos": [2.10, 1.20, 1.18]
}
```

---

## Deployment on Jetson

1. Copy `path_player` to the Jetson workspace:
   ```bash
   scp -r /home/abhinav/DroneHacks/path_player dronehacks@192.168.55.1:~/team_ws/src/
   ```
2. Build the package:
   ```bash
   cd ~/team_ws
   colcon build --symlink-install --packages-select path_player
   source install/setup.bash
   ```
3. Run tests:
   ```bash
   pytest src/path_player/test/test_replay.py
   ```
