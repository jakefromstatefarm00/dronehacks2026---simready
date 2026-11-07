#!/usr/bin/env bash
# The organizer README's checklist (steps 3-6) as one non-interactive report.
source /opt/ros/jazzy/setup.bash
source "$(dirname "${BASH_SOURCE[0]}")/env.sh"
source "$ORGANIZER_SIM_SCRIPTS/isaac_ros_env.sh"
source "$SKYBETA_WS/install/setup.bash"
T="timeout ${CHECK_TIMEOUT:-8}"
echo "== step 3: /fmu topics: $(ros2 topic list | grep -c '^/fmu')"
$T ros2 topic echo /fmu/out/vehicle_status_v1 --once --field nav_state 2>/dev/null | head -1 | sed 's/^/   vehicle_status_v1.nav_state: /'
echo "== step 4: Pegasus topics"; ros2 topic list | grep drone1 | sed 's/^/   /'
for t in /drone1/left/color/image_raw /drone1/sensors/imu /drone1/state/pose; do
  echo "   $t: $($T ros2 topic hz $t 2>/dev/null | grep -m1 'average rate' )"; done
$T ros2 topic echo /drone1/left/color/image_raw --field encoding --once 2>/dev/null | head -1 | sed 's/^/   encoding: /'
echo "== step 5: cuVSLAM"
echo "   /oak/left/image_mono: $($T ros2 topic hz /oak/left/image_mono 2>/dev/null | grep -m1 'average rate')"
$T ros2 topic echo /visual_slam/status --field vo_state --once 2>/dev/null | head -1 | sed 's/^/   vo_state: /'
echo "   /visual_slam/tracking/odometry: $($T ros2 topic hz /visual_slam/tracking/odometry 2>/dev/null | grep -m1 'average rate')"
echo "== step 6: bridge -> PX4"
echo "   /fmu/in/vehicle_visual_odometry: $($T ros2 topic hz /fmu/in/vehicle_visual_odometry 2>/dev/null | grep -m1 'average rate')"
$T ros2 topic echo /fmu/out/estimator_status_flags --once 2>/dev/null | grep -E "cs_ev_pos|cs_ev_yaw|cs_baro_hgt" | sed 's/^/   /'
