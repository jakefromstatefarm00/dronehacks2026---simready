#!/usr/bin/env bash
# T3 of the organizer sim profile (skybeta/sim/README.md step 5): the organizer's
# sim/skybeta_sim.launch.py -- ImageFormatConverter + cuVSLAM (isaac_ros_visual_slam),
# static TFs, MicroXRCEAgent udp4 8888, px4_odom_bridge (FRD).
#
# Deviations from the organizer container, all forced by the GB10 (aarch64, Ubuntu 24.04):
#   * ROS 2 Jazzy + Isaac ROS 4.0 (arm64 apt build, unpacked by fetch_isaac_ros.py)
#     instead of Humble + Isaac ROS 3.2 in the x86_64 container. Same nodes, same params.
#   * mirror_oak_topics:=false: topic_tools is not installed; the /oak/* camera_info/IMU
#     mirrors are only for code written against hardware topic names (neither
#     square_mission nor the FlyX node reads them).
set -eo pipefail
source "$(dirname "${BASH_SOURCE[0]}")/env.sh"
source /opt/ros/jazzy/setup.bash
source "$ORGANIZER_SIM_SCRIPTS/isaac_ros_env.sh"
source "$SKYBETA_WS/install/setup.bash"
export PATH="$XRCE_AGENT_DIR:$PATH"
exec ros2 launch "$SKYBETA_REPO/sim/skybeta_sim.launch.py" mirror_oak_topics:=false "$@"
