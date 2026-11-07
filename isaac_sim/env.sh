# Shared paths for the organizer-sim scripts. Override any of these in the environment.
_ENV_HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# Isaac Sim 5.1.0, Pegasus v5.1.0, PX4 v1.16.2 build, Isaac ROS prefix, stack_ws live here (see README).
export ORGANIZER_SIM="${ORGANIZER_SIM:-$HOME/organizer-sim}"
# The organizer's skybeta repo (sim/, src/). Default: ../skybeta next to this folder.
if [ -z "$SKYBETA_REPO" ]; then
  SKYBETA_REPO="$(realpath -m "$_ENV_HERE/../skybeta")"
  [ -f "$SKYBETA_REPO/sim/px4_sitl.env" ] || SKYBETA_REPO="$ORGANIZER_SIM/simready/skybeta"
fi
export SKYBETA_REPO
# colcon workspace with the organizer's skybeta_flight_interface, oakd_vslam_bringup and px4_msgs.
export SKYBETA_WS="${SKYBETA_WS:-$HOME/dronehacks-runtime/skybeta_ws}"
# colcon workspace with the organizer's competitor_example (square_mission).
export COMPETITOR_WS="${COMPETITOR_WS:-$ORGANIZER_SIM/stack_ws}"
export XRCE_AGENT_DIR="${XRCE_AGENT_DIR:-$HOME/dronehacks-runtime/Micro-XRCE-DDS-Agent/build}"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-55}"
export ORGANIZER_SIM_SCRIPTS="$_ENV_HERE"
unset _ENV_HERE
