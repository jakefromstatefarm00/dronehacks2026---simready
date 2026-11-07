# WSL side of the laptop setup (Isaac Sim on Windows, PX4 + stack in WSL2 Ubuntu 24.04, mirrored
# networking). Source before the run_*.sh scripts:  source windows/laptop_env.sh
export ORGANIZER_SIM="$HOME/organizer-sim"
export SKYBETA_REPO="$ORGANIZER_SIM/simready/skybeta"
export SKYBETA_WS="$ORGANIZER_SIM/skybeta_ws"           # flight_interface, oakd_vslam_bringup, competitor_example
export COMPETITOR_WS="$ORGANIZER_SIM/skybeta_ws"
export XRCE_AGENT_DIR="$HOME/dronehacks-runtime/Micro-XRCE-DDS-Agent/build"
export CUDA13_LIB="$ORGANIZER_SIM/cuda-venv/lib/python3.12/site-packages/nvidia/cu13/lib"
export PX4_DIR="$ORGANIZER_SIM/PX4-Autopilot-v1.16.2"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-55}"
# Windows <-> WSL share one IP in mirrored mode: force UDP (Fast DDS would pick shared memory)
_here="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
export FASTRTPS_DEFAULT_PROFILES_FILE="$_here/fastdds_udp_only.xml"
export FASTDDS_DEFAULT_PROFILES_FILE="$FASTRTPS_DEFAULT_PROFILES_FILE"
# px4_msgs lives in an underlay workspace on this machine
[ -f "$HOME/dronehacks-runtime/ros_ws/install/setup.bash" ] && export LAPTOP_PX4_MSGS_WS="$HOME/dronehacks-runtime/ros_ws"
unset _here
