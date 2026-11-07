#!/usr/bin/env bash
# T2 of the organizer sim profile (skybeta/sim/README.md step 1): PX4 SITL v1.16.2,
# hand-launched from a fixed run directory, airframe 10015_gazebo-classic_iris
# (MAVLink simulator on TCP 4560; no Gazebo). The parameter block is exactly the one
# in skybeta/sim/px4_sitl.env, applied through rcS's PX4_PARAM_* hook instead of
# typing it at pxh> (-d: no console in an unattended run).
set -eo pipefail
source "$(dirname "${BASH_SOURCE[0]}")/env.sh"
SIM="$ORGANIZER_SIM"
export PX4_DIR="${PX4_DIR:-$SIM/PX4-Autopilot-v1.16.2}"
# ROS_DOMAIN_ID is exported by env.sh before px4_sitl.env, which would default it to 0
source "$SKYBETA_REPO/sim/px4_sitl.env"
export PX4_PARAM_UXRCE_DDS_SYNCT=0
export PX4_PARAM_EKF2_EV_CTRL=9
export PX4_PARAM_EKF2_EV_DELAY=50
export PX4_PARAM_EKF2_EV_NOISE_MD=0
export PX4_PARAM_EKF2_HGT_REF=0
export PX4_PARAM_EKF2_BARO_CTRL=1
export PX4_PARAM_EKF2_GPS_CTRL=0
export PX4_PARAM_EKF2_OF_CTRL=0
export PX4_PARAM_EKF2_RNG_CTRL=0
export PX4_PARAM_COM_RCL_EXCEPT=4
RUN="${1:-$PX4_RUN_DIR}"
mkdir -p "$RUN" && cd "$RUN"
exec "$PX4_SITL_BIN" "$PX4_ROMFS" -s "$PX4_RC_SCRIPT" -i 0 -d
