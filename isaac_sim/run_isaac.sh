#!/usr/bin/env bash
# T1 of the organizer sim profile (skybeta/sim/README.md step 2): Isaac Sim 5.1.0 +
# Pegasus v5.1.0 running the organizer's sim/pegasus_skybeta.py, unmodified, headless.
# Every knob is the script's own SKYBETA_* environment variable (see its header).
set -eo pipefail
source "$(dirname "${BASH_SOURCE[0]}")/env.sh"
SIM="$ORGANIZER_SIM"
source "$SIM/pegasus_env.sh"
export PX4_DIR="${PX4_DIR:-$SIM/PX4-Autopilot-v1.16.2}"
export SKYBETA_HEADLESS="${SKYBETA_HEADLESS:-1}"
export SKYBETA_ORGANIZER_SCRIPT="${SKYBETA_ORGANIZER_SCRIPT:-$SKYBETA_REPO/sim/pegasus_skybeta.py}"
SCRIPT="${1:-$ORGANIZER_SIM_SCRIPTS/isaac/pegasus_skybeta_course.py}"
isaac_run "$SCRIPT"
