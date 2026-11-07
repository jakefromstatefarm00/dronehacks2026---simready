#!/usr/bin/env bash
# Bring up the organizer sim profile in README order: PX4 SITL (T1/step 1), Isaac Sim +
# Pegasus (step 2), the stack (steps 3-6). Each runs in its own session; PIDs (= process
# group ids) go to <run-dir>/pids so stop_all.sh can stop exactly these processes.
#   run_all.sh <run-dir> [stack launch args...]
set -eo pipefail
RUN="$(realpath -m "$1")"; shift
HERE="$(cd "$(dirname "$0")" && pwd)"
mkdir -p "$RUN"; : > "$RUN/pids"
source "$HERE/env.sh"
# Camera x stays at the organizer default 0.10 (square4 completed with it once the view
# direction was fixed). If you change it, SKYBETA_CAMERA_X and the launch's camera_x must
# change together (their README); this script passes the same value to both.
export SKYBETA_CAMERA_X="${SKYBETA_CAMERA_X:-0.10}"
export SKYBETA_CAMERA_VIEW_ROLL_DEG="${SKYBETA_CAMERA_VIEW_ROLL_DEG:-180}"   # see isaac/pegasus_skybeta_course.py
start() { setsid bash "$HERE/$2" "${@:3}" > "$RUN/$1.log" 2>&1 < /dev/null & echo "$1 $!" >> "$RUN/pids"; }
start px4 run_px4.sh "$RUN/px4"
for _ in $(seq 60); do grep -q "Waiting for simulator" "$RUN/px4.log" 2>/dev/null && break; sleep 1; done
start isaac run_isaac.sh
for _ in $(seq 300); do grep -q "Startup script returned successfully" "$RUN/px4.log" 2>/dev/null && break; sleep 2; done
grep -q "Startup script returned successfully" "$RUN/px4.log" || { echo "PX4 never linked with Pegasus"; exit 1; }
start stack run_stack.sh camera_x:="$SKYBETA_CAMERA_X" "$@"
echo "up: $(tr '\n' ' ' < "$RUN/pids")"
