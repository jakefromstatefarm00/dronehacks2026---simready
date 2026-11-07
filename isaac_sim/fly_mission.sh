#!/usr/bin/env bash
# Organizer README step 7 against a sim brought up by run_all.sh, recorded.
#   fly_mission.sh <run-dir> square            # competitor_example square_mission (5 m square)
#   fly_mission.sh <run-dir> flyx <node.py>    # a FlyX single-file node (e.g. pathplan_mission.py)
# Sequence (as the README): competitor.launch.py allow_mode_request:=true (flight_interface
# + the competitor), `commander arm`, then /skybeta/request_offboard. The organizer's
# competitor nodes never land ("the safety pilot can take over and land"), so the sim
# pilot sends `commander land` when the mission says it is holding at the end.
# Extra env: MISSION_TIMEOUT_S (default 400), ARM_DELAY_S (default 15).
set -o pipefail
RUN="$(realpath -m "$1")"; MODE="$2"; NODE="$3"
HERE="$(cd "$(dirname "$0")" && pwd)"
source "$HERE/env.sh"
SIM="$ORGANIZER_SIM"
source /opt/ros/jazzy/setup.bash
source "$HERE/isaac_ros_env.sh"
source "$SKYBETA_WS/install/setup.bash"
source "$COMPETITOR_WS/install/setup.bash"
PX4BIN="${PX4_DIR:-$SIM/PX4-Autopilot-v1.16.2}/build/px4_sitl_default/bin"
OUT="$RUN/flight-$MODE-$(date +%H%M%S)"; mkdir -p "$OUT"
log() { echo "[fly $(date +%T)] $*" | tee -a "$OUT/fly.log"; }
start() { setsid "${@:2}" > "$OUT/$1.log" 2>&1 < /dev/null & echo "$1 $!" >> "$RUN/pids"; }
px4() { (cd "$RUN/px4" && "$PX4BIN/px4-$1" --instance 0 "${@:2}") 2>&1 | tail -3 | tee -a "$OUT/fly.log"; }

start recorder python3 "$HERE/record_flight.py" --output "$OUT"
if [ "$MODE" = square ]; then
  start competitor ros2 launch competitor_example competitor.launch.py allow_mode_request:=true square_side_m:=5.0 ${SQUARE_ARGS:-}
  DONE_RE='square complete -- holding|not reached within'
else
  start competitor ros2 launch competitor_example competitor.launch.py allow_mode_request:=true competitor:=false
  start flyx python3 -u "$NODE" ${FLYX_ARGS:+--ros-args $FLYX_ARGS}
  DONE_RE='landing pad reached|holding here'
fi
arming() {   # ask PX4 directly (the DDS vehicle_status stream proved unreliable during landing)
  (cd "$RUN/px4" && timeout 5 "$PX4BIN/px4-listener" --instance 0 vehicle_status 2>/dev/null) | grep -m1 -oE "arming_state: [0-9]+" | grep -oE "[0-9]+$"; }
sleep "${ARM_DELAY_S:-15}"
# README step 7 arms first, then requests Offboard. In this sim that order is refused:
# with an FRD vision frame EKF2 never sets yaw_align ("heading estimate not stable"),
# which blocks arming in the boot mode (AUTO_LOITER); arming in Offboard is accepted. So
# the sim pilot requests Offboard (flight_interface is already streaming), then arms.
log "request offboard"; timeout 20 ros2 service call /skybeta/request_offboard std_srvs/srv/Trigger {} 2>&1 | tail -2 | tee -a "$OUT/fly.log"
sleep 2
for try in $(seq 1 "${ARM_TRIES:-20}"); do
  log "arming (pxh> commander arm), try $try"; px4 commander arm
  sleep 2; [ "$(arming)" = 2 ] && break
  timeout 20 ros2 service call /skybeta/request_offboard std_srvs/srv/Trigger {} > /dev/null 2>&1
  sleep 1
done
[ "$(arming)" = 2 ] || { log "PX4 refused to arm"; }
T0=$(date +%s)
while :; do
  if grep -qE "$DONE_RE" "$OUT"/competitor.log "$OUT"/flyx.log 2>/dev/null; then log "mission holding at the end: pilot lands"; break; fi
  if [ $(( $(date +%s) - T0 )) -gt "${MISSION_TIMEOUT_S:-400}" ]; then log "mission timeout: pilot lands"; break; fi
  if [ "$(arming)" != 2 ]; then log "not armed (arming_state $(arming))"; break; fi
  sleep 1
done
px4 commander land
for _ in $(seq 150); do [ "$(arming)" = 2 ] || break; sleep 2; done
log "disarmed (arming_state $(arming))"
log "done"; sleep 2
for n in flyx competitor recorder; do
  pid=$(awk -v n=$n '$1==n{p=$2} END{print p}' "$RUN/pids"); [ -n "$pid" ] && kill -INT -- "-$pid" 2>/dev/null
done
sleep 3
echo "$OUT"
