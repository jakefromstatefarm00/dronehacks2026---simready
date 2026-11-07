#!/usr/bin/env bash
# One command in the FlyX lab (or any sim): the drone's own control path, end to end.
#   1. the organizer's skybeta_flight_interface, UNMODIFIED, from skybeta/src (owns /fmu/in,
#      exactly as on the drone; allow_mode_request:=true is the SITL-only Offboard service)
#   2. the program, on the Sky Beta contract (the same file that flies the drone)
#   3. flyx_skybeta/sim_pilot.py: SIM ONLY, does the safety pilot's arm / land
#
#   bash flyx_skybeta/run.sh course       [program args]   # flyx_course/fly_course.py (friend's route)
#   bash flyx_skybeta/run.sh pathfinder   [program args]   # pathfinding/flyx/pathplan_mission.py
#
# Reset world first; vehicle on the launch mark facing up the first corridor. Ctrl-C lands.
# Logs: flyx_skybeta/runs/<program>-<time>/ (flight_interface.log, program.log, pilot.log).
set -u
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
WHAT="${1:-course}"; shift || true
case "$WHAT" in
  course)     PROG=(python3 -u "$ROOT/flyx_course/fly_course.py" "$@") ;;
  pathfinder) PROG=(python3 -u "$ROOT/pathfinding/flyx/pathplan_mission.py" --ros-args
                    -p depth_topic:=/camera/depth/image_raw "$@") ;;
  *) echo "usage: run.sh course|pathfinder [args]"; exit 2 ;;
esac
FI_SRC="$ROOT/skybeta/src/skybeta_flight_interface"
[ -d "$FI_SRC/skybeta_flight_interface" ] || { echo "missing $FI_SRC (pull the repo's skybeta/ folder)"; exit 1; }
python3 -c "import px4_msgs, geometry_msgs, nav_msgs, std_srvs" || { echo "ROS 2 / px4_msgs not sourced"; exit 1; }
STAMP="$(date +%Y%m%d-%H%M%S)"
OUT="$ROOT/flyx_skybeta/runs/$WHAT-$STAMP"
mkdir -p "$OUT" 2>/dev/null || { OUT="$HOME/flyx-runs/skybeta-$WHAT-$STAMP"; mkdir -p "$OUT"; }
echo "logs: $OUT"

pids=()
cleanup() {
  trap - INT TERM EXIT
  [ -n "${PILOT:-}" ] && kill -INT "$PILOT" 2>/dev/null && wait "$PILOT" 2>/dev/null   # pilot lands first
  for p in "${pids[@]}"; do kill -INT "$p" 2>/dev/null; done
  sleep 2; for p in "${pids[@]}"; do kill -KILL "$p" 2>/dev/null; done
  echo "stopped; logs: $OUT"
}
trap cleanup INT TERM EXIT

PYTHONPATH="$FI_SRC${PYTHONPATH:+:$PYTHONPATH}" python3 -u -c \
  "from skybeta_flight_interface.flight_interface import main; main()" --ros-args -p allow_mode_request:=true > "$OUT/flight_interface.log" 2>&1 &
pids+=($!)
sleep 3
kill -0 "${pids[0]}" 2>/dev/null || { echo "flight interface died:"; tail -20 "$OUT/flight_interface.log"; exit 1; }
echo "flight interface up (pid ${pids[0]})"

(cd "$OUT" && exec "${PROG[@]}") > "$OUT/program.log" 2>&1 &
pids+=($!)
tail -n +1 -f "$OUT/program.log" --pid="${pids[1]}" &
pids+=($!)

python3 -u "$ROOT/flyx_skybeta/sim_pilot.py" > >(tee "$OUT/pilot.log") 2>&1 &
PILOT=$!
wait "$PILOT"; PILOT=
sleep 3
