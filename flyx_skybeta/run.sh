#!/usr/bin/env bash
# One command in the FlyX lab (or any sim): the drone's own control path, end to end.
#   1. the organizer's skybeta_flight_interface, UNMODIFIED, from skybeta/src, with the drone's
#      settings (auto_offboard_on_arm: arming then switching to Position starts Offboard) plus the
#      SITL-only request_offboard service for pilot mode "direct"
#   2. the program, on the Sky Beta contract (the same file that flies the drone)
#   3. team_mission's sim_pilot (SIM ONLY; refuses to run on the drone): arms and hands over the
#      way the safety pilot does, and lands when the program reports HOLD / LAND
#
#   bash flyx_skybeta/run.sh suraj   [program args]   # suraj_course/suraj_course.py
#   bash flyx_skybeta/run.sh jolie   [program args]   # jolie_course/jolie_course.py (+ FlyX depth camera)
#   bash flyx_skybeta/run.sh jolie_flyx [args]        # jolie_course/upload_to_flyx_skybeta.py (FlyX-proven file, Sky Beta I/O)
#   bash flyx_skybeta/run.sh team       [args]        # team_ws/src/pathfinding (dronehacks_pathplan) from source, no colcon
#   PILOT=direct bash flyx_skybeta/run.sh ...         # if PX4 refuses the Altitude/Position handshake
#
# team_mission (Jake's) has its own launcher: ros2 launch team_mission sim.launch.py (see its README).
# Reset world first; vehicle on the launch mark facing up the first corridor. Ctrl-C: the pilot lands.
# Logs: flyx_skybeta/runs/<program>-<time>/ (flight_interface.log, program.log, pilot.log).
set -u
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
WHAT="${1:-suraj}"; shift || true
STAMP="$(date +%Y%m%d-%H%M%S)"
OUT="$ROOT/flyx_skybeta/runs/$WHAT-$STAMP"
mkdir -p "$OUT" 2>/dev/null || { OUT="$HOME/flyx-runs/skybeta-$WHAT-$STAMP"; mkdir -p "$OUT"; }
case "$WHAT" in
  suraj) PROG=(python3 -u "$ROOT/suraj_course/suraj_course.py" "$@"); STATUS=/suraj_course/status ;;
  jolie) PROG=(python3 -u "$ROOT/jolie_course/jolie_course.py" --ros-args
               -p depth_topic:=/camera/depth/image_raw "$@")
         STATUS=/dronehacks_pathplan/status ;;
  jolie_flyx) PROG=(python3 -u "$ROOT/jolie_course/upload_to_flyx_skybeta.py" "$@")   # Jolie's FlyX-proven file, Sky Beta I/O
         STATUS=/jolie_flyx/status ;;
  team)  # team_ws/src/pathfinding (dronehacks_pathplan) from source, no colcon build needed. Same
         # parameters as its sim.launch.py; with colcon, prefer: ros2 launch dronehacks_pathplan sim.launch.py
         TEAM_SRC="$ROOT/team_ws/src/pathfinding"
         SIMT="${SIM_TIME:-true}"                                  # FlyX publishes /clock: run limit in sim time
         PROG=(env PYTHONPATH="$TEAM_SRC${PYTHONPATH:+:$PYTHONPATH}" python3 -u -c
               "from dronehacks_pathplan.node import main; main()" --ros-args
               -p use_sim_time:="$SIMT" -p depth_topic:=/camera/depth/image_raw -p journal_dir:="$OUT" "$@")
         # flight_interface on the WALL clock: its OffboardControlMode stamps are what PX4 checks for
         # Offboard loss. PX4 v1.16 uxrce_dds_client: stamp -> min(stamp - timesync offset, PX4 now), so
         # a wall-clock stamp is clamped to PX4 time (UXRCE_DDS_SYNCT=0, lockstep) or converted exactly
         # (SYNCT=1); a sim-time stamp is compared raw with PX4's clock and read as stale (FlyX run 5).
         FI_EXTRA=(-p use_sim_time:=false)
         # Jake's sim pilot (lands on Ctrl-C / SIGTERM); land when the node holds low or gives up
         PILOT_EXTRA=(-p land_states:=LOW_HOLD,FAULT,TAKEN_OVER)
         STATUS=/dronehacks_pathplan/status ;;
  *) echo "usage: run.sh suraj|jolie|jolie_flyx|team [args]"; exit 2 ;;
esac
FI_SRC="$ROOT/skybeta/src/skybeta_flight_interface"
PILOT_SRC="$ROOT/team_ws/src/team_mission"
for d in "$FI_SRC/skybeta_flight_interface" "$PILOT_SRC/team_mission/sim"; do
  [ -d "$d" ] || { echo "missing $d (use the whole repo)"; exit 1; }
done
python3 -c "import px4_msgs, geometry_msgs, nav_msgs, std_srvs" || { echo "ROS 2 / px4_msgs not sourced"; exit 1; }
echo "logs: $OUT"

pids=()
cleanup() {
  trap - INT TERM EXIT
  [ -n "${PILOT_PID:-}" ] && kill -INT "$PILOT_PID" 2>/dev/null && wait "$PILOT_PID" 2>/dev/null   # pilot lands first
  for p in "${pids[@]}"; do kill -INT "$p" 2>/dev/null; done
  sleep 2; for p in "${pids[@]}"; do kill -KILL "$p" 2>/dev/null; done
  echo "stopped; logs: $OUT"
}
trap cleanup INT TERM EXIT

PYTHONPATH="$FI_SRC${PYTHONPATH:+:$PYTHONPATH}" python3 -u -c \
  "from skybeta_flight_interface.flight_interface import main; main()" --ros-args \
  -p auto_offboard_on_arm:=true -p allow_mode_request:=true ${FI_EXTRA[@]+"${FI_EXTRA[@]}"} \
  > "$OUT/flight_interface.log" 2>&1 &
pids+=($!)
sleep 3
kill -0 "${pids[0]}" 2>/dev/null || { echo "flight interface died:"; tail -20 "$OUT/flight_interface.log"; exit 1; }
echo "flight interface up (pid ${pids[0]})"

(cd "$OUT" && exec "${PROG[@]}") > "$OUT/program.log" 2>&1 &
pids+=($!)
tail -n +1 -f "$OUT/program.log" --pid="${pids[1]}" &
pids+=($!)

PYTHONPATH="$PILOT_SRC:$FI_SRC${PYTHONPATH:+:$PYTHONPATH}" python3 -u -c \
  "from team_mission.sim.sim_pilot import main; main()" --ros-args \
  -p mode:="${PILOT:-handshake}" -p status_topic:="$STATUS" ${PILOT_EXTRA[@]+"${PILOT_EXTRA[@]}"} \
  > >(tee "$OUT/pilot.log") 2>&1 &
PILOT_PID=$!
# the program died: stop the pilot too (it lands if armed), so this script never waits forever
( while kill -0 "${pids[1]}" 2>/dev/null; do sleep 1; done
  kill -0 "$PILOT_PID" 2>/dev/null && { echo "program exited: stopping the pilot"; kill -TERM "$PILOT_PID"; } ) &
pids+=($!)
wait "$PILOT_PID"; PILOT_PID=
sleep 3
