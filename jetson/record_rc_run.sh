#!/usr/bin/env bash
# Record everything useful from a remote-control (RC) run of the course on the Sky Beta drone.
# Run on the Jetson HOST (plain SSH, not inside the container), with the stack up:
#
#   sudo -v                                                # optional: lets it log USB events too
#   bash ~/team_ws/jetson/record_rc_run.sh run1            # label is optional
#   bash ~/team_ws/jetson/record_rc_run.sh run2 --images   # + left/right camera, JPEG (heavy: ~3-6 GB/min)
#   bash ~/team_ws/jetson/record_rc_run.sh run3 --no-depth # without the depth image (~0.9 GB/min less)
#   ... fly the run on RC ...  then Ctrl-C here: the bag is closed, checked and summarised.
#
# WHAT: every ROS topic on the drone (ros2 bag record -a), EXCEPT the raw / compressed / NITROS camera
# image streams (hundreds of MB/s). Kept: the depth image the planner uses (/camera/depth/image_raw),
# all camera_info, IMU, cuVSLAM odometry / poses / status, /skybeta/state and setpoints, the program's
# status and path, every /fmu/out (PX4 estimator, attitude, RC sticks, battery, failsafe flags ...) and
# /fmu/in (what was sent to PX4), /tf, /tf_static, /rosout (all node logs). Plus tegrastats (CPU/GPU/
# RAM/temperature/power, 1 Hz), kernel USB events, notes, and a per-topic check at the end.
#
# SAFETY: the boot program (competitor window, set in /etc/skybeta/drone.env) flies autonomously when the pilot arms with the
# switch NOT on Position and then flips to Position on the ground. For RC runs this script switches the
# competitor window to that same program with shadow:=true (no setpoints: Offboard can never start) and restores the normal boot
# program on Ctrl-C. --keep-competitor leaves it alone (not advised).
#
# Output: ~/team_ws/runs/rc-<label>-<time>/  bag/ (split every 2 GB), notes.txt, check.txt,
#         tegrastats.log, usb.log, record.log   (root-owned: read with ls/cat/scp, delete with sudo rm -r)
# Copy to a laptop:  scp -r dronehacks@skybeta-06.local:~/team_ws/runs/rc-<label>-<time> .
#
# --local: test mode without the drone (no docker / tmux / lsusb; bag under $RUNS or ./runs).
set -u
LABEL="run"; IMAGES=0; DEPTH=1; KEEP=0; LOCAL=0
for a in "$@"; do
  case "$a" in
    --images) IMAGES=1 ;; --no-depth) DEPTH=0 ;; --keep-competitor) KEEP=1 ;; --local) LOCAL=1; KEEP=1 ;;
    -h|--help) sed -n '2,27p' "$0"; exit 0 ;;
    -*) echo "unknown option $a"; exit 2 ;;
    *) LABEL="$a" ;;
  esac
done
C=skybeta-stack
NAME="rc-$LABEL-$(date +%Y%m%d-%H%M%S)"
if [ "$LOCAL" -eq 1 ]; then
  RUNDIR="${RUNS:-$PWD/runs}/$NAME"; HOSTDIR="$RUNDIR"
  in_c() { bash -c "$*"; }
  c_cp() { cp "$1" "$2"; }
else
  RUNDIR=/root/team_ws/runs/$NAME; HOSTDIR=~/team_ws/runs/$NAME     # same folder (bind mount)
  in_c() { docker exec "$C" bash -ic "source /root/team_ws/install/setup.bash 2>/dev/null; $*" 2> >(grep -v "no job control\|cannot set terminal" >&2); }
  c_cp() { docker cp "$1" "$C:$2"; }
fi
TMP=$(mktemp -d)
say() { printf '[rc-record] %s\n' "$*"; }

# Topics that MUST have data in a good RC run (the end-of-run check flags them).
KEY=(/visual_slam/tracking/odometry /skybeta/state /oak/imu/data /fmu/out/vehicle_odometry
     /fmu/out/sensor_combined /fmu/out/vehicle_status_v1 /fmu/out/manual_control_setpoint /tf_static)
[ "$DEPTH" -eq 1 ] && KEY+=(/camera/depth/image_raw /camera/depth/camera_info)
[ "$IMAGES" -eq 1 ] && KEY+=(/oak/left/image_raw/compressed /oak/right/image_raw/compressed)

# Excluded (regex): camera image streams except the depth image the planner uses; NITROS / theora /
# compressedDepth variants; /oak/stereo/image_raw (same data as /camera/depth/image_raw); growing paths.
EXCL='^/oak/(left|right)/image_(raw|mono)$|.*/nitros$|.*/theora$|.*/compressedDepth$|^/oak/stereo/image_raw.*|.*_path$|^/parameter_events$'
[ "$IMAGES" -eq 0 ] && EXCL="$EXCL|^/oak/(left|right)/image_(raw|mono)/compressed$"
[ "$DEPTH" -eq 0 ] && EXCL="$EXCL|^/camera/depth/image_raw$"

# ---------------------------------------------------------------- preflight
if [ "$LOCAL" -eq 0 ]; then
  docker ps --format '{{.Names}}' | grep -qx "$C" || { say "container $C is not running (sudo systemctl start skybeta-stack)"; exit 1; }
  cam=$(lsusb | grep 03e7)
  case "$cam" in
    *f63b*) say "camera OK: $cam" ;;
    *) say "WARNING camera not booted (${cam:-not on USB}): no depth / VIO. Fix: sudo systemctl restart skybeta-stack" ;;
  esac
else
  cam="(local test)"
fi
in_c "mkdir -p $RUNDIR"
free_gb=$(df -BG --output=avail "$(dirname "$HOSTDIR")" 2>/dev/null | tail -1 | tr -dc 0-9)
say "disk free: ${free_gb:-?} GB (about 1 GB per minute with depth; 4-7 GB per minute with --images)"
[ "${free_gb:-99}" -lt 5 ] && say "WARNING less than 5 GB free"
state=$(in_c "timeout 6 ros2 topic echo --once --full-length /skybeta/state 2>/dev/null" | head -1)
case "$state" in
  *'"px4_link": true'*) say "PX4 link up" ;;
  *) say "WARNING /skybeta/state missing or px4_link false: is the flight controller powered?" ;;
esac
case "$state" in *'"armed": true'*) say "REFUSE: the vehicle is ARMED. Disarm, then start the recording."; exit 1 ;; esac

# ---------------------------------------------------------------- competitor -> shadow
restore() { :; }
if [ "$KEEP" -eq 0 ]; then
  PKG=$(sed -n 's/^COMPETITOR_PKG=//p' /etc/skybeta/drone.env 2>/dev/null | tail -1)
  LAU=$(sed -n 's/^COMPETITOR_LAUNCH=//p' /etc/skybeta/drone.env 2>/dev/null | tail -1)
  [ -n "$PKG" ] && [ -n "$LAU" ] || { say "REFUSE: COMPETITOR_PKG / COMPETITOR_LAUNCH not found in /etc/skybeta/drone.env"; exit 1; }
  say "switching the competitor window to $PKG $LAU in SHADOW (shadow:=true, no setpoints) for the RC run"
  docker exec "$C" tmux respawn-pane -k -t skybeta:competitor \
    "bash -c 'source skybeta_env.sh; source /root/team_ws/install/setup.bash; exec ros2 launch $PKG $LAU shadow:=true'"
  restore() {
    say "restoring the competitor window (normal boot program)"
    docker exec "$C" tmux respawn-pane -k -t skybeta:competitor "exec run_competitor.sh"
  }
  for i in $(seq 1 30); do
    sleep 2
    docker exec "$C" tmux capture-pane -p -J -t skybeta:competitor -S -40 | grep -qi "shadow" && break
  done
  if docker exec "$C" tmux capture-pane -p -J -t skybeta:competitor -S -40 | grep -qi "shadow"; then
    say "competitor is in SHADOW (no autonomy)"
  else
    say "WARNING could not confirm SHADOW in the competitor window: check it before arming (tmux attach -t skybeta)"
  fi
  sleep 3
fi
# a shadow node still REGISTERS a publisher; what matters is whether setpoints actually flow
sp=$(in_c "timeout 5 ros2 topic echo --once /skybeta/setpoint/pose 2>/dev/null" | grep -c "position")
if [ "$sp" -gt 0 ]; then
  say "WARNING setpoints ARE being sent on /skybeta/setpoint/pose: an arm -> Position switch could start Offboard"
else
  say "no setpoints flowing on /skybeta/setpoint/pose (arm -> Position cannot start Offboard)"
fi

# ---------------------------------------------------------------- notes + side logs
{
  echo "run: $NAME"; echo "start: $(date -Is)"
  echo "options: images=$IMAGES depth=$DEPTH keep_competitor=$KEEP local=$LOCAL"
  echo "host: $(hostname)  uptime: $(uptime)"
  echo "camera: $cam"
  grep -E "^(TEAM|COMPETITOR_PKG|COMPETITOR_LAUNCH|ROS_DOMAIN_ID|CAMERA_)" /etc/skybeta/drone.env 2>/dev/null
  echo "v4c sha256: $(sha256sum ~/team_ws/src/jolie_best/scripts/upload_to_flyx_v4c_skybeta.py 2>/dev/null | cut -c1-16)"
  echo "state at start: $state"
  echo "record: ros2 bag record -a --exclude '$EXCL'"
  echo "topics at start:"; in_c "timeout 10 ros2 topic list -t 2>/dev/null" | sed 's/^/  /'
} > "$TMP/notes.txt"
c_cp "$TMP/notes.txt" "$RUNDIR/notes.txt"
TG=; DM=
command -v tegrastats >/dev/null && { tegrastats --interval 1000 > "$TMP/tegrastats.log" 2>&1 & TG=$!; }
if [ "$LOCAL" -eq 0 ] && sudo -n true 2>/dev/null; then
  ( sudo -n dmesg -w -T 2>/dev/null | grep --line-buffered -iE "usb|xlink|03e7" > "$TMP/usb.log" ) & DM=$!
else
  say "(USB events not logged: run 'sudo -v' first to include them)"
fi

# ---------------------------------------------------------------- record
XOPT=--exclude                                         # Humble (the drone)
in_c "ros2 bag record --help 2>/dev/null" | grep -q -- "--exclude-regex" && XOPT=--exclude-regex   # Iron / Jazzy
say "RECORDING to $HOSTDIR/bag  --  fly the run on RC, then press Ctrl-C here"
# job control on while starting it: otherwise a background job of a script ignores SIGINT, and the
# recorder could not be stopped cleanly (no metadata.yaml = a bag that "doesn't really work")
set -m
in_c "cd $RUNDIR && exec ros2 bag record -a $XOPT '$EXCL' --max-bag-size 2000000000 -o bag" > "$TMP/record.log" 2>&1 &
REC=$!
set +m
T0=$(date +%s)
finish() {
  trap - INT TERM
  say "stopping the recorder ($(( $(date +%s) - T0 )) s recorded)..."
  stop_rec() {   # $1 = signal; the recorder writes metadata.yaml when it shuts down cleanly
    if [ "$LOCAL" -eq 1 ]; then pkill "-$1" -f "ros2 bag record -a" 2>/dev/null
    else docker exec "$C" pkill "-$1" -f "ros2 bag record -a" 2>/dev/null; fi
  }
  stop_rec INT
  for i in $(seq 1 15); do kill -0 $REC 2>/dev/null || break; sleep 1; done
  if kill -0 $REC 2>/dev/null; then say "recorder still running after SIGINT: sending SIGTERM"; stop_rec TERM; fi
  for i in $(seq 1 30); do kill -0 $REC 2>/dev/null || break; sleep 1; done
  kill -0 $REC 2>/dev/null && say "WARNING recorder did not exit: the bag may lack metadata.yaml (ros2 bag reindex can repair it)"
  [ -n "$TG" ] && kill $TG 2>/dev/null
  [ -n "$DM" ] && { sudo -n pkill -f "dmesg -w -T" 2>/dev/null; kill $DM 2>/dev/null; }
  for f in tegrastats usb record; do [ -f "$TMP/$f.log" ] && c_cp "$TMP/$f.log" "$RUNDIR/$f.log"; done
  restore
  # ------------------------------------------------------------ check
  in_c "ros2 bag info $RUNDIR/bag" > "$TMP/info" 2>&1
  {
    echo "== ros2 bag info"; cat "$TMP/info"
    echo; echo "== key topics (count; 0 = MISSING)"
    for t in "${KEY[@]}"; do
      n=$(grep -E "Topic: $t \|" "$TMP/info" | grep -oE "Count: [0-9]+" | tr -dc 0-9)
      printf '  %-9s %s\n' "${n:-0}" "$t"
    done
    echo; echo "== usb events during the run"; tail -20 "$TMP/usb.log" 2>/dev/null
  } > "$TMP/check.txt"
  c_cp "$TMP/check.txt" "$RUNDIR/check.txt"
  say "$(grep -E '^Duration' "$TMP/info")   $(grep -E '^Bag size' "$TMP/info")   $(grep -E '^Messages' "$TMP/info")"
  say "topics recorded: $(grep -c 'Topic: ' "$TMP/info")"
  missing=$(sed -n '/key topics/,/usb events/p' "$TMP/check.txt" | awk '$1=="0"{print $2}' | tr '\n' ' ')
  if [ -n "$missing" ]; then say "MISSING key topics (0 messages): $missing"; else say "all key topics have data"; fi
  grep -qi "disconnect" "$TMP/usb.log" 2>/dev/null && say "WARNING USB disconnects during the run: see usb.log"
  grep -qiE "error|failed" "$TMP/record.log" 2>/dev/null && say "recorder reported errors: see record.log"
  say "files: $HOSTDIR"
  rm -rf "$TMP"
  exit 0
}
trap finish INT TERM
wait $REC
say "the recorder exited on its own (see record.log)"
finish
