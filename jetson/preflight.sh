#!/usr/bin/env bash
# preflight.sh -- go / no-go for an autonomous arm -> Position start on the Sky Beta drone.
# Run on the Jetson HOST (ssh), drone on the launch mark, Pixhawk powered, DISARMED, props as you like.
# Read-only: changes nothing. Every line is PASS / FAIL / WARN; the last line is GO or NO-GO.
#
#   bash ~/team_ws/jetson/preflight.sh            # ~45 s (includes a 15 s latency measurement)
#   bash ~/team_ws/jetson/preflight.sh --quick    # skip the latency measurement
#
# What it checks (each one stopped a takeoff on 2026-11-08, see DEBUG_TAKEOFF.md):
#   1 camera on USB3 (03e7:f63b on Bus 002, 5000 Mbps)        the OAK fell to USB2 / unbooted at arming
#   2 camera images + cuVSLAM tracking (vo_state 1, ~30 Hz)   driver crashed after the USB drop
#   3 vision -> PX4 latency (organisers' probe)               290 ms is dropped by EKF2 with EV_DELAY 70
#   4 EKF2 fusing vision (cs_ev_pos / cs_ev_yaw true)          the real gate for Position mode
#   5 PX4 has a local position (xy_valid, /skybeta/state local_enu non-null)
#   6 boot program is LIVE: publishes /skybeta/setpoint/pose, not shadow
#   7 disarmed + landed, PX4 link up
set -u
QUICK=0; [ "${1:-}" = "--quick" ] && QUICK=1
C=skybeta-stack; GO=1
pass() { echo "PASS  $*"; }; fail() { echo "FAIL  $*"; GO=0; }; warn() { echo "WARN  $*"; }
in_c() { docker exec "$C" bash -ic "$*" 2>/dev/null; }
once() { in_c "timeout -s KILL 8 ros2 topic echo --once $1" | tr -d '\r'; }

docker ps --format '{{.Names}}' | grep -qx "$C" || { fail "stack container $C not running"; echo NO-GO; exit 1; }

# 1 camera USB
cam=$(lsusb | grep -E '03e7:(f63b|2485)')
case "$cam" in
  *"Bus 002"*f63b*) spd=$(cat /sys/bus/usb/devices/2-1.1/speed 2>/dev/null); [ "${spd:-0}" = "5000" ] && pass "camera booted on USB3: $cam" || warn "camera on Bus 002 but speed=$spd (expected 5000)";;
  *2485*)            fail "camera UNBOOTED / fell back to USB2 ($cam): reseat the camera USB-C, restart the stack";;
  "")                fail "no OAK camera on USB";;
  *)                 fail "camera on the wrong bus ($cam): expected Bus 002";;
esac
dmesg_err=$(sudo -n dmesg 2>/dev/null | grep -cE 'usb 2-1\.1: (USB disconnect|reset SuperSpeed)')
[ "${dmesg_err:-0}" -gt 0 ] && warn "$dmesg_err camera USB reset/disconnect event(s) since boot (dmesg): check the cable"

# 2 images + cuVSLAM
hz() { in_c "timeout -s KILL 6 ros2 topic hz $1" 2>&1 | grep -oE 'rate: [0-9.]+' | tail -1 | cut -d' ' -f2; }
ir=$(hz /oak/left/camera_info); or=$(hz /visual_slam/tracking/odometry); br=$(hz /fmu/in/vehicle_visual_odometry)
[ -n "$ir" ] && pass "camera publishing (${ir} Hz)" || fail "no camera frames on /oak/left/camera_info (driver dead? restart the stack)"
vo=$(once /visual_slam/status | grep -E '^vo_state' | grep -oE '[0-9]+')
[ "${vo:-x}" = "1" ] && pass "cuVSLAM tracking (vo_state 1, odometry ${or:-0} Hz)" || fail "cuVSLAM not tracking (vo_state ${vo:-none}, odometry ${or:-0} Hz)"
[ -n "$br" ] && pass "bridge -> PX4 vision odometry ${br} Hz" || fail "nothing on /fmu/in/vehicle_visual_odometry"

# 3 latency
if [ "$QUICK" -eq 0 ]; then
  docker cp ~/skybeta/tools/latency_probe.py "$C":/tmp/latency_probe.py 2>/dev/null
  probe=$(docker exec "$C" bash -lc 'source skybeta_env.sh; timeout -s KILL 40 python3 /tmp/latency_probe.py --seconds 15' 2>/dev/null)
  drv=$(echo "$probe" | grep -A2 '^driver leg' | grep -oE 'p50=[0-9.]+' | grep -oE '[0-9]+' | head -1)
  rec=$(echo "$probe" | grep -oE 'Recommended EKF2_EV_DELAY = [0-9]+' | grep -oE '[0-9]+$')
  p95=$(echo "$probe" | grep -A2 '^full chain' | grep -oE 'p95=[0-9.]+' | grep -oE '[0-9]+' | head -1)
  if [ -n "$rec" ]; then
    # clean boot: driver leg ~36 ms, full chain p50 ~72 ms (organisers' bench). bad boot: a constant 200-290 ms
    # (camera queue backed up at startup) that never drains -> EKF2 (EV_DELAY 70 / DELAY_MAX 200) drops it.
    if [ "${drv:-999}" -lt 150 ] && [ "$rec" -lt 150 ]; then pass "vision latency clean boot: driver leg ${drv} ms, full chain p50 ${rec} ms (p95 ${p95:-?} ms)"
    elif [ "$rec" -lt 235 ]; then warn "vision latency high (driver ${drv} ms, full chain p50 ${rec} ms): on EKF2's edge; a stack restart usually gives a clean boot"
    else fail "BAD BOOT: vision ${rec} ms old (driver leg ${drv} ms): EKF2 will drop it. sudo systemctl restart skybeta-stack, wait 90 s, rerun (repeat until driver leg < 150 ms)"; fi
    [ -n "$p95" ] && [ "$p95" -ge 300 ] && warn "latency spikes p95 ${p95} ms: EKF2_DELAY_MAX 400 (jetson/px4/skybeta06_ekf2_vision_delay.params) keeps them fused"
  else warn "latency probe gave no result"; fi
fi

# 4-5 EKF2 / PX4
st=$(once /skybeta/state | head -1)
case "$st" in *'"px4_link": true'*) pass "PX4 link up";; *) fail "PX4 link down (Pixhawk powered? TELEM2 seated?)";; esac
fl=$(once /fmu/out/estimator_status_flags)
evp=$(echo "$fl" | grep -E '^cs_ev_pos' | grep -oE 'true|false'); evy=$(echo "$fl" | grep -E '^cs_ev_yaw' | grep -oE 'true|false')
if [ "$evp" = "true" ] && [ "$evy" = "true" ]; then pass "EKF2 fusing vision (cs_ev_pos, cs_ev_yaw)"
elif [ -z "$evp" ]; then fail "no estimator_status_flags from PX4"
else fail "EKF2 NOT fusing vision (cs_ev_pos=$evp cs_ev_yaw=$evy): vision samples too old for EKF2_EV_DELAY/DELAY_MAX, or EKF2_EV_CTRL not 9"; fi
lp=$(once /fmu/out/vehicle_local_position)
xyv=$(echo "$lp" | grep -E '^xy_valid' | grep -oE 'true|false'); dr=$(echo "$lp" | grep -E '^dead_reckoning' | grep -oE 'true|false')
[ "$xyv" = "true" ] && pass "PX4 local position valid (xy_valid true, dead_reckoning $dr)" || fail "PX4 local position INVALID (xy_valid=$xyv): Position mode will fall back to Altitude and disarm"
case "$st" in *'"local_enu": null'*) fail "/skybeta/state local_enu is null (flight_interface sees no PX4 position)";; *'"local_enu": ['*) pass "/skybeta/state local_enu present";; esac
nav=$(echo "$st" | grep -oE '"nav_state_name": "[A-Z_0-9-]+"' | cut -d'"' -f4)
case "$nav" in POSCTL) pass "PX4 in POSCTL (switch on Position, disarmed): Position accepted";; ALTCTL) warn "PX4 in ALTCTL: if the switch is on Position, PX4 refused it (no position)";; *) warn "PX4 nav_state $nav (put the switch on Position while disarmed to test acceptance)";; esac

# 6 boot program live
pk=$(sed -n 's/^COMPETITOR_PKG=//p' /etc/skybeta/drone.env | tail -1); la=$(sed -n 's/^COMPETITOR_LAUNCH=//p' /etc/skybeta/drone.env | tail -1)
pubs=$(in_c "timeout -s KILL 6 ros2 topic info /skybeta/setpoint/pose" | grep -oE 'Publisher count: [0-9]+' | grep -oE '[0-9]+$')
prog=$(docker exec "$C" bash -c "ps -eo args | grep -oE 'upload_to_flyx[^ ]*\.py( --shadow)?' | head -1")
echo "INFO  boot program: $pk $la  running: ${prog:-none}"
case "$prog" in *--shadow*) fail "program is in SHADOW: it will never fly (restart the competitor without shadow)";; esac
if [ "$pk" = "path_recorder" ]; then warn "boot program is path_recorder: RC recording only, no autonomy (set jolie_best in drone.env for autonomy)"
elif [ "${pubs:-0}" -ge 1 ]; then pass "a program publishes /skybeta/setpoint/pose ($pubs publisher)"
else fail "no publisher on /skybeta/setpoint/pose: the flight interface will refuse Offboard"; fi
jst=$(once /jolie_flyx/status | head -1 | grep -oE '"state": "[A-Z_]+"' | cut -d'"' -f4)
[ -n "$jst" ] && echo "INFO  v4c state: $jst  (WAIT_HEALTH until PX4 position is valid; STREAM/WAIT_OFFBOARD = ready for arm -> Position)"

# 7 armed state
case "$st" in *'"armed": true'*) fail "vehicle is ARMED";; *'"armed": false'*) pass "disarmed";; esac

echo "----------------------------------------------------------------"
if [ "$GO" -eq 1 ]; then echo "GO: arm with the switch OFF Position, then flip to Position on the ground. Pilot on the kill switch."
else echo "NO-GO: fix the FAIL lines above first."; fi
[ "$GO" -eq 1 ]
