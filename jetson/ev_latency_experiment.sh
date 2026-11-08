#!/usr/bin/env bash
# Runs ON THE JETSON HOST. Drone must be DISARMED on the ground.
# A: organizers' latency probe with the stack as-is (competitor = hard v4c, LIVE).
# B: same probe with the competitor window idle (no team nodes at all).
# Then restores the competitor (boot program) and confirms the hard script is running.
set -u
C=skybeta-stack
S=$(docker exec $C bash -ic "timeout -s KILL 8 ros2 topic echo --once /skybeta/state" 2>/dev/null | head -1)
case "$S" in *'"armed": true'*) echo "REFUSE: armed"; exit 1;; esac
echo "state: $(echo "$S" | cut -c1-160)"
docker cp ~/skybeta/tools/latency_probe.py $C:/tmp/latency_probe.py
echo "== subscribers of the camera images"
docker exec $C bash -ic "timeout -s KILL 10 ros2 topic info -v /oak/left/image_raw 2>/dev/null | grep -E 'Node name|Subscription count'; timeout -s KILL 10 ros2 topic info -v /oak/stereo/image_raw 2>/dev/null | grep -E 'Node name|Subscription count'" 2>/dev/null
echo "== competitor processes"
docker exec $C bash -c "ps -eo pid,pcpu,comm,args | grep -E 'upload_to_flyx|depth_relay|flight_interface' | grep -v grep | cut -c1-140"
echo "== A: probe, stack as-is (20 s)"
docker exec $C bash -lc 'source skybeta_env.sh; timeout -s KILL 40 python3 /tmp/latency_probe.py --seconds 20' 2>/dev/null | tail -25
echo "== competitor -> idle"
docker exec $C tmux respawn-pane -k -t skybeta:competitor "exec sleep infinity"
sleep 10
docker exec $C bash -ic "timeout -s KILL 6 ros2 topic info /skybeta/setpoint/pose 2>/dev/null | grep Publisher" 2>/dev/null
echo "== B: probe, competitor idle (20 s)"
docker exec $C bash -lc 'source skybeta_env.sh; timeout -s KILL 40 python3 /tmp/latency_probe.py --seconds 20' 2>/dev/null | tail -25
echo "== EKF2 flags while idle"
docker exec $C bash -ic "timeout -s KILL 8 ros2 topic echo --once /fmu/out/estimator_status_flags 2>/dev/null | grep -E '^cs_(ev_pos|ev_yaw|ev_hgt)'; timeout -s KILL 8 ros2 topic echo --once /fmu/out/vehicle_local_position 2>/dev/null | grep -E '^(xy_valid|z_valid|dead_reckoning):'" 2>/dev/null
echo "== restoring the competitor (boot program)"
docker exec $C tmux respawn-pane -k -t skybeta:competitor "exec run_competitor.sh"
sleep 40
docker exec $C bash -c "ps -eo args | grep -oE 'python3 -u [^ ]*upload_to_flyx[^ ]*' | head -1"
docker exec $C bash -ic "timeout -s KILL 8 ros2 topic echo --once /jolie_flyx/status 2>/dev/null | head -1 | cut -c1-120" 2>/dev/null
echo DONE
