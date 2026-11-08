#!/usr/bin/env bash
# Competitor window: organizer flight_interface + team depth relay + Jolie's v4c (Sky Beta copy).
# SHADOW by default (plans + logs, publishes no setpoints). For a real flight: MODE=fly bash run_jolie.sh
source /opt/ros/humble/setup.bash; source /root/team_ws/install/setup.bash
T=$(date +%Y%m%d-%H%M%S); OUT=/root/team_ws/runs/jolie-v4c-$T; mkdir -p $OUT
ros2 launch dronehacks_pathplan pathplan.launch.py competitor:=false > $OUT/interface_relay.log 2>&1 &
IF=$!; trap 'kill -INT $IF 2>/dev/null; wait $IF 2>/dev/null' EXIT
sleep 4
ARGS="--camera-offset 0.1053 0 0.0304 --depth-topic /camera/depth/image_raw --output $OUT/run"
[ "${MODE:-shadow}" = fly ] || ARGS="--shadow $ARGS"
echo "jolie v4c: ${MODE:-shadow}; logs $OUT"
python3 -u /root/team_ws/jolie/upload_to_flyx_v4c_skybeta.py $ARGS 2>&1 | tee $OUT/program.log
