#!/usr/bin/env bash
# Local end-to-end test of record_rc_run.sh (no drone): fake drone topics + --local mode + a real SIGINT.
# Needs ROS 2 (Humble or Jazzy). Prints the per-topic check; /fmu/out topics show 0 (no PX4 in the test).
source /opt/ros/${ROS_DISTRO:-jazzy}/setup.bash
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"; D=$(mktemp -d)
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-91}
python3 "$HERE/fake_drone.py" & FAKE=$!; sleep 4
RUNS=$D/runs python3 -c "import signal,os,sys; signal.signal(signal.SIGINT, signal.SIG_DFL); os.execvp('bash',['bash',sys.argv[1],'localtest','--local'])" \
  "$HERE/../record_rc_run.sh" > $D/rec.out 2>&1 & R=$!
for i in $(seq 1 60); do grep -q RECORDING $D/rec.out && break; sleep 1; done
sleep 15; kill -INT $R; wait $R; kill $FAKE
cat $D/rec.out; sed -n '/key topics/,$p' $D/runs/*/check.txt
grep -q "Topic: /oak/left/image_raw " $D/runs/*/check.txt && echo "FAIL: raw image was recorded" || echo "ok: raw image excluded"
