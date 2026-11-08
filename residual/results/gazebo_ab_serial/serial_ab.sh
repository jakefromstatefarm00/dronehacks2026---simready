#!/usr/bin/env bash
# Serial A/B runner: exactly ONE flight at a time (unlike queue_runner.sh, which keeps 3 going).
# usage: serial_ab.sh <plan file>   (plan lines: NNN program-path [args], same format as queue.txt)
# For each line: launch.sh, wait until native_program_exit_code appears (or processes vanish),
# wait for the slot's processes to tear down, post_flight, append to done.txt like queue_runner.
# A flight running > HANG_S wall seconds is killed by PID (only that slot's processes) and noted.
cd ~/dronehacks-2026-gb10
PLAN=${1:?plan file}; LOG=jolie_flyx/serial_ab.log; LOADLOG=jolie_flyx/serial_ab_load.log
HANG_S=${HANG_S:-1200}
echo "=== serial_ab start $(date '+%F %T') plan=$PLAN load=$(cut -d' ' -f1-3 /proc/loadavg)" >> $LOG
( while true; do echo "$(date +%T) $(cut -d' ' -f1-3 /proc/loadavg)" >> $LOADLOG; sleep 60; done ) &
LOADPID=$!
trap 'kill $LOADPID 2>/dev/null' EXIT
while read -r line; do
  [ -z "$line" ] && continue; case "$line" in \#*) continue;; esac
  n=${line%% *}
  echo "$(date +%T) load=$(cut -d' ' -f1-3 /proc/loadavg) launching v$n: $line" >> $LOG
  jolie_flyx/launch.sh $line || { echo "$(date +%T) launch FAILED v$n" >> $LOG; continue; }
  L=artifacts/gb10-sky-beta-course-v$n.launch.log; D=artifacts/gb10-sky-beta-course-v$n
  t0=$(date +%s); hung=0
  while true; do
    sleep 15
    grep -q native_program_exit_code $L 2>/dev/null && break
    pgrep -f "gb10-sky-beta-course-v$n" >/dev/null || break
    if [ $(( $(date +%s) - t0 )) -gt $HANG_S ]; then
      hung=1; pids=$(pgrep -f "gb10-sky-beta-course-v$n" | tr '\n' ' ')
      echo "$(date +%T) v$n HUNG > ${HANG_S}s, killing PIDs: $pids" >> $LOG
      kill $pids 2>/dev/null; sleep 10; kill -9 $(pgrep -f "gb10-sky-beta-course-v$n") 2>/dev/null; break
    fi
  done
  # let the slot tear down completely (gazebo/px4/qgc exit) before the next launch; cap 5 min
  for i in $(seq 1 60); do pgrep -f "gb10-sky-beta-course-v$n" >/dev/null || break; sleep 5; done
  left=$(pgrep -f "gb10-sky-beta-course-v$n" | tr '\n' ' ')
  [ -n "$left" ] && { echo "$(date +%T) v$n teardown incomplete, killing leftover PIDs: $left" >> $LOG; kill -9 $left 2>/dev/null; }
  sleep 3
  bash sim/runtime/post_flight.sh $n > $D/post.log 2>&1
  echo "v$n done $(date +%T): $(grep -o "\"native_program_exit_code\": *[-0-9a-z]*" $L) | $(grep RESULT $D/native-program.log 2>/dev/null)${hung:+ | HUNG-KILLED}" | sed 's/ | HUNG-KILLED$//; s/ | $//' >> jolie_flyx/done.txt
  [ $hung = 1 ] && echo "v$n HUNG-KILLED $(date +%T)" >> jolie_flyx/done.txt
  echo "$(date +%T) load=$(cut -d' ' -f1-3 /proc/loadavg) v$n finished wall=$(( $(date +%s) - t0 ))s hung=$hung" >> $LOG
done < $PLAN
echo "=== serial_ab DONE $(date '+%F %T')" >> $LOG
