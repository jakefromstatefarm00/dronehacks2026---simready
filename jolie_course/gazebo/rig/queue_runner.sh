#!/usr/bin/env bash
# Keeps <=3 flights going from jolie_flyx/queue.txt (lines: NNN program-path [args]); post-flights each.
cd ~/dronehacks-2026-gb10
Q=jolie_flyx/queue.txt; ACTIVE=jolie_flyx/active.txt; touch $Q $ACTIVE
while true; do
  for n in $(cat $ACTIVE); do
    L=artifacts/gb10-sky-beta-course-v$n.launch.log
    if grep -q native_program_exit_code $L 2>/dev/null || ! pgrep -f "gb10-sky-beta-course-v$n" >/dev/null; then
      sleep 3; bash sim/runtime/post_flight.sh $n > artifacts/gb10-sky-beta-course-v$n/post.log 2>&1
      echo "v$n done $(date +%T): $(grep -o "\"native_program_exit_code\": *[-0-9a-z]*" $L) | $(grep RESULT artifacts/gb10-sky-beta-course-v$n/native-program.log)" >> jolie_flyx/done.txt
      sed -i "/^$n\$/d" $ACTIVE
    fi
  done
  if [ $(wc -l < $ACTIVE) -lt 3 ] && [ -s $Q ]; then
    line=$(head -1 $Q); sed -i 1d $Q
    n=${line%% *}; jolie_flyx/launch.sh $line && echo $n >> $ACTIVE
    sleep 20; continue
  fi
  [ -f jolie_flyx/STOP ] && break
  sleep 10
done
