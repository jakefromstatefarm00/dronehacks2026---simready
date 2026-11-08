#!/usr/bin/env bash
# best-v5 A/B feeder (v4c vs v5 candidate, runs v9xx), a copy of jolie_flyx/turns_feeder.sh with our own
# names. Appends to jolie_flyx/queue.txt only; never reorders or removes other entries. Keeps at
# most MAXMINE of OUR flights (9xx) queued+active, queues only while GB10's 1- and 5-min load
# averages are below MAXLOAD. Logs /proc/loadavg every 15 s (v5_load.log) and `uptime` at the
# start and end of each of our runs (v5_uptime.log).
# usage: v5_feeder.sh plan.txt   (lines: NNN jolie_flyx/with_depth.py <abs program path> [args])
cd ~/dronehacks-2026-gb10
PLAN=$1; MAXLOAD=${MAXLOAD:-15}; MAXMINE=${MAXMINE:-2}
Q=jolie_flyx/queue.txt; A=jolie_flyx/active.txt; D=jolie_flyx/done.txt
L=jolie_flyx/v5_load.log; U=jolie_flyx/v5_uptime.log; F=jolie_flyx/v5_feeder.log
( while [ ! -f jolie_flyx/V5_STOP ]; do
    echo "$(date +%s) $(cut -d' ' -f1-3 /proc/loadavg)" >> $L
    for n in $(cut -d' ' -f1 $PLAN); do
      if grep -qx "$n" $A && [ ! -e jolie_flyx/.v5_started_$n ]; then
        touch jolie_flyx/.v5_started_$n; echo "v$n start $(date +%s) $(uptime)" >> $U; fi
      if grep -q "^v$n done" $D && [ ! -e jolie_flyx/.v5_ended_$n ]; then
        touch jolie_flyx/.v5_ended_$n; echo "v$n end $(date +%s) $(uptime)" >> $U; fi
    done
    sleep 15; done ) &
SAMPLER=$!
trap 'kill $SAMPLER' EXIT
lowload() { read l1 l5 _ < /proc/loadavg; awk -v a=$l1 -v b=$l5 -v m=$MAXLOAD 'BEGIN{exit !(a<m && b<m)}'; }
while read -r line; do
  [ -z "$line" ] && continue
  n=${line%% *}
  [ -e artifacts/gb10-sky-beta-course-v$n ] && continue
  while true; do
    [ -f jolie_flyx/V5_STOP ] && exit 0
    mine=$(( $(grep -c '^9[0-9][0-9]$' $A) + $(grep -c '^9[0-9][0-9] ' $Q) ))
    [ $mine -lt $MAXMINE ] && lowload && break
    sleep 15
  done
  echo "$line" >> $Q
  echo "$(date +%T) queued $line load $(cut -d' ' -f1-3 /proc/loadavg)" >> $F
  sleep 30
done < "$PLAN"
echo "$(date +%T) plan done" >> $F
while true; do
  left=0; for n in $(cut -d' ' -f1 $PLAN); do grep -q "^v$n done" $D || left=1; done
  [ $left = 0 ] && break; [ -f jolie_flyx/V5_STOP ] && break; sleep 30
done
sleep 20; echo "$(date +%T) all done" >> $F
