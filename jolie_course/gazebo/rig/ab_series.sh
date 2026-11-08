#!/usr/bin/env bash
# sequential flights, one Gazebo slot, launched only at 1-min load < LOADMAX (default 20):
# series4.sh NNN:on|off ...  (PROG env = program).  Logs load at launch and max load during flight.
cd ~/dronehacks-2026-gb10
LOADMAX=${LOADMAX:-20}
for item in "$@"; do
  n=${item%%:*}; m=${item##*:}
  while pgrep -f "gb10-sky-beta-course-v4[0-9][0-9]" >/dev/null; do sleep 10; done
  [ -e artifacts/gb10-sky-beta-course-v$n ] && continue
  while awk -v L=$LOADMAX "{exit !(\$1>=L)}" /proc/loadavg; do sleep 20; done
  l0=$(cut -d" " -f1 /proc/loadavg)
  if [ "$m" = on ]; then DEPTH=1 jolie_flyx/launch.sh $n ${PROG:-jolie_flyx/upload_to_flyx.py}; else DEPTH=0 jolie_flyx/launch.sh $n ${PROG:-jolie_flyx/upload_to_flyx.py}; fi
  lmax=$l0; sleep 10
  while pgrep -f "gb10-sky-beta-course-v$n" >/dev/null; do l=$(cut -d" " -f1 /proc/loadavg); lmax=$(awk -v a=$l -v b=$lmax "BEGIN{print (a>b)?a:b}"); echo "$(date +%T) $l" >> artifacts/gb10-sky-beta-course-v$n.load.txt; sleep 10; done; sleep 3
  bash sim/runtime/post_flight.sh $n > artifacts/gb10-sky-beta-course-v$n/post.log 2>&1
  echo "v$n $m done $(date +%T) load launch $l0 max $lmax: $(tail -1 artifacts/gb10-sky-beta-course-v$n.launch.log | cut -c1-60) | $(grep RESULT artifacts/gb10-sky-beta-course-v$n/native-program.log) | $(grep "^depth:" artifacts/gb10-sky-beta-course-v$n/native-program.log)" >> ~/depth-agent/done.txt
  pgrep -af "depth_bridge|with_depth" | grep -v pgrep >> ~/depth-agent/done.txt
done
echo SERIES_DONE >> ~/depth-agent/done.txt
