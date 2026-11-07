#!/usr/bin/env bash
# Stop the process groups recorded by run_all.sh / fly scripts in <run-dir>/pids (nothing else).
RUN="$1"
[ -f "$RUN/pids" ] || { echo "no $RUN/pids"; exit 1; }
tac "$RUN/pids" | while read -r name pid; do
  kill -INT -- "-$pid" 2>/dev/null && echo "stopping $name ($pid)"
done
sleep 5
tac "$RUN/pids" | while read -r name pid; do kill -KILL -- "-$pid" 2>/dev/null && echo "killed $name ($pid)"; done
true
