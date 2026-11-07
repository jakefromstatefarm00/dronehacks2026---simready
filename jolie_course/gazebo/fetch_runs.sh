#!/usr/bin/env bash
# fetch_runs.sh NNN... : pull the small scoring files of GB10 runs into $RUNS_DIR (default ./gazebo-runs)
D="${RUNS_DIR:-./gazebo-runs}"
mkdir -p "$D"; cd "$D"
for n in "$@"; do
  r=gb10-sky-beta-course-v$n
  ssh promaxgb10-6c88 "cd ~/dronehacks-2026-gb10/artifacts && tar czf - $r.launch.log $r/truth-poses.csv $r/native-program.log $r/journal" | tar xzf -
done
