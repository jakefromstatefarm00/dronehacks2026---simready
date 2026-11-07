#!/usr/bin/env bash
# usage: launch.sh NNN <program path rel to repo> [extra args]
N=$1; P=$2; shift 2
cd ~/dronehacks-2026-gb10
OUT=artifacts/gb10-sky-beta-course-v$N
[ -e $OUT ] && { echo "exists $OUT"; exit 1; }
(NATIVE_PROGRAM="$HOME/dronehacks-2026-gb10/$P --output $HOME/dronehacks-2026-gb10/$OUT/journal $*" nohup sim/runtime/gb10_slot_run.sh artifacts/published-course-continuous-k12/trajectory.csv configs/sky_beta_course_continuous_k12_ff.json $OUT > /dev/null 2>&1 &)
echo "v$N $P $* started $(date +%T)" | tee -a ~/dronehacks-2026-gb10/jolie_flyx/runs.txt
