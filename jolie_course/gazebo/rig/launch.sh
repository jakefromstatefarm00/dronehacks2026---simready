#!/usr/bin/env bash
# usage: [DEPTH=1] launch.sh NNN <program path rel to repo> [extra args]
# DEPTH=1 (sim only, opt-in): run the program under jolie_flyx/with_depth.py, which adds
# /camera/depth/image_raw + /camera/depth/camera_info (see jolie_course/gazebo/DEPTH.md).
N=$1; P=$2; shift 2
cd ~/dronehacks-2026-gb10
OUT=artifacts/gb10-sky-beta-course-v$N
[ -e $OUT ] && { echo "exists $OUT"; exit 1; }
PRE=""; [ "${DEPTH:-0}" = 1 ] && PRE="$HOME/dronehacks-2026-gb10/jolie_flyx/with_depth.py "
(NATIVE_PROGRAM="$PRE$HOME/dronehacks-2026-gb10/$P --output $HOME/dronehacks-2026-gb10/$OUT/journal $*" nohup sim/runtime/gb10_slot_run.sh artifacts/published-course-continuous-k12/trajectory.csv configs/sky_beta_course_continuous_k12_ff.json $OUT > /dev/null 2>&1 &)
echo "v$N $P $* ${PRE:+[depth] }started $(date +%T)" | tee -a ~/dronehacks-2026-gb10/jolie_flyx/runs.txt
