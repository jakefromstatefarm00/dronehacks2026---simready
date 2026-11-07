#!/usr/bin/env bash
# One planner flight in Gazebo Harmonic (Docker; Linux or WSL2 Ubuntu).
#
#   gzsim/run.sh                         # published course, depth camera on
#   gzsim/run.sh --clutter 3             # + unmapped obstacles (seed 3)
#   gzsim/run.sh --clutter 3 --no-depth  # same world, published map only (expect a crash)
#   gzsim/run.sh --name my-run --planner astar
#
# Uses the course world and X3 quad from github.com/machmoon/dronehacks-sim (cloned into
# gzsim/.dronehacks-sim on first run; not our code). Watch it at
# http://localhost:8080/vnc.html?autoconnect=1&resize=scale . Results: gzsim/runs/<name>/.
set -euo pipefail
here="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
root="$(dirname "$here")"                       # pathfinding/
sim="$here/.dronehacks-sim"
runs="$here/runs"

name="run-$(date +%Y%m%d-%H%M%S)"; world=nominal; seed=0; depth="--depth-topic /course_quad/depth"; extra=()
while (( $# )); do
    case "$1" in
        --clutter) world=clutter; seed="$2"; shift 2 ;;
        --no-depth) depth=""; shift ;;
        --name) name="$2"; shift 2 ;;
        *) extra+=("$1"); shift ;;               # passed to fly_planner.py (e.g. --planner astar)
    esac
done

if [[ ! -d "$sim" ]]; then
    git clone -q https://github.com/machmoon/dronehacks-sim "$sim"
fi
if ! docker image inspect dronehacks-sim:planner >/dev/null 2>&1; then
    docker build -q -t dronehacks-sim:local "$sim"
    printf 'FROM dronehacks-sim:local\nRUN apt-get update && DEBIAN_FRONTEND=noninteractive apt-get install -y --no-install-recommends python3-numpy python3-scipy python3-yaml && rm -rf /var/lib/apt/lists/*\n' \
        | docker build -q -t dronehacks-sim:planner -
fi

mkdir -p "$runs/worlds"
wname="$world-$seed"
gen=(--sim-root /workspace --output "/out/worlds/$wname" --seed "$seed")
[[ $world == clutter ]] && gen+=(--clutter)
mounts=(-v "$sim:/workspace:ro" -v "$root:/repo:ro" -v "$runs:/out")
docker run --rm "${mounts[@]}" dronehacks-sim:planner python3 /repo/gzsim/make_world.py "${gen[@]}"

docker rm -f dh-sim >/dev/null 2>&1 || true
docker run -d --name dh-sim --init --shm-size 256m -e SIM_MISSION=0 -e SIM_GUI=1 \
    -e SIM_WORLD="/out/worlds/$wname/course.sdf" \
    -e GZ_SIM_RESOURCE_PATH="/out/worlds/$wname/models:/workspace/models:/workspace/assets" \
    -p 127.0.0.1:8080:6080 "${mounts[@]}" dronehacks-sim:planner >/dev/null
for _ in $(seq 120); do
    docker exec dh-sim gz topic -l 2>/dev/null | grep -q /model/course_quad/odometry && break
    sleep 1
done
echo "Gazebo up: http://localhost:8080/vnc.html?autoconnect=1&resize=scale"
rm -rf "$runs/$name"
# shellcheck disable=SC2086
docker exec dh-sim timeout 500 python3 /repo/gzsim/fly_planner.py --output "/out/$name" $depth "${extra[@]}" || true
echo "results: $runs/$name/summary.json   (the sim keeps running for viewing: docker rm -f dh-sim)"
