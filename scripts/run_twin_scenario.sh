#!/usr/bin/env bash
# Run one scenarios/twin/*.json scenario against the TurtleBot3 twin (Docker).
#
#   ./scripts/run_twin_scenario.sh scenarios/twin/demo-room-obstacle-stop.json
#
# Brings the twin up in the scenario's world and start pose, waits for Nav2's
# behavior server and bt_navigator to be active, runs the scenario through the
# Generic ROS 2 Adapter over rosbridge (sim/twin/tools/run_scenario.py), copies
# the twin's service logs out, and takes the twin down. Exits non-zero unless
# every check in the verdict passed. Evidence lands in results/twin-scenario/.
#
# Simulation only. It publishes the twin on 127.0.0.1:19090, the same port as
# the robot's SSH tunnel, so it refuses to start while that port is taken.
set -euo pipefail

repository_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
scenario="${1:?usage: run_twin_scenario.sh SCENARIO.json}"
python_bin="${PYTHON:-python3}"
run_id="${FLYTO_ROBOTICS_TWIN_RUN_ID:-$(date -u +%Y%m%dT%H%M%SZ)}"
run_directory="${repository_root}/results/twin-scenario/${run_id}"
container="flyto-turtlebot3-twin"
ready_seconds="${FLYTO_ROBOTICS_TWIN_READY_SECONDS:-600}"
compose=(docker compose -f "${repository_root}/sim/twin/compose.yaml")

if (exec 3<>/dev/tcp/127.0.0.1/19090) 2>/dev/null; then
  echo "127.0.0.1:19090 is already in use (a twin or the robot's tunnel); refusing" >&2
  exit 2
fi

# World and start pose come from the scenario, validated before anything starts.
read -r TWIN_WORLD TWIN_X TWIN_Y TWIN_YAW < <(
  "${python_bin}" - "${scenario}" "${repository_root}/sim/twin" <<'PY'
import sys
from pathlib import Path
sys.path.insert(0, str(Path(sys.argv[2]) / "tools"))
from scenario_check import load_scenario
twin = load_scenario(Path(sys.argv[1]), twin_root=Path(sys.argv[2]))["twin"]
pose = twin["start_pose"]
print(twin["world"], pose["x"], pose["y"], pose["yaw"])
PY
)
export TWIN_WORLD TWIN_X TWIN_Y TWIN_YAW
mkdir -p "${run_directory}"

collect() {
  docker cp "${container}:/var/log/twin" "${run_directory}/twin-logs" >/dev/null 2>&1 || true
  "${compose[@]}" logs --no-color >"${run_directory}/compose.log" 2>&1 || true
  "${compose[@]}" down --timeout 20 >/dev/null 2>&1 || true
}
trap collect EXIT

"${compose[@]}" up -d --build

lifecycle_active() {
  docker exec "${container}" bash -c \
    "source /opt/ros/jazzy/setup.bash && ros2 lifecycle get /$1" 2>/dev/null | grep -q '^active'
}

waited=0
until lifecycle_active behavior_server && lifecycle_active bt_navigator; do
  if ! docker inspect -f '{{.State.Running}}' "${container}" 2>/dev/null | grep -q true; then
    echo "the twin container stopped before Nav2 became active" >&2
    exit 1
  fi
  if ((waited >= ready_seconds)); then
    echo "Nav2 did not become active within ${ready_seconds}s" >&2
    exit 1
  fi
  sleep 10
  waited=$((waited + 10))
done
echo "Nav2 active after ~${waited}s"

"${python_bin}" "${repository_root}/sim/twin/tools/run_scenario.py" "${scenario}" \
  --url ws://127.0.0.1:19090 \
  --output "${run_directory}/verdict.json" 2>&1 | tee "${run_directory}/scenario.log"
