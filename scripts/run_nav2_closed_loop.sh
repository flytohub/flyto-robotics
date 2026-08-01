#!/usr/bin/env bash
set -euo pipefail

repository_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
image_name="${FLYTO_ROBOTICS_IMAGE:-flyto-robotics:jazzy-harmonic}"
run_id="${FLYTO_ROBOTICS_NAV2_RUN_ID:-$(date -u +%Y%m%dT%H%M%SZ)}"
run_directory="results/nav2-closed-loop/${run_id}"
domain_id="${FLYTO_ROBOTICS_ROS_DOMAIN_ID:-73}"

if [[ ! "${domain_id}" =~ ^[0-9]+$ ]] || ((domain_id > 230)); then
  echo "FLYTO_ROBOTICS_ROS_DOMAIN_ID must be between 0 and 230" >&2
  exit 2
fi

image_rebuild=0
if ! docker image inspect "${image_name}" >/dev/null 2>&1; then
  image_rebuild=1
elif ! docker run --rm "${image_name}" bash -lc \
  "source /opt/ros/jazzy/setup.bash && ros2 pkg prefix nav2_bringup >/dev/null"; then
  image_rebuild=1
fi
if [[ "${image_rebuild}" == "1" ]]; then
  docker build \
    -t "${image_name}" \
    -f "${repository_root}/docker/Dockerfile.jazzy" \
    "${repository_root}"
fi

mkdir -p "${repository_root}/${run_directory}"
docker run --rm \
  -v "${repository_root}:/workspace" \
  -w /workspace \
  "${image_name}" \
  bash -lc '
    set -e
    source /opt/ros/jazzy/setup.bash
    colcon build --symlink-install
    source install/setup.bash
  '

scenario_index=0
for scenario in success cancel emergency_stop; do
  evidence="${repository_root}/${run_directory}/${scenario}.json"
  scenario_domain=$((domain_id + scenario_index))
  docker run --rm \
    -e "ROS_DOMAIN_ID=${scenario_domain}" \
    -e "RCUTILS_LOGGING_USE_STDOUT=1" \
    -e "FLYTO_NAV2_SCENARIO=${scenario}" \
    -e "FLYTO_NAV2_EVIDENCE=/workspace/${run_directory}/${scenario}.json" \
    -v "${repository_root}:/workspace" \
    -w /workspace \
    "${image_name}" \
    bash -lc '
      set -eo pipefail
      source /opt/ros/jazzy/setup.bash
      source install/setup.bash
      timeout --signal=TERM --kill-after=15s 180s \
        ros2 launch flyto_robotics nav2_closed_loop.launch.py \
          headless:=true \
          scenario:=${FLYTO_NAV2_SCENARIO} \
          output_file:=${FLYTO_NAV2_EVIDENCE}
    ' 2>&1 | tee "${repository_root}/${run_directory}/${scenario}.log"
  test -s "${evidence}"
  python3 -m flyto_robotics.cli verify-ros2-execution-evidence \
    --evidence "${evidence}" \
    --scenario "${scenario}"
  scenario_index=$((scenario_index + 1))
done

python3 - "${repository_root}/${run_directory}" <<'PY'
import json
import sys
from pathlib import Path
root = Path(sys.argv[1])
docs = {name: json.loads((root / f'{name}.json').read_text()) for name in ('success', 'cancel', 'emergency_stop')}
assert docs['success']['status'] == 'succeeded'
assert docs['cancel']['status'] == 'canceled'
assert docs['emergency_stop']['status'] == 'safety_stopped'
assert len({doc['snapshot'] for doc in docs.values()}) == 3
print(json.dumps({'passed': True, 'scenarios': {key: value['snapshot'] for key, value in docs.items()}}, sort_keys=True))
PY

echo "Nav2 closed-loop evidence: ${repository_root}/${run_directory}"
