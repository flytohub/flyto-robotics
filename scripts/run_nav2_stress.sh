#!/usr/bin/env bash
set -euo pipefail

repository_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
image_name="${FLYTO_ROBOTICS_IMAGE:-flyto-robotics:jazzy-harmonic}"
run_id="${FLYTO_ROBOTICS_STRESS_RUN_ID:-$(date -u +%Y%m%dT%H%M%SZ)}"
run_directory="results/nav2-stress/${run_id}"
domain_id="${FLYTO_ROBOTICS_ROS_DOMAIN_ID:-91}"
soak_runs="${FLYTO_ROBOTICS_STRESS_SOAK_RUNS:-5}"

if [[ ! "${soak_runs}" =~ ^[0-9]+$ ]] || ((soak_runs < 1 || soak_runs > 100)); then
  echo "FLYTO_ROBOTICS_STRESS_SOAK_RUNS must be between 1 and 100" >&2
  exit 2
fi
required_domains=$((soak_runs + 3))
if [[ ! "${domain_id}" =~ ^[0-9]+$ ]] || ((domain_id + required_domains - 1 > 230)); then
  echo "FLYTO_ROBOTICS_ROS_DOMAIN_ID range cannot fit this stress run" >&2
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

evidence_files=()
scenario_index=0
run_scenario() {
  local scenario="$1"
  local label="$2"
  local evidence="${repository_root}/${run_directory}/${label}.json"
  local scenario_domain=$((domain_id + scenario_index))
  docker run --rm \
    -e "ROS_DOMAIN_ID=${scenario_domain}" \
    -e "RCUTILS_LOGGING_USE_STDOUT=1" \
    -e "FLYTO_NAV2_SCENARIO=${scenario}" \
    -e "FLYTO_NAV2_EVIDENCE=/workspace/${run_directory}/${label}.json" \
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
    ' 2>&1 | tee "${repository_root}/${run_directory}/${label}.log"
  test -s "${evidence}"
  python3 -m flyto_robotics.cli verify-ros2-execution-evidence \
    --evidence "${evidence}" \
    --scenario "${scenario}"
  evidence_files+=("${evidence}")
  scenario_index=$((scenario_index + 1))
}

for ((run=1; run<=soak_runs; run++)); do
  printf -v label 'success-%03d' "${run}"
  run_scenario success "${label}"
done
for scenario in lidar_dropout odometry_freeze nav2_lifecycle_failure; do
  run_scenario "${scenario}" "${scenario}"
done

grant_probe="${repository_root}/${run_directory}/grant-expiry.json"
python3 -m flyto_robotics.cli prove-ros2-expired-grant \
  --manifest "${repository_root}/examples/ros2-adapters/flyto2-standard.json" \
  --runtime "${repository_root}/examples/ros2-runtime/ready-sim.json" \
  --resource-plan "${repository_root}/examples/resource-plans/nav2-hospital-delivery.json" \
  --semantic-map "${repository_root}/examples/maps/atomic-color-route.json" \
  --output "${grant_probe}"

report="${repository_root}/${run_directory}/report.json"
python3 -m flyto_robotics.cli build-ros2-stress-report \
  --evidence "${evidence_files[@]}" \
  --grant-expiry-probe "${grant_probe}" \
  --soak-runs "${soak_runs}" \
  --output "${report}"
python3 -m flyto_robotics.cli verify-ros2-stress-report --report "${report}"

echo "Nav2 stress evidence: ${repository_root}/${run_directory}"
