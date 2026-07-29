#!/usr/bin/env bash
set -euo pipefail

repository_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
matrix_id="${FLYTO_ROBOTICS_MATRIX_ID:-$(date -u +%Y%m%dT%H%M%SZ)}"
run_count="${FLYTO_ROBOTICS_GAZEBO_RUNS:-3}"
matrix_directory="${repository_root}/results/gazebo-matrix/${matrix_id}"

if [[ ! "${run_count}" =~ ^[0-9]+$ ]] || ((run_count < 1 || run_count > 20)); then
  echo "FLYTO_ROBOTICS_GAZEBO_RUNS must be an integer between 1 and 20" >&2
  exit 2
fi

mkdir -p "${matrix_directory}"
report_paths=()
for ((run_number = 1; run_number <= run_count; run_number += 1)); do
  printf -v run_suffix "%02d" "${run_number}"
  run_id="${matrix_id}-run-${run_suffix}"
  FLYTO_ROBOTICS_LAB_RUN_ID="${run_id}" \
    "${repository_root}/scripts/run-gazebo-lab.sh"
  report_paths+=(
    "${repository_root}/results/gazebo-lab/${run_id}/report.json"
  )
done

python3 -m flyto_robotics.cli aggregate-lab \
  --reports "${report_paths[@]}" \
  --report "${matrix_directory}/report.json" \
  --markdown "${matrix_directory}/report.md" \
  --junit "${matrix_directory}/junit.xml"

echo "Gazebo matrix evidence: ${matrix_directory}"
