#!/usr/bin/env bash
set -euo pipefail

script_directory="$(dirname "${BASH_SOURCE[0]}")"

export FLYTO_ROBOTICS_SHOWCASE_LAUNCH="ai4all_medication_showcase.launch.py"
export FLYTO_ROBOTICS_LAB_SCENARIO="scenarios/gazebo/ai4all-medication-handoff.json"
export FLYTO_ROBOTICS_SHOWCASE_GOAL="${FLYTO_ROBOTICS_SHOWCASE_GOAL:-幫 12 號病人領藥。只有批價完成、藥袋編號一致、病人驗證成功才能交付；設備失效時改走可驗證的安全路線。}"

exec "${script_directory}/run-ai4all-showcase.sh"
