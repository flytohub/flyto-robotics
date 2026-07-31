#!/usr/bin/env bash
set -euo pipefail

script_directory="$(dirname "${BASH_SOURCE[0]}")"
repository_root="$(realpath "${script_directory}/..")"
image_name="${FLYTO_ROBOTICS_IMAGE:-flyto-robotics:jazzy-harmonic}"
run_id="${FLYTO_ROBOTICS_SHOWCASE_RUN_ID:-$(date -u +%Y%m%dT%H%M%SZ)}"
run_directory="results/ai4all-showcase/${run_id}"
container_name="flyto-ai4all-showcase-${run_id//[^A-Za-z0-9_.-]/-}"
approval_key_material="$(python3 -c 'import secrets; print(secrets.token_hex(32))')"
qr_key_material="$(python3 -c 'import secrets; print(secrets.token_hex(32))')"
video_fps="${FLYTO_ROBOTICS_VIDEO_FPS:-8}"
planner_url="${FLYTO_ROBOTICS_PLANNER_URL:-http://127.0.0.1:8787/v1/robotics/plan}"
showcase_goal="${FLYTO_ROBOTICS_SHOWCASE_GOAL:-把補給品送到紫區護理站；設備失效時改走可驗證的安全路線，交付前需要人員確認。}"
showcase_launch="${FLYTO_ROBOTICS_SHOWCASE_LAUNCH:-ai4all_showcase.launch.py}"
lab_scenario="${FLYTO_ROBOTICS_LAB_SCENARIO:-scenarios/gazebo/ai4all-branching.json}"

if [[ ! "${video_fps}" =~ ^[1-9][0-9]*$ ]] || ((video_fps > 30)); then
  echo "FLYTO_ROBOTICS_VIDEO_FPS must be an integer between 1 and 30" >&2
  exit 2
fi
if [[ ! "${run_id}" =~ ^[A-Za-z0-9][A-Za-z0-9_.-]{0,63}$ ]]; then
  echo "FLYTO_ROBOTICS_SHOWCASE_RUN_ID must be a safe identifier" >&2
  exit 2
fi
if [[ ! "${showcase_launch}" =~ ^[A-Za-z0-9][A-Za-z0-9_.-]*\.launch\.py$ ]]; then
  echo "FLYTO_ROBOTICS_SHOWCASE_LAUNCH must be a safe ROS launch filename" >&2
  exit 2
fi
if [[ ! "${lab_scenario}" =~ ^scenarios/gazebo/[A-Za-z0-9][A-Za-z0-9_.-]*\.json$ ]]; then
  echo "FLYTO_ROBOTICS_LAB_SCENARIO must be a Gazebo scenario in scenarios/gazebo" >&2
  exit 2
fi
if [[ ! -f "${repository_root}/${lab_scenario}" ]]; then
  echo "Gazebo lab scenario does not exist: ${lab_scenario}" >&2
  exit 2
fi

if ! docker image inspect "${image_name}" >/dev/null 2>&1; then
  docker build \
    -t "${image_name}" \
    -f "${repository_root}/docker/Dockerfile.jazzy" \
    "${repository_root}"
fi

mkdir -p \
  "${repository_root}/${run_directory}/facility" \
  "${repository_root}/${run_directory}/frames/active-camera" \
  "${repository_root}/${run_directory}/frames/overhead" \
  "${repository_root}/${run_directory}/images"

PYTHONPATH="${repository_root}" python3 -m flyto_robotics.planning_session \
  --scenario-file \
    "${repository_root}/examples/routes/ai4all-branching-routes.json" \
  --semantic-map-file \
    "${repository_root}/examples/maps/ai4all-branching-route.json" \
  --semantic-map-id gazebo.ai4all-branching-route.v1 \
  --planner-url "${planner_url}" \
  --output-dir "${repository_root}/${run_directory}" \
  --goal "${showcase_goal}" \
  --robot-id flyto-rover-sim-001 \
  --timeout-seconds 120

set +e
docker run --rm \
  --name "${container_name}" \
  -e "FLYTO_ROBOTICS_APPROVAL_SECRET=${approval_key_material}" \
  -e "FLYTO_ROBOTICS_QR_SECRET=${qr_key_material}" \
  -v "${repository_root}:/workspace" \
  -w /workspace \
  "${image_name}" \
  bash -lc "
    set -eo pipefail
    source /opt/ros/jazzy/setup.bash
    set -u
    colcon build --symlink-install
    set +u
    source install/setup.bash
    set -u
    set +e
    timeout --signal=TERM --kill-after=10s 180s \
      ros2 launch flyto_robotics ${showcase_launch} \
        headless:=true \
        output_dir:=/workspace/${run_directory} \
        plan_file:=/workspace/${run_directory}/validated-plan.json \
        planning_session_file:=/workspace/${run_directory}/planning-session.json
    launch_status=\$?
    set -e
    required_files=(
      /workspace/${run_directory}/mission-result.json
      /workspace/${run_directory}/images/driver-manifest.json
      /workspace/${run_directory}/facility/showcase-evidence.json
      /workspace/${run_directory}/planning-session.json
      /workspace/${run_directory}/validated-plan.json
    )
    for required_file in \"\${required_files[@]}\"; do
      if [[ ! -f \"\${required_file}\" ]]; then
        echo \"Gazebo showcase did not produce \${required_file}\" >&2
        exit \${launch_status:-3}
      fi
    done
    python3 -m flyto_robotics.cli evaluate-lab \
      --scenario /workspace/${lab_scenario} \
      --result /workspace/${run_directory}/mission-result.json \
      --evidence-dir /workspace/${run_directory}/images \
      --report /workspace/${run_directory}/lab-report.json \
      --markdown /workspace/${run_directory}/lab-report.md \
      --junit /workspace/${run_directory}/lab-junit.xml
    python3 -m flyto_robotics.showcase_evidence \
      --showcase /workspace/${run_directory}/facility/showcase-evidence.json \
      --mission /workspace/${run_directory}/mission-result.json \
      --driver /workspace/${run_directory}/images/driver-manifest.json \
      --report /workspace/${run_directory}/showcase-report.json \
      --markdown /workspace/${run_directory}/showcase-report.md

    mission_duration=\$(python3 -c \
      'import json, sys; print(float(json.load(open(sys.argv[1], encoding=\"utf-8\"))[\"elapsed_seconds\"]))' \
      /workspace/${run_directory}/mission-result.json)
    encode_frames() {
      local frames_dir=\"\$1\"
      local output_file=\"\$2\"
      shopt -s nullglob
      local frames=(\"\${frames_dir}\"/frame-*.png)
      local frame_count=\${#frames[@]}
      if ((frame_count < 2)); then
        echo \"Video requires at least two frames in \${frames_dir}; got \${frame_count}\" >&2
        exit 5
      fi
      local source_fps
      source_fps=\$(python3 -c \
        'import sys; print(max(0.1, int(sys.argv[1]) / float(sys.argv[2])))' \
        \${frame_count} \${mission_duration})
      ffmpeg -hide_banner -loglevel warning -y \
        -framerate \${source_fps} \
        -start_number 1 \
        -i \"\${frames_dir}/frame-%06d.png\" \
        -r ${video_fps} \
        -c:v libx264 -preset medium -crf 20 -pix_fmt yuv420p \
        -movflags +faststart \
        \"\${output_file}\"
    }
    encode_frames \
      /workspace/${run_directory}/frames/active-camera \
      /workspace/${run_directory}/gazebo-active-camera.mp4
    encode_frames \
      /workspace/${run_directory}/frames/overhead \
      /workspace/${run_directory}/gazebo-overhead.mp4
    ffprobe -v error -show_entries \
      format=duration,size:stream=codec_name,width,height,avg_frame_rate,nb_frames \
      -of json /workspace/${run_directory}/gazebo-active-camera.mp4 \
      > /workspace/${run_directory}/active-camera-video-probe.json
    sha256sum \
      /workspace/${run_directory}/gazebo-active-camera.mp4 \
      /workspace/${run_directory}/gazebo-overhead.mp4 \
      > /workspace/${run_directory}/videos.sha256
  "
status=$?
set -e
approval_key_material=""

echo "Flyto2 AI4ALL showcase evidence: ${repository_root}/${run_directory}"
exit "${status}"
