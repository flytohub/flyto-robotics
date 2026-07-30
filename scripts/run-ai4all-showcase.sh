#!/usr/bin/env bash
set -euo pipefail

script_directory="$(dirname "${BASH_SOURCE[0]}")"
repository_root="$(realpath "${script_directory}/..")"
image_name="${FLYTO_ROBOTICS_IMAGE:-flyto-robotics:jazzy-harmonic}"
run_id="${FLYTO_ROBOTICS_SHOWCASE_RUN_ID:-$(date -u +%Y%m%dT%H%M%SZ)}"
run_directory="results/ai4all-showcase/${run_id}"
container_name="flyto-ai4all-showcase-${run_id//[^A-Za-z0-9_.-]/-}"
approval_key_material="$(python3 -c 'import secrets; print(secrets.token_hex(32))')"
video_fps="${FLYTO_ROBOTICS_VIDEO_FPS:-8}"

if [[ ! "${video_fps}" =~ ^[1-9][0-9]*$ ]] || ((video_fps > 30)); then
  echo "FLYTO_ROBOTICS_VIDEO_FPS must be an integer between 1 and 30" >&2
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

set +e
docker run --rm \
  --name "${container_name}" \
  -e "FLYTO_ROBOTICS_APPROVAL_SECRET=${approval_key_material}" \
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
      ros2 launch flyto_robotics ai4all_showcase.launch.py \
        headless:=true \
        output_dir:=/workspace/${run_directory}
    launch_status=\$?
    set -e
    required_files=(
      /workspace/${run_directory}/mission-result.json
      /workspace/${run_directory}/images/driver-manifest.json
      /workspace/${run_directory}/facility/showcase-evidence.json
    )
    for required_file in \"\${required_files[@]}\"; do
      if [[ ! -f \"\${required_file}\" ]]; then
        echo \"Gazebo showcase did not produce \${required_file}\" >&2
        exit \${launch_status:-3}
      fi
    done
    python3 -m flyto_robotics.cli evaluate-lab \
      --scenario /workspace/scenarios/gazebo/careflow-adversarial.json \
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
