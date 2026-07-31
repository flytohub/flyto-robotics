#!/usr/bin/env bash
set -euo pipefail

repository_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
image_name="${FLYTO_ROBOTICS_IMAGE:-flyto-robotics:jazzy-harmonic}"
run_id="${FLYTO_ROBOTICS_LAB_RUN_ID:-$(date -u +%Y%m%dT%H%M%SZ)}"
run_directory="results/gazebo-lab/${run_id}"
container_name="flyto-gazebo-lab-${run_id//[^A-Za-z0-9_.-]/-}"
approval_key_material="$(python3 -c 'import secrets; print(secrets.token_hex(32))')"
qr_key_material="$(python3 -c 'import secrets; print(secrets.token_hex(32))')"
record_video="${FLYTO_ROBOTICS_RECORD_VIDEO:-0}"
video_fps="${FLYTO_ROBOTICS_VIDEO_FPS:-4}"
video_max_frames="${FLYTO_ROBOTICS_VIDEO_MAX_FRAMES:-600}"

if [[ ! "${record_video}" =~ ^(0|1)$ ]]; then
  echo "FLYTO_ROBOTICS_RECORD_VIDEO must be 0 or 1" >&2
  exit 2
fi
if [[ ! "${video_fps}" =~ ^[1-9][0-9]*$ ]] || ((video_fps > 30)); then
  echo "FLYTO_ROBOTICS_VIDEO_FPS must be an integer between 1 and 30" >&2
  exit 2
fi
if [[ ! "${video_max_frames}" =~ ^[1-9][0-9]*$ ]] || ((video_max_frames > 3600)); then
  echo "FLYTO_ROBOTICS_VIDEO_MAX_FRAMES must be between 1 and 3600" >&2
  exit 2
fi

image_rebuild=0
if ! docker image inspect "${image_name}" >/dev/null 2>&1; then
  image_rebuild=1
elif [[ "${record_video}" == "1" ]] && ! docker run --rm \
  "${image_name}" bash -lc "command -v ffmpeg >/dev/null"; then
  image_rebuild=1
fi
if [[ "${image_rebuild}" == "1" ]]; then
  docker build \
    -t "${image_name}" \
    -f "${repository_root}/docker/Dockerfile.jazzy" \
    "${repository_root}"
fi

mkdir -p "${repository_root}/${run_directory}/images"
video_launch_arguments=""
if [[ "${record_video}" == "1" ]]; then
  mkdir -p "${repository_root}/${run_directory}/video-frames"
  video_launch_arguments="video_frames_dir:=/workspace/${run_directory}/video-frames video_max_frames:=${video_max_frames}"
fi

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
    timeout --signal=TERM --kill-after=10s 150s \
      ros2 launch flyto_robotics gazebo_lab.launch.py \
        headless:=true \
        result_file:=/workspace/${run_directory}/mission-result.json \
        evidence_dir:=/workspace/${run_directory}/images \
        ${video_launch_arguments}
    launch_status=\$?
    set -e
    if [[ ! -f /workspace/${run_directory}/mission-result.json ]]; then
      echo 'Gazebo did not produce a mission result' >&2
      exit \${launch_status:-3}
    fi
    python3 -m flyto_robotics.cli evaluate-lab \
      --scenario /workspace/scenarios/gazebo/careflow-adversarial.json \
      --result /workspace/${run_directory}/mission-result.json \
      --evidence-dir /workspace/${run_directory}/images \
      --report /workspace/${run_directory}/report.json \
      --markdown /workspace/${run_directory}/report.md \
      --junit /workspace/${run_directory}/junit.xml
    if [[ ${record_video} == 1 ]]; then
      frame_count=\$(find /workspace/${run_directory}/video-frames \
        -name 'frame-*.png' -type f | wc -l)
      if ((frame_count < 2)); then
        echo \"Gazebo video requires at least two captured frames; got \${frame_count}\" >&2
        exit 5
      fi
      mission_duration=\$(python3 -c \
        'import json, sys; print(float(json.load(open(sys.argv[1], encoding=\"utf-8\"))[\"elapsed_seconds\"]))' \
        /workspace/${run_directory}/mission-result.json)
      source_fps=\$(python3 -c \
        'import sys; print(max(0.1, int(sys.argv[1]) / float(sys.argv[2])))' \
        \${frame_count} \${mission_duration})
      ffmpeg -hide_banner -loglevel warning -y \
        -framerate \${source_fps} \
        -start_number 1 \
        -i /workspace/${run_directory}/video-frames/frame-%06d.png \
        -r ${video_fps} \
        -c:v libx264 -preset medium -crf 20 -pix_fmt yuv420p \
        -movflags +faststart \
        /workspace/${run_directory}/gazebo-careflow.mp4
      ffprobe -v error -show_entries \
        format=duration,size:stream=codec_name,width,height,avg_frame_rate,nb_frames \
        -of json /workspace/${run_directory}/gazebo-careflow.mp4 \
        > /workspace/${run_directory}/video-probe.json
      sha256sum /workspace/${run_directory}/gazebo-careflow.mp4 \
        > /workspace/${run_directory}/gazebo-careflow.mp4.sha256
    fi
  "
status=$?
set -e
approval_key_material=""

echo "Gazebo lab evidence: ${repository_root}/${run_directory}"
if [[ "${record_video}" == "1" ]]; then
  echo "Gazebo lab video: ${repository_root}/${run_directory}/gazebo-careflow.mp4"
fi
exit "${status}"
