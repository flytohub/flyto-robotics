#!/usr/bin/env bash
set -euo pipefail

repository_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
image_name="${FLYTO_ROBOTICS_IMAGE:-flyto-robotics:jazzy-harmonic}"
run_id="${FLYTO_ROBOTICS_SHORTCUT_RUN_ID:-$(date -u +%Y%m%dT%H%M%SZ)}"
run_directory="results/shortcut-gazebo/${run_id}"
container_name="flyto-shortcut-gazebo-${run_id//[^A-Za-z0-9_.-]/-}"
video_fps="${FLYTO_ROBOTICS_VIDEO_FPS:-4}"
video_max_frames="${FLYTO_ROBOTICS_VIDEO_MAX_FRAMES:-600}"

if [[ ! "${video_fps}" =~ ^[1-9][0-9]*$ ]] || ((video_fps > 30)); then
  echo "FLYTO_ROBOTICS_VIDEO_FPS must be an integer between 1 and 30" >&2
  exit 2
fi
if [[ ! "${video_max_frames}" =~ ^[1-9][0-9]*$ ]] || ((video_max_frames > 3600)); then
  echo "FLYTO_ROBOTICS_VIDEO_MAX_FRAMES must be between 1 and 3600" >&2
  exit 2
fi

if ! docker image inspect "${image_name}" >/dev/null 2>&1; then
  docker build \
    -t "${image_name}" \
    -f "${repository_root}/docker/Dockerfile.jazzy" \
    "${repository_root}"
fi
mkdir -p \
  "${repository_root}/${run_directory}/images" \
  "${repository_root}/${run_directory}/video-frames"

set +e
docker run --rm \
  --name "${container_name}" \
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
    timeout --signal=TERM --kill-after=10s 120s \
      ros2 launch flyto_robotics shortcut_gazebo_demo.launch.py \
        headless:=true \
        result_file:=/workspace/${run_directory}/shortcut-result.json \
        evidence_dir:=/workspace/${run_directory}/images \
        video_frames_dir:=/workspace/${run_directory}/video-frames \
        video_max_frames:=${video_max_frames}
    test -f /workspace/${run_directory}/shortcut-result.json
    test -f /workspace/${run_directory}/images/driver-manifest.json
    python3 -m flyto_robotics.shortcut_evidence \
      --result /workspace/${run_directory}/shortcut-result.json \
      --manifest /workspace/${run_directory}/images/driver-manifest.json \
      --report /workspace/${run_directory}/report.json \
      --markdown /workspace/${run_directory}/report.md
    frame_count=\$(find /workspace/${run_directory}/video-frames \
      -name 'frame-*.png' -type f | wc -l)
    elapsed=\$(python3 -c \
      'import json, sys; print(max(1.0, float(json.load(open(sys.argv[1], encoding=\"utf-8\"))[\"elapsed_seconds\"])))' \
      /workspace/${run_directory}/shortcut-result.json)
    source_fps=\$(python3 -c \
      'import sys; print(max(0.1, int(sys.argv[1]) / float(sys.argv[2])))' \
      \${frame_count} \${elapsed})
    ffmpeg -hide_banner -loglevel warning -y \
      -framerate \${source_fps} \
      -start_number 1 \
      -i /workspace/${run_directory}/video-frames/frame-%06d.png \
      -r ${video_fps} \
      -c:v libx264 -preset medium -crf 20 -pix_fmt yuv420p \
      -movflags +faststart \
      /workspace/${run_directory}/gazebo-shortcut-closed-loop.mp4
    ffprobe -v error -show_entries \
      format=duration,size:stream=codec_name,width,height,avg_frame_rate,nb_frames \
      -of json /workspace/${run_directory}/gazebo-shortcut-closed-loop.mp4 \
      > /workspace/${run_directory}/video-probe.json
    video_sha=\$(sha256sum \
      /workspace/${run_directory}/gazebo-shortcut-closed-loop.mp4 \
      | awk '{print \$1}')
    printf '%s  %s\n' "\${video_sha}" 'gazebo-shortcut-closed-loop.mp4' \
      > /workspace/${run_directory}/gazebo-shortcut-closed-loop.mp4.sha256
  "
status=$?
set -e

echo "Gazebo shortcut evidence: ${repository_root}/${run_directory}"
echo "Gazebo shortcut video: ${repository_root}/${run_directory}/gazebo-shortcut-closed-loop.mp4"
exit "${status}"
