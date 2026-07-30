#!/usr/bin/env bash
set -euo pipefail

if (($# != 1)); then
  echo "usage: $0 <showcase-run-directory>" >&2
  exit 2
fi

script_directory="$(dirname "${BASH_SOURCE[0]}")"
repository_root="$(realpath "${script_directory}/..")"
run_directory="$(realpath "$1")"
input_video="${run_directory}/gazebo-active-camera.mp4"
output_video="${run_directory}/flyto2-ai4all-showcase.mp4"
logo_file="${FLYTO2_LOGO_FILE:-}"
font_file="${FLYTO2_CJK_FONT_FILE:-/System/Library/Fonts/PingFang.ttc}"
filter_file="${repository_root}/video/ai4all-showcase-filter.txt"
subtitle_file="${repository_root}/video/ai4all-showcase.ass"
image_name="${FLYTO_ROBOTICS_IMAGE:-flyto-robotics:jazzy-harmonic}"
render_asset_directory=""

cleanup() {
  if [[ -n "${render_asset_directory}" && -d "${render_asset_directory}" ]]; then
    rm -rf -- "${render_asset_directory}"
  fi
}
trap cleanup EXIT

if [[ ! -f "${input_video}" ]]; then
  echo "missing active-camera video: ${input_video}" >&2
  exit 2
fi
if [[ -z "${logo_file}" || ! -f "${logo_file}" ]]; then
  echo "FLYTO2_LOGO_FILE must point to the approved Flyto2 logo PNG" >&2
  exit 2
fi
if [[ ! -f "${font_file}" ]]; then
  echo "FLYTO2_CJK_FONT_FILE must point to a CJK font" >&2
  exit 2
fi
if [[ ! -f "${filter_file}" ]]; then
  echo "missing video filter: ${filter_file}" >&2
  exit 2
fi
if [[ ! -f "${subtitle_file}" ]]; then
  echo "missing video subtitles: ${subtitle_file}" >&2
  exit 2
fi

render_asset_directory="$(mktemp -d "${run_directory}/.flyto2-render-assets.XXXXXX")"
cp "${logo_file}" "${render_asset_directory}/flyto2-logo.png"
cp "${font_file}" "${render_asset_directory}/cjk-font.ttf"
cp "${filter_file}" "${render_asset_directory}/ai4all-showcase-filter.txt"
cp "${subtitle_file}" "${render_asset_directory}/ai4all-showcase.ass"

docker run --rm \
  -e LANG=C.utf8 \
  -e LC_ALL=C.utf8 \
  -v "${run_directory}:/evidence" \
  -v "${render_asset_directory}:/assets:ro" \
  "${image_name}" \
  ffmpeg -hide_banner -loglevel warning -y \
    -i /evidence/gazebo-active-camera.mp4 \
    -loop 1 -i /assets/flyto2-logo.png \
    -filter_complex_script /assets/ai4all-showcase-filter.txt \
    -map "[out]" \
    -t 25.65 \
    -r 30 \
    -c:v libx264 -preset medium -crf 19 -pix_fmt yuv420p \
    -movflags +faststart \
    /evidence/flyto2-ai4all-showcase.mp4

docker run --rm \
  -e LANG=C.utf8 \
  -e LC_ALL=C.utf8 \
  -v "${run_directory}:/evidence" \
  "${image_name}" \
  ffprobe -v error -show_entries \
    format=duration,size:stream=codec_name,width,height,avg_frame_rate,nb_frames \
    -of json /evidence/flyto2-ai4all-showcase.mp4 \
    > "${run_directory}/showcase-video-probe.json"

shasum -a 256 "${output_video}" > "${output_video}.sha256"
echo "Flyto2 showcase video: ${output_video}"
