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
overhead_video="${run_directory}/gazebo-overhead.mp4"
driver_manifest="${run_directory}/images/driver-manifest.json"
mission_result="${run_directory}/mission-result.json"
showcase_report="${run_directory}/showcase-report.json"
lab_report="${run_directory}/lab-report.json"
logo_file="${FLYTO2_LOGO_FILE:-}"
font_file="${FLYTO2_CJK_FONT_FILE:-/System/Library/Fonts/PingFang.ttc}"
filter_file="${repository_root}/video/ai4all-showcase-filter.txt"
subtitle_file="${repository_root}/video/ai4all-showcase.ass"
output_name="flyto2-ai4all-showcase.mp4"
image_name="${FLYTO_ROBOTICS_IMAGE:-flyto-robotics:jazzy-harmonic}"
render_asset_directory=""
render_duration="25.25"

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
if [[ ! -f "${overhead_video}" ]]; then
  echo "missing overhead video: ${overhead_video}" >&2
  exit 2
fi
if [[ ! -f "${driver_manifest}" || ! -f "${mission_result}" ]]; then
  echo "missing driver manifest or mission result in ${run_directory}" >&2
  exit 2
fi

guarded_handoff_enabled="$(python3 -c \
  'import json, sys; value=json.load(open(sys.argv[1], encoding="utf-8")); print("true" if value.get("guarded_handoff", {}).get("enabled") is True else "false")' \
  "${driver_manifest}")"
if [[ "${guarded_handoff_enabled}" == "true" ]]; then
  filter_file="${repository_root}/video/ai4all-medication-showcase-filter.txt"
  subtitle_file="${repository_root}/video/ai4all-medication-showcase.ass"
  output_name="flyto2-ai4all-medication-showcase.mp4"
  render_duration="$(python3 -c \
    'import json, sys; value=json.load(open(sys.argv[1], encoding="utf-8")); print("{:.2f}".format(min(180.0, max(25.25, float(value["elapsed_seconds"]) + 3.0))))' \
    "${mission_result}")"
fi
output_video="${run_directory}/${output_name}"
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
mkdir -p "${render_asset_directory}/fonts"
cp "${logo_file}" "${render_asset_directory}/flyto2-logo.png"
cp "${font_file}" "${render_asset_directory}/fonts/cjk-font.ttf"
filter_asset_name="$(basename "${filter_file}")"
subtitle_asset_name="$(basename "${subtitle_file}")"
cp "${filter_file}" "${render_asset_directory}/${filter_asset_name}"
cp "${subtitle_file}" "${render_asset_directory}/${subtitle_asset_name}"

if [[ "${guarded_handoff_enabled}" == "true" ]]; then
  python3 - \
    "${driver_manifest}" \
    "${render_asset_directory}/${subtitle_asset_name}" \
    "${render_duration}" <<'PY'
import json
import sys
from pathlib import Path


def ass_time(value: float) -> str:
    bounded = max(0.0, value)
    hours = int(bounded // 3600)
    minutes = int((bounded % 3600) // 60)
    seconds = bounded % 60
    return f"{hours}:{minutes:02d}:{seconds:05.2f}"


manifest_path = Path(sys.argv[1])
subtitle_path = Path(sys.argv[2])
render_end = float(sys.argv[3])
manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
actions = {
    item["kind"]: float(item["at_seconds"])
    for item in manifest.get("actions", [])
    if item.get("kind") in {
        "item_rejected",
        "item_verified",
        "checkpoint_resumed",
        "recipient_rejected",
        "recipient_verified",
        "container_unlocked",
        "handoff_completed",
    }
}
ordered = [
    "item_rejected",
    "item_verified",
    "checkpoint_resumed",
    "recipient_rejected",
    "recipient_verified",
    "container_unlocked",
    "handoff_completed",
]
missing = [kind for kind in ordered if kind not in actions]
if missing:
    raise SystemExit(f"missing guarded handoff video events: {missing}")

copy = subtitle_path.read_text(encoding="utf-8")
for index, kind in enumerate(ordered):
    start = max(0.0, actions[kind] - 0.08)
    next_time = (
        actions[ordered[index + 1]]
        if index + 1 < len(ordered)
        else render_end
    )
    end = min(render_end, max(start + 0.25, next_time - 0.08))
    copy = copy.replace(f"__{kind.upper()}_START__", ass_time(start))
    copy = copy.replace(f"__{kind.upper()}_END__", ass_time(end))
copy = copy.replace("__VIDEO_END__", ass_time(render_end))
subtitle_path.write_text(copy, encoding="utf-8")
PY
fi

docker run --rm \
  -e LANG=C.utf8 \
  -e LC_ALL=C.utf8 \
  -v "${run_directory}:/evidence" \
  -v "${render_asset_directory}:/assets:ro" \
  "${image_name}" \
  ffmpeg -hide_banner -loglevel warning -y \
    -i /evidence/gazebo-active-camera.mp4 \
    -i /evidence/gazebo-overhead.mp4 \
    -loop 1 -i /assets/flyto2-logo.png \
    -filter_complex_script "/assets/${filter_asset_name}" \
    -map "[out]" \
    -t "${render_duration}" \
    -r 30 \
    -c:v libx264 -preset medium -crf 19 -pix_fmt yuv420p \
    -movflags +faststart \
    "/evidence/${output_name}"

docker run --rm \
  -e LANG=C.utf8 \
  -e LC_ALL=C.utf8 \
  -v "${run_directory}:/evidence" \
  "${image_name}" \
  ffprobe -v error -show_entries \
    format=duration,size:stream=codec_name,width,height,avg_frame_rate,nb_frames \
    -of json "/evidence/${output_name}" \
    > "${run_directory}/showcase-video-probe.json"

shasum -a 256 "${output_video}" > "${output_video}.sha256"
echo "Flyto2 showcase video: ${output_video}"
