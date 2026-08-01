#!/usr/bin/env bash
set -euo pipefail

if (($# != 1)); then
  echo "usage: $0 <gui-capture-run-directory>" >&2
  exit 2
fi

script_directory="$(dirname "${BASH_SOURCE[0]}")"
repository_root="$(realpath "${script_directory}/..")"
run_directory="$(realpath "$1")"
input_video="${run_directory}/gazebo-gui.mp4"
planning_session="${run_directory}/planning-session.json"
mission_result="${run_directory}/mission-result.json"
driver_manifest="${run_directory}/images/driver-manifest.json"
capture_metadata="${run_directory}/gui-capture-metadata.json"
window_geometry="${run_directory}/gazebo-window-geometry.env"
subtitle_file="${run_directory}/hospital-story.ass"
narration_schedule="${run_directory}/hospital-story-narration.tsv"
output_video="${run_directory}/flyto2-hospital-story.mp4"
logo_file="${FLYTO2_LOGO_FILE:-}"
font_file="${FLYTO2_CJK_FONT_FILE:-/System/Library/Fonts/PingFang.ttc}"
narration_voice="${FLYTO2_NARRATION_VOICE:-Meijia}"
narration_rate="${FLYTO2_NARRATION_RATE:-185}"
narration_enabled="${STORY_NARRATION:-1}"
image_name="${FLYTO_ROBOTICS_IMAGE:-flyto-robotics:jazzy-harmonic}"
intro_duration="8.0"
outro_duration="12.0"
render_assets=""

cleanup() {
  if [[ -n "${render_assets}" && -d "${render_assets}" ]]; then
    rm -rf -- "${render_assets}"
  fi
}
trap cleanup EXIT

required_files=(
  "${input_video}"
  "${planning_session}"
  "${mission_result}"
  "${driver_manifest}"
  "${capture_metadata}"
  "${window_geometry}"
)
for required_file in "${required_files[@]}"; do
  if [[ ! -s "${required_file}" ]]; then
    echo "missing story evidence: ${required_file}" >&2
    exit 2
  fi
done
if [[ -z "${logo_file}" || ! -f "${logo_file}" ]]; then
  echo "FLYTO2_LOGO_FILE must point to the approved Flyto2 logo PNG" >&2
  exit 2
fi
if [[ ! -f "${font_file}" ]]; then
  echo "FLYTO2_CJK_FONT_FILE must point to a CJK font" >&2
  exit 2
fi
if [[ "${narration_enabled}" != "0" ]] && ! command -v say >/dev/null 2>&1; then
  echo "macOS say is required for narration; set STORY_NARRATION=0 for a silent render" >&2
  exit 2
fi

read -r trim_start source_duration story_duration outro_start <<< "$(python3 -c \
  'import json, sys; m=json.load(open(sys.argv[1], encoding="utf-8")); c=json.load(open(sys.argv[2], encoding="utf-8")); intro=float(sys.argv[3]); outro=float(sys.argv[4]); source=float(m["elapsed_seconds"])*float(c.get("simulation_time_scale", 1.0))+3.0; print("{:.3f} {:.3f} {:.3f} {:.3f}".format(float(c["mission_offset_seconds"]), source, intro+source+outro, intro+source))' \
  "${mission_result}" "${capture_metadata}" "${intro_duration}" "${outro_duration}")"

read -r main_x main_y main_width main_height <<< "$(python3 - "${window_geometry}" <<'PY'
import re
import sys
from pathlib import Path

values = dict(
    re.findall(r"^(X|Y|WIDTH|HEIGHT)=(\d+)$", Path(sys.argv[1]).read_text(), re.M)
)
if set(values) != {"X", "Y", "WIDTH", "HEIGHT"}:
    raise SystemExit("invalid Gazebo window geometry")
x, y, width, height = (int(values[key]) for key in ("X", "Y", "WIDTH", "HEIGHT"))
main_x = x + round(width * 0.24)
main_y = y + round(height * 0.30)
main_width = round(width * 0.65) // 2 * 2
main_height = round(main_width * 9 / 16) // 2 * 2
if main_x + main_width > 1920 or main_y + main_height > 1080:
    raise SystemExit("derived Gazebo scene crop is outside the recorded screen")
print(main_x, main_y, main_width, main_height)
PY
)"

python3 - "${planning_session}" "${mission_result}" "${driver_manifest}" \
  "${capture_metadata}" "${subtitle_file}" "${narration_schedule}" \
  "${intro_duration}" "${source_duration}" "${story_duration}" <<'PY'
import json
import sys
from pathlib import Path


def ass_time(value: float) -> str:
    bounded = max(0.0, value)
    hours = int(bounded // 3600)
    minutes = int((bounded % 3600) // 60)
    seconds = bounded % 60
    return f"{hours}:{minutes:02d}:{seconds:05.2f}"


def safe_text(value: object) -> str:
    return str(value).replace("\\", "／").replace("{", "（").replace("}", "）").replace("\n", " ")


planning = json.loads(Path(sys.argv[1]).read_text(encoding="utf-8"))
mission = json.loads(Path(sys.argv[2]).read_text(encoding="utf-8"))
driver = json.loads(Path(sys.argv[3]).read_text(encoding="utf-8"))
capture = json.loads(Path(sys.argv[4]).read_text(encoding="utf-8"))
subtitle_output = Path(sys.argv[5])
narration_output = Path(sys.argv[6])
intro = float(sys.argv[7])
source_duration = float(sys.argv[8])
story_end = float(sys.argv[9])
outro_start = intro + source_duration
time_scale = float(capture.get("simulation_time_scale", 1.0))


def timeline(simulation_seconds: float) -> float:
    return intro + simulation_seconds * time_scale


mission_events = {event["kind"]: event for event in mission["events"]}
driver_events = {}
for event in driver.get("actions", []):
    driver_events.setdefault(event["kind"], event)


def event_time(kind: str, source: dict[str, dict], fallback: float) -> float:
    event = source.get(kind)
    return timeline(float(event["at_seconds"]) if event else fallback)


obstacle_in = event_time("fault_injection", driver_events, 3.0)
obstacle_stop = event_time("obstacle_stop", mission_events, 3.3)
path_clear = event_time("path_clear", mission_events, 6.2)
destination = next(
    (
        timeline(float(event["at_seconds"]))
        for event in mission["events"]
        if event.get("kind") == "primitive_completed" and event.get("step_id") == "step-4"
    ),
    timeline(21.9),
)

lines = [
    "[Script Info]",
    "ScriptType: v4.00+",
    "PlayResX: 1920",
    "PlayResY: 1080",
    "WrapStyle: 2",
    "ScaledBorderAndShadow: yes",
    "",
    "[V4+ Styles]",
    "Format: Name, Fontname, Fontsize, PrimaryColour, SecondaryColour, OutlineColour, BackColour, Bold, Italic, Underline, StrikeOut, ScaleX, ScaleY, Spacing, Angle, BorderStyle, Outline, Shadow, Alignment, MarginL, MarginR, MarginV, Encoding",
    "Style: Brand,PingFang TC,25,&H00FFFFFF,&H00FFFFFF,&H00101820,&H95101820,-1,0,0,0,100,100,0,0,3,1,0,7,120,120,34,1",
    "Style: Stage,PingFang TC,31,&H00FFFFFF,&H00FFFFFF,&H00101820,&HBC101820,-1,0,0,0,100,100,0,0,3,1,0,9,120,120,36,1",
    "Style: Title,PingFang TC,74,&H00FFFFFF,&H00FFFFFF,&H00101820,&H00101820,-1,0,0,0,100,100,0,0,1,3,0,5,110,110,60,1",
    "Style: Subtitle,PingFang TC,43,&H00FFFFFF,&H00FFFFFF,&H00101820,&H00101820,-1,0,0,0,100,100,0,0,1,2,0,5,150,150,60,1",
    "Style: Boundary,PingFang TC,25,&H00D8E6EF,&H00FFFFFF,&H00101820,&H95101820,0,0,0,0,100,100,0,0,3,1,0,2,120,120,38,1",
    "Style: Event,PingFang TC,43,&H00FFFFFF,&H00FFFFFF,&H00101820,&HCD101820,-1,0,0,0,100,100,0,0,3,1,0,2,115,115,70,1",
    "Style: Danger,PingFang TC,47,&H006B6BFF,&H00FFFFFF,&H00101820,&HDE161C28,-1,0,0,0,100,100,0,0,3,2,0,5,130,130,80,1",
    "Style: Success,PingFang TC,47,&H0087D155,&H00FFFFFF,&H00101820,&HDE14251E,-1,0,0,0,100,100,0,0,3,2,0,5,130,130,80,1",
    "Style: Explain,PingFang TC,31,&H00FFFFFF,&H00FFFFFF,&H00101820,&HBD101820,-1,0,0,0,100,100,0,0,3,1,0,2,160,160,45,1",
    "Style: Label,PingFang TC,29,&H00FFFFFF,&H00FFFFFF,&H00101820,&HD0101820,-1,0,0,0,100,100,0,0,3,1,0,5,40,40,30,1",
    "Style: OutroTitle,PingFang TC,58,&H00FFFFFF,&H00FFFFFF,&H00101820,&H00101820,-1,0,0,0,100,100,0,0,1,3,0,5,120,120,45,1",
    "Style: OutroLine,PingFang TC,39,&H00FFFFFF,&H00FFFFFF,&H00101820,&H00101820,-1,0,0,0,100,100,0,0,1,2,0,5,130,130,50,1",
    "",
    "[Events]",
    "Format: Layer, Start, End, Style, Name, MarginL, MarginR, MarginV, Effect, Text",
]


def add(start: float, end: float, style: str, text: str, layer: int = 0) -> None:
    if end <= start:
        end = start + 0.25
    lines.append(
        f"Dialogue: {layer},{ass_time(start)},{ass_time(min(end, story_end))},{style},,0,0,0,,{safe_text(text)}"
    )


def add_positioned(start: float, end: float, style: str, x: int, y: int, text: str, layer: int = 2) -> None:
    lines.append(
        f"Dialogue: {layer},{ass_time(start)},{ass_time(min(end, story_end))},{style},,0,0,0,,"
        f"{{\\an5\\pos({x},{y})}}{safe_text(text)}"
    )


add(0.0, story_end, "Brand", "Flyto2｜醫院送藥安全閉環", 4)
add(0.0, intro, "Title", "這是一台醫院送藥機器人", 5)
add(1.8, intro, "Subtitle", "任務：幫 12 號病人送 A12 藥袋", 5)
add(0.0, intro, "Boundary", "真實 Gazebo 模擬畫面｜所有人物與藥品資料皆為合成測試", 5)

add(intro, obstacle_stop, "Stage", "步驟 1／4　AI 先挑一條仍然安全、看得到的路線", 3)
add_positioned(intro, obstacle_in, "Label", 520, 455, "送藥機器人\n↓")
add_positioned(intro, obstacle_in, "Label", 1450, 415, "護理站（目的地）\n↓")
add(intro, obstacle_in, "Event", "攝影機故障，AI 改走安全路線", 4)
add(intro, obstacle_in, "Explain", "原本路線無法可靠確認 → 改走橘線接紫線", 4)

add(obstacle_in, path_clear, "Stage", "步驟 2／4　行駛中持續檢查前方距離", 3)
add_positioned(obstacle_in, path_clear, "Label", 800, 420, "突然出現的障礙物\n↓")
add(obstacle_in, path_clear, "Danger", "有人擋路，機器人自己停下來", 4)
add(obstacle_stop, path_clear, "Explain", "安全規則：距離太近 = 速度立刻歸零", 4)

resume_explanation_end = min(destination, path_clear + 8.0)
add(path_clear, destination, "Stage", "步驟 2／4　確認前方安全後再繼續", 3)
add(path_clear, resume_explanation_end, "Success", "障礙移開，從剛才的位置繼續", 4)
add(resume_explanation_end, destination, "Event", "正在前往護理站", 4)
add(path_clear, destination, "Explain", "AI 已選路｜固定安全規則逐步執行", 4)

add(destination, event_time("item_rejected", driver_events, 23.4), "Stage", "步驟 3／4　抵達護理站，先確認藥袋", 3)
add(destination, event_time("item_rejected", driver_events, 23.4), "Event", "抵達後還不能開箱：先驗藥，再驗病人", 4)

handoff_cards = [
    ("item_rejected", 2.8, "Danger", "藥袋 B13　✕\n不是任務指定的 A12｜藥箱保持上鎖"),
    ("item_verified", 2.8, "Success", "藥袋 A12　✓\n第一關通過｜藥箱仍保持上鎖"),
    ("recipient_rejected", 2.8, "Danger", "病人 13　✕\n不是任務指定的 12 號病人｜藥箱保持上鎖"),
    ("recipient_verified", 2.8, "Success", "12 號病人　✓\n第二關通過｜現在才允許解鎖"),
    ("container_unlocked", 2.0, "Success", "藥袋正確 + 病人正確 = 才能解鎖"),
    ("handoff_completed", None, "Success", "送藥完成　✓\n錯的藥、不對的人，都不會開鎖"),
]
card_cursor = event_time("item_rejected", driver_events, 23.4)
for start_kind, duration, style, copy in handoff_cards:
    if start_kind not in driver_events:
        continue
    evidence_start = timeline(float(driver_events[start_kind]["at_seconds"]))
    start = max(card_cursor, evidence_start)
    end = min(outro_start, start + duration) if duration else outro_start
    add(start, end, style, copy, 5)
    card_cursor = end

handoff_start = event_time("item_rejected", driver_events, 23.4)
unlock = event_time("container_unlocked", driver_events, 26.9)
add(handoff_start, unlock, "Stage", "步驟 3／4　錯一項就不開鎖", 3)
add(unlock, outro_start, "Stage", "步驟 4／4　兩項都正確，安全交付", 3)

add(outro_start, story_end, "OutroTitle", "看懂 Flyto2 的一句話", 6)
add(outro_start + 1.8, story_end, "OutroLine", "AI 負責理解任務與選路", 6)
add(outro_start + 4.2, story_end, "OutroLine", "安全規則負責決定能不能執行", 6)
add(outro_start + 6.8, story_end, "Success", "錯藥 ✕　錯人 ✕　兩者正確才解鎖 ✓", 6)
add(outro_start, story_end, "Boundary", "這是 Gazebo 模擬，不是實體醫院｜不是生成式影片", 6)

subtitle_output.write_text("\n".join(lines) + "\n", encoding="utf-8")

narration = [
    (0.6, "這是一台醫院送藥機器人。它的任務，是把 A 十二藥袋，安全交給十二號病人。"),
    (8.3, "先看 AI 做什麼。它理解任務、比較路線。出發前發現 B 攝影機故障，就改走仍然看得到、也能驗證的路線。"),
    (18.0, "路上突然有人擋住。安全規則立刻讓機器人停下來。這個停止不靠 AI 猜測，也不能被 AI 略過。"),
    (26.5, "障礙移開後，機器人從原本進度繼續，不會跳過前面的安全步驟。"),
    (40.0, "它沿著重新選好的路線前往護理站。AI 負責選路，實際移動仍由固定規則逐步執行。"),
    (64.5, "抵達後還不能打開藥箱。系統先確認藥袋，再確認病人。"),
    (70.0, "藥袋 B 十三錯誤，拒絕並保持上鎖；換成 A 十二，第一關通過。"),
    (75.8, "十三號病人不符合任務，仍不開鎖；確認十二號病人後，第二關通過。"),
    (82.2, "兩關都正確，藥箱才解鎖，送藥完成。"),
    (outro_start + 1.3, "重點是，AI 負責理解任務與選路；安全規則負責決定能不能執行。這是 Gazebo 模擬，不是實體醫院，也不是生成式影片。"),
]
narration_output.write_text(
    "".join(f"{start:.3f}\t{text}\n" for start, text in narration), encoding="utf-8"
)
PY

render_assets="$(mktemp -d "${run_directory}/.flyto-story-render.XXXXXX")"
mkdir -p "${render_assets}/fonts"
cp "${logo_file}" "${render_assets}/flyto2-logo.png"
cp "${font_file}" "${render_assets}/fonts/cjk-font.ttf"
cp "${subtitle_file}" "${render_assets}/hospital-story.ass"

if [[ "${narration_enabled}" != "0" ]]; then
  audio_inputs=()
  audio_filters=""
  audio_mix_inputs=""
  narration_count=0
  while IFS=$'\t' read -r narration_start narration_text; do
    clip_file="${render_assets}/narration-${narration_count}.aiff"
    say -v "${narration_voice}" -r "${narration_rate}" \
      -o "${clip_file}" "${narration_text}"
    narration_delay_ms="$(python3 -c 'import sys; print(round(float(sys.argv[1]) * 1000))' "${narration_start}")"
    audio_inputs+=( -i "/assets/$(basename "${clip_file}")" )
    audio_filters="${audio_filters}[${narration_count}:a]adelay=${narration_delay_ms}:all=1[n${narration_count}];"
    audio_mix_inputs="${audio_mix_inputs}[n${narration_count}]"
    narration_count=$((narration_count + 1))
  done < "${narration_schedule}"
  audio_filters="${audio_filters}${audio_mix_inputs}amix=inputs=${narration_count}:normalize=0,alimiter=limit=0.95,apad=whole_dur=${story_duration}[narration]"
  docker run --rm \
    -v "${render_assets}:/assets" \
    "${image_name}" \
    ffmpeg -hide_banner -loglevel error -y \
      "${audio_inputs[@]}" \
      -filter_complex "${audio_filters}" \
      -map "[narration]" -t "${story_duration}" \
      -c:a aac -b:a 160k -ar 48000 -ac 2 /assets/story-narration.m4a
else
  docker run --rm \
    -v "${render_assets}:/assets" \
    "${image_name}" \
    ffmpeg -hide_banner -loglevel error -y \
      -f lavfi -i anullsrc=r=48000:cl=stereo -t "${story_duration}" \
      -c:a aac -b:a 128k -ar 48000 -ac 2 /assets/story-narration.m4a
fi

docker run --rm \
  -e LANG=C.utf8 \
  -e LC_ALL=C.utf8 \
  -v "${run_directory}:/evidence" \
  -v "${render_assets}:/assets:ro" \
  "${image_name}" \
  ffmpeg -hide_banner -loglevel error -y \
    -ss "${trim_start}" \
    -i /evidence/gazebo-gui.mp4 \
    -i /assets/flyto2-logo.png \
    -i /assets/story-narration.m4a \
    -filter_complex \
      "[0:v]crop=${main_width}:${main_height}:${main_x}:${main_y},scale=1920:1080,tpad=start_mode=clone:start_duration=${intro_duration}:stop_mode=clone:stop_duration=${outro_duration},setpts=PTS-STARTPTS,drawbox=x=0:y=0:w=iw:h=ih:color=0x071018@0.72:t=fill:enable='lt(t,${intro_duration})+gte(t,${outro_start})'[scene];[1:v]scale=78:-1,format=rgba,loop=loop=-1:size=1:start=0,setpts=N/30/TB[logo];[scene][logo]overlay=34:27:shortest=1,subtitles=/assets/hospital-story.ass:fontsdir=/assets/fonts[out]" \
    -map "[out]" -map 2:a \
    -t "${story_duration}" -r 30 \
    -c:v libx264 -preset medium -crf 18 -pix_fmt yuv420p \
    -c:a aac -b:a 160k -ar 48000 -ac 2 \
    -movflags +faststart \
    /evidence/flyto2-hospital-story.mp4

docker run --rm \
  -v "${run_directory}:/evidence" \
  "${image_name}" \
  ffprobe -v error -show_entries \
    format=duration,size:stream=codec_name,codec_type,width,height,avg_frame_rate,nb_frames,sample_rate,channels \
    -of json /evidence/flyto2-hospital-story.mp4 \
    > "${run_directory}/hospital-story-video-probe.json"

shasum -a 256 \
  "${output_video}" \
  "${planning_session}" \
  "${mission_result}" \
  "${driver_manifest}" \
  > "${run_directory}/hospital-story-video.sha256"

echo "Flyto2 layperson hospital story video: ${output_video}"
