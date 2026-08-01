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
driver_manifest="${run_directory}/images/driver-manifest.json"
mission_result="${run_directory}/mission-result.json"
planning_session="${run_directory}/planning-session.json"
validated_plan="${run_directory}/validated-plan.json"
capture_metadata="${run_directory}/gui-capture-metadata.json"
window_geometry="${run_directory}/gazebo-window-geometry.env"
subtitle_file="${run_directory}/verification-events.ass"
output_video="${run_directory}/flyto2-gazebo-verification.mp4"
logo_file="${FLYTO2_LOGO_FILE:-}"
font_file="${FLYTO2_CJK_FONT_FILE:-/System/Library/Fonts/PingFang.ttc}"
image_name="${FLYTO_ROBOTICS_IMAGE:-flyto-robotics:jazzy-harmonic}"
render_assets=""

cleanup() {
  if [[ -n "${render_assets}" && -d "${render_assets}" ]]; then
    rm -rf -- "${render_assets}"
  fi
}
trap cleanup EXIT

required_files=(
  "${input_video}"
  "${driver_manifest}"
  "${mission_result}"
  "${planning_session}"
  "${validated_plan}"
  "${capture_metadata}"
  "${window_geometry}"
)
for required_file in "${required_files[@]}"; do
  if [[ ! -s "${required_file}" ]]; then
    echo "missing verification evidence: ${required_file}" >&2
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

read -r trim_start render_duration <<< "$(python3 -c \
  'import json, sys; m=json.load(open(sys.argv[1], encoding="utf-8")); c=json.load(open(sys.argv[2], encoding="utf-8")); trim=float(c["mission_offset_seconds"]); duration=float(m["elapsed_seconds"])*float(c.get("simulation_time_scale", 1.0))+3.0; print("{:.3f} {:.3f}".format(trim, duration))' \
  "${mission_result}" "${capture_metadata}")"

read -r window_x window_y window_width window_height \
  main_x main_y main_width main_height <<< "$(python3 - "${window_geometry}" <<'PY'
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
print(x, y, width // 2 * 2, height // 2 * 2, main_x, main_y, main_width, main_height)
PY
)"

python3 - "${planning_session}" "${validated_plan}" "${mission_result}" \
  "${driver_manifest}" "${capture_metadata}" "${subtitle_file}" "${render_duration}" <<'PY'
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
plan = json.loads(Path(sys.argv[2]).read_text(encoding="utf-8"))
mission = json.loads(Path(sys.argv[3]).read_text(encoding="utf-8"))
driver = json.loads(Path(sys.argv[4]).read_text(encoding="utf-8"))
capture = json.loads(Path(sys.argv[5]).read_text(encoding="utf-8"))
output = Path(sys.argv[6])
render_end = float(sys.argv[7])
offset = 0.0
time_scale = float(capture.get("simulation_time_scale", 1.0))

def timeline(simulation_seconds: float) -> float:
    return offset + simulation_seconds * time_scale

rounds = planning["rounds"]
first = rounds[0]
final = rounds[-1]
first_route = first["response"]["attestation"]["selected_route_id"]
final_route = final["response"]["attestation"]["selected_route_id"]
initial_candidates = first["route_evaluation"]["candidate_count"]
final_candidates = final["route_evaluation"]["candidate_count"]
excluded = final["route_evaluation"].get("excluded_count", 0)
attestation = final["response"]["attestation"]
model = attestation["model"]
plan_hash = attestation["plan_sha256"][:12]
goal = safe_text(plan["goal"])

mission_events = {event["kind"]: event for event in mission["events"]}
driver_events = {}
for event in driver.get("actions", []):
    driver_events.setdefault(event["kind"], event)

def event_time(kind: str, source: dict[str, dict], fallback: float) -> float:
    event = source.get(kind)
    return timeline(float(event["at_seconds"]) if event else fallback)

obstacle_in = event_time("fault_injection", driver_events, 3.0)
obstacle_stop = event_time("obstacle_stop", mission_events, 3.3)
path_clear = event_time("path_clear", mission_events, 6.3)
merge = next(
    (timeline(float(event["at_seconds"])) for event in mission["events"]
     if event.get("kind") == "primitive_completed" and event.get("step_id") == "step-2"),
    timeline(14.3),
)
destination = next(
    (timeline(float(event["at_seconds"])) for event in mission["events"]
     if event.get("kind") == "primitive_completed" and event.get("step_id") == "step-4"),
    timeline(22.0),
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
    "Style: Meta,PingFang TC,22,&H00FFFFFF,&H00FFFFFF,&H001B2530,&H9A101820,-1,0,0,0,100,100,0,0,3,1,0,7,124,440,34,1",
    "Style: Goal,PingFang TC,26,&H00FFFFFF,&H00FFFFFF,&H00131B24,&H96101820,-1,0,0,0,100,100,0,0,3,1,0,7,124,440,105,1",
    "Style: Event,PingFang TC,31,&H00FFFFFF,&H00FFFFFF,&H00101820,&H92101820,-1,0,0,0,100,100,0,0,3,1,0,2,140,140,82,1",
    "Style: Alert,PingFang TC,32,&H00FFFFFF,&H00FFFFFF,&H00272A31,&H9A222A33,-1,0,0,0,100,100,0,0,3,1,0,2,140,140,82,1",
    "Style: Success,PingFang TC,31,&H00F3FFF6,&H00FFFFFF,&H00122820,&H96122820,-1,0,0,0,100,100,0,0,3,1,0,2,140,140,82,1",
    "Style: ProofLabel,PingFang TC,18,&H00FFFFFF,&H00FFFFFF,&H00101820,&H9A101820,-1,0,0,0,100,100,0,0,3,1,0,9,1400,36,35,1",
    "Style: EvidenceLog,PingFang TC,19,&H00FFFFFF,&H00FFFFFF,&H00101820,&HAD101820,0,0,0,0,100,100,0,0,3,1,0,9,1390,36,344,1",
    "",
    "[Events]",
    "Format: Layer, Start, End, Style, Name, MarginL, MarginR, MarginV, Effect, Text",
]

def add(start: float, end: float, style: str, text: str, layer: int = 0) -> None:
    if end <= start:
        end = start + 0.25
    lines.append(
        f"Dialogue: {layer},{ass_time(start)},{ass_time(min(end, render_end))},{style},,0,0,0,,{safe_text(text)}"
    )


def add_log(start: float, end: float, title: str, rows: list[str]) -> None:
    content = "\\N".join(safe_text(row) for row in (title, *rows))
    if end <= start:
        end = start + 0.25
    lines.append(
        f"Dialogue: 3,{ass_time(start)},{ass_time(min(end, render_end))},EvidenceLog,,0,0,0,,{content}"
    )

meta = (
    f"真實 Gazebo GUI｜{model}｜候選 {initial_candidates}→{final_candidates}｜"
    f"{first_route} ×  {final_route} ✓｜plan {plan_hash}"
)
add(0.0, render_end, "Meta", meta, 2)
add(0.0, render_end, "ProofLabel", "原始未裁切 Gazebo GUI", 4)
add(0.0, min(obstacle_in, timeline(3.0)), "Goal", f"自然語言任務：{goal}", 1)
add(
    max(0.0, min(offset, obstacle_in - 1.8 * time_scale)),
    obstacle_in,
    "Event",
    f"Flyto AI 先從 {initial_candidates} 條候選選 {first_route}；攝影機 B 失效後排除 {excluded} 條，再驗證 {final_route}",
)
add(obstacle_in, obstacle_stop + 0.45 * time_scale, "Alert", "動態障礙進入實際路徑｜LiDAR 即時感知", 1)
add(obstacle_stop, path_clear, "Alert", "安全閉環：距離低於門檻 → 速度歸零 → 保持停止", 2)
add(path_clear, min(merge, path_clear + 3.6 * time_scale), "Success", "障礙移除且距離恢復 → 同一原子流程繼續，不跳步", 1)
add(min(merge, path_clear + 3.6 * time_scale), destination, "Event", f"確定性 Executor：沿 {final_route} 原子位置逐步執行 → 紫區護理站", 1)

resource_change = planning["resource_change"]
final_validation = final["response"]["attestation"]["attempts"][-1]["validation"]["passed"]
add_log(
    0.0,
    obstacle_in,
    "實際規劃 LOG · planning-session.json",
    [
        f"mode={planning['planning_mode']} model={model}",
        f"round-1 candidates={initial_candidates} selected={first_route}",
        f"{resource_change['resource_id']} healthy=false excluded={excluded}",
        f"round-2 selected={final_route} validation={'PASS' if final_validation else 'FAIL'}",
    ],
)

obstacle_observation = next(
    (
        event
        for event in driver.get("actions", [])
        if event.get("kind") == "image_captured" and event.get("detail") == "obstacle"
    ),
    {},
)
minimum_range = obstacle_observation.get("minimum_range")
range_copy = "minimum_range=unavailable"
if isinstance(minimum_range, (int, float)):
    range_copy = f"minimum_range={minimum_range:.3f} m"
add_log(
    obstacle_in,
    path_clear,
    "實際安全 LOG · mission-result.json + driver-manifest.json",
    [
        f"t={mission_events['obstacle_stop']['at_seconds']:.3f} event=obstacle_stop",
        range_copy,
        f"step_id={mission_events['obstacle_stop']['step_id']} state=hold_stop",
        f"safety_stop_count={mission.get('safety_stop_count', 0)}",
    ],
)
add_log(
    path_clear,
    destination,
    "實際執行 LOG · mission-result.json",
    [
        f"t={mission_events['path_clear']['at_seconds']:.3f} event=path_clear",
        f"resume_step={mission_events['path_clear']['step_id']}",
        f"route={final_route}",
        "executor=deterministic checkpoint=preserved",
    ],
)

guarded = driver.get("guarded_handoff", {}).get("enabled") is True
if guarded:
    handoff_copy = [
        ("item_rejected", "Alert", "掃描 B13 ≠ A12 → 錯誤物品拒絕，箱體保持上鎖"),
        ("item_verified", "Success", "掃描 A12 通過 → 只開放 verify_item checkpoint"),
        ("checkpoint_resumed", "Event", "人員從已驗證 checkpoint 恢復 → 進入收件者驗證"),
        ("recipient_rejected", "Alert", "patient-13 ≠ patient-12 → 錯誤收件者拒絕，仍保持上鎖"),
        ("recipient_verified", "Success", "patient-12 驗證通過 → 解鎖條件成立"),
        ("container_unlocked", "Success", "所有前置條件都成立 → 才執行 unlock_container"),
        ("handoff_completed", "Success", "交付完成 → 發布核准 → 任務安全停止"),
    ]
    timed = []
    for kind, style, text in handoff_copy:
        if kind in driver_events:
            timed.append((timeline(float(driver_events[kind]["at_seconds"])), style, text))
    for index, (start, style, text) in enumerate(timed):
        end = timed[index + 1][0] - 0.05 if index + 1 < len(timed) else render_end
        add(start, end, style, text, 2)
        kind = handoff_copy[index][0]
        evidence = driver_events[kind]
        rows = [
            f"t={evidence['at_seconds']:.2f} event={kind}",
            f"state={evidence.get('handoff_state', 'completed')}",
        ]
        if evidence.get("actual") is not None:
            rows.append(f"actual={evidence['actual']} expected={evidence.get('expected', 'policy-match')}")
        rows.append(f"container_locked={str(evidence.get('container_locked', False)).lower()}")
        add_log(start, end, "實際交付 LOG · driver-manifest.json", rows)
else:
    approval = event_time("human_approval_requested", mission_events, float(mission["elapsed_seconds"]) - 1.0)
    completed = event_time("mission_completed", mission_events, float(mission["elapsed_seconds"]))
    add(approval, completed, "Event", "抵達後請求人員核准；同一 nonce 重播會被拒絕", 2)
    add(completed, render_end, "Success", "核准、重播防護與安全停止完成", 2)

output.write_text("\n".join(lines) + "\n", encoding="utf-8")
PY

render_assets="$(mktemp -d "${run_directory}/.flyto-verification-render.XXXXXX")"
mkdir -p "${render_assets}/fonts"
cp "${logo_file}" "${render_assets}/flyto2-logo.png"
cp "${font_file}" "${render_assets}/fonts/cjk-font.ttf"
cp "${subtitle_file}" "${render_assets}/verification-events.ass"

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
    -filter_complex \
      "[0:v]split=2[mainraw][proofraw];[mainraw]crop=${main_width}:${main_height}:${main_x}:${main_y},scale=1920:1080,tpad=stop_mode=clone:stop_duration=4,setpts=PTS-STARTPTS[scene];[proofraw]crop=${window_width}:${window_height}:${window_x}:${window_y},scale=360:-2,format=rgba[proof];[scene][proof]overlay=W-w-24:24:shortest=1[verified];[1:v]scale=70:-1,format=rgba,loop=loop=-1:size=1:start=0,setpts=N/30/TB[logo];[verified][logo]overlay=36:28:shortest=1,subtitles=/assets/verification-events.ass:fontsdir=/assets/fonts[out]" \
    -map "[out]" \
    -t "${render_duration}" \
    -r 30 \
    -c:v libx264 -preset medium -crf 18 -pix_fmt yuv420p \
    -movflags +faststart \
    /evidence/flyto2-gazebo-verification.mp4

docker run --rm \
  -v "${run_directory}:/evidence" \
  "${image_name}" \
  ffprobe -v error -show_entries \
    format=duration,size:stream=codec_name,width,height,avg_frame_rate,nb_frames \
    -of json /evidence/flyto2-gazebo-verification.mp4 \
    > "${run_directory}/verification-video-probe.json"

shasum -a 256 \
  "${output_video}" \
  "${planning_session}" \
  "${validated_plan}" \
  "${mission_result}" \
  "${driver_manifest}" \
  > "${run_directory}/verification-video.sha256"

echo "Flyto2 continuous Gazebo verification video: ${output_video}"
