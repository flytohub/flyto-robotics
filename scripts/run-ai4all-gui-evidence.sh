#!/usr/bin/env bash
set -euo pipefail

if (($# < 1 || $# > 2)); then
  echo "usage: $0 <validated-showcase-run-directory> [capture-output-directory]" >&2
  exit 2
fi

script_directory="$(dirname "${BASH_SOURCE[0]}")"
repository_root="$(realpath "${script_directory}/..")"
source_run="$(realpath "$1")"
capture_run="${2:-${source_run}-gazebo-gui-$(date -u +%Y%m%dT%H%M%SZ)}"
capture_run="$(python3 -c 'from pathlib import Path; import sys; print(Path(sys.argv[1]).resolve())' "${capture_run}")"
image_name="${FLYTO_ROBOTICS_GUI_IMAGE:-flyto-robotics:gui-capture}"
video_fps="${FLYTO_ROBOTICS_GUI_FPS:-30}"

if [[ "${capture_run}" == "${source_run}" ]]; then
  echo "capture output must differ from the validated source run" >&2
  exit 2
fi
if [[ ! "${video_fps}" =~ ^[1-9][0-9]*$ ]] || ((video_fps > 60)); then
  echo "FLYTO_ROBOTICS_GUI_FPS must be an integer between 1 and 60" >&2
  exit 2
fi

required_source_files=(
  planning-session.json
  validated-plan.json
  mission-result.json
  images/driver-manifest.json
)
for relative_path in "${required_source_files[@]}"; do
  if [[ ! -f "${source_run}/${relative_path}" ]]; then
    echo "validated source run is missing ${relative_path}: ${source_run}" >&2
    exit 2
  fi
done
if [[ -d "${capture_run}" ]] && find "${capture_run}" -mindepth 1 -print -quit | grep -q .; then
  echo "capture output already exists and is not empty: ${capture_run}" >&2
  exit 2
fi
if ! docker image inspect "${image_name}" >/dev/null 2>&1; then
  echo "missing GUI capture image ${image_name}; it must contain ROS 2 Jazzy, Gazebo Harmonic, Xvfb, openbox, xdotool and ffmpeg" >&2
  exit 2
fi

guarded_handoff_enabled="$(python3 -c \
  'import json, sys; d=json.load(open(sys.argv[1], encoding="utf-8")); print("true" if d.get("guarded_handoff", {}).get("enabled") is True else "false")' \
  "${source_run}/images/driver-manifest.json")"
showcase_launch="ai4all_showcase.launch.py"
lab_scenario="scenarios/gazebo/ai4all-branching.json"
if [[ "${guarded_handoff_enabled}" == "true" ]]; then
  showcase_launch="ai4all_medication_showcase.launch.py"
  lab_scenario="scenarios/gazebo/ai4all-medication-handoff.json"
fi

mkdir -p \
  "${capture_run}/facility" \
  "${capture_run}/frames/active-camera" \
  "${capture_run}/frames/overhead" \
  "${capture_run}/images"
cp "${source_run}/planning-session.json" "${capture_run}/planning-session.json"
cp "${source_run}/validated-plan.json" "${capture_run}/validated-plan.json"

relative_capture_run="$(python3 -c \
  'from pathlib import Path; import sys; print(Path(sys.argv[1]).resolve().relative_to(Path(sys.argv[2]).resolve()))' \
  "${capture_run}" "${repository_root}")"
approval_key_material="$(python3 -c 'import secrets; print(secrets.token_hex(32))')"
qr_key_material="$(python3 -c 'import secrets; print(secrets.token_hex(32))')"
container_name="flyto-gazebo-gui-$(basename "${capture_run}" | tr -cd 'A-Za-z0-9_.-' | cut -c1-48)"

set +e
docker run --rm \
  --name "${container_name}" \
  --shm-size=2g \
  -e "FLYTO_ROBOTICS_APPROVAL_SECRET=${approval_key_material}" \
  -e "FLYTO_ROBOTICS_QR_SECRET=${qr_key_material}" \
  -e QT_X11_NO_MITSHM=1 \
  -e LIBGL_ALWAYS_SOFTWARE=1 \
  -e MESA_GL_VERSION_OVERRIDE=3.3 \
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

    export DISPLAY=:99
    export XDG_RUNTIME_DIR=/tmp/flyto-gui-runtime
    mkdir -p \"\${XDG_RUNTIME_DIR}\"
    chmod 700 \"\${XDG_RUNTIME_DIR}\"
    Xvfb :99 -screen 0 1920x1080x24 -nolisten tcp \
      > /workspace/${relative_capture_run}/xvfb.log 2>&1 &
    xvfb_pid=\$!
    openbox > /workspace/${relative_capture_run}/openbox.log 2>&1 &
    openbox_pid=\$!
    sleep 1

    python3 /workspace/scripts/ai4all-live-evidence-panel.py \
      --planning-session /workspace/${relative_capture_run}/planning-session.json \
      --driver-manifest /workspace/${relative_capture_run}/images/driver-manifest.json \
      --panel-text /workspace/${relative_capture_run}/live-evidence-panel.txt \
      --panel-image /workspace/${relative_capture_run}/live-evidence-panel.png \
      --font-file /usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc \
      --font-index 3 \
      --state-output /workspace/${relative_capture_run}/live-evidence-panel-state.json \
      --events-output /workspace/${relative_capture_run}/live-evidence-panel-events.jsonl \
      --ready-file /workspace/${relative_capture_run}/live-evidence-panel-ready.txt \
      --stop-file /workspace/${relative_capture_run}/live-evidence-panel-stop.txt \
      > /workspace/${relative_capture_run}/live-evidence-panel.log 2>&1 &
    panel_writer_pid=\$!
    for attempt in \$(seq 1 30); do
      [[ -s /workspace/${relative_capture_run}/live-evidence-panel-ready.txt ]] && break
      sleep 0.2
    done
    if [[ ! -s /workspace/${relative_capture_run}/live-evidence-panel-ready.txt ]]; then
      echo 'Live evidence panel writer did not become ready' >&2
      exit 4
    fi

    (
      while [[ ! -e /workspace/${relative_capture_run}/live-evidence-panel-stop.txt ]]; do
        if [[ -s /workspace/${relative_capture_run}/live-evidence-panel.png ]]; then
          /bin/cat /workspace/${relative_capture_run}/live-evidence-panel.png
        fi
        sleep 0.2
      done
    ) | ffplay -hide_banner -loglevel error -nostats -an \
      -fflags nobuffer -flags low_delay -framedrop \
      -probesize 32 -analyzeduration 0 \
      -f image2pipe -framerate 5 -vcodec png -i - \
      -window_title 'Flyto2 Live Evidence' -noborder \
      > /workspace/${relative_capture_run}/ffplay-evidence-panel.log 2>&1 &
    panel_window_pid=\$!
    panel_window_id=''
    for attempt in \$(seq 1 30); do
      panel_window_id=\$(xdotool search --name 'Flyto2 Live Evidence' 2>/dev/null | tail -n 1 || true)
      [[ -n \"\${panel_window_id}\" ]] && break
      sleep 0.2
    done
    if [[ -z \"\${panel_window_id}\" ]]; then
      echo 'Live evidence panel window did not become visible' >&2
      exit 4
    fi
    python3 -c 'import time; print(time.time())' \
      > /workspace/${relative_capture_run}/live-panel-window-visible-epoch.txt
    xdotool windowmap \"\${panel_window_id}\" || true
    xdotool windowsize \"\${panel_window_id}\" 520 1040 || true
    xdotool windowmove \"\${panel_window_id}\" 1400 20 || true
    xdotool windowraise \"\${panel_window_id}\" || true
    sleep 1
    xdotool getwindowgeometry --shell \"\${panel_window_id}\" \
      > /workspace/${relative_capture_run}/live-panel-window-geometry.env

    capture_started_epoch=\$(python3 -c 'import time; print(time.time())')
    printf '%s\\n' \"\${capture_started_epoch}\" \
      > /workspace/${relative_capture_run}/capture-started-epoch.txt
    ffmpeg -hide_banner -loglevel warning -y \
      -thread_queue_size 1024 \
      -f x11grab -draw_mouse 0 -framerate ${video_fps} \
      -video_size 1920x1080 -i :99.0 \
      -c:v libx264 -preset veryfast -crf 18 -pix_fmt yuv420p \
      -movflags +faststart \
      /workspace/${relative_capture_run}/gazebo-gui.mp4 \
      > /workspace/${relative_capture_run}/ffmpeg-gui.log 2>&1 &
    recorder_pid=\$!

    (
      for attempt in \$(seq 1 45); do
        window_id=\$(xdotool search --name 'Gazebo' 2>/dev/null | tail -n 1 || true)
        if [[ -n \"\${window_id}\" ]]; then
          python3 -c 'import time; print(time.time())' \
            > /workspace/${relative_capture_run}/gazebo-window-visible-epoch.txt
          xdotool windowmap \"\${window_id}\" || true
          xdotool windowactivate --sync \"\${window_id}\" || true
          xdotool windowsize \"\${window_id}\" 1380 1040 || true
          xdotool windowmove \"\${window_id}\" 0 20 || true
          xdotool windowraise \"\${window_id}\" || true
          sleep 2
          xdotool getwindowgeometry --shell \"\${window_id}\" \
            > /workspace/${relative_capture_run}/gazebo-window-geometry.env
          exit 0
        fi
        sleep 1
      done
      exit 1
    ) &
    framing_pid=\$!

    set +e
    timeout --signal=TERM --kill-after=10s 180s \
      ros2 launch flyto_robotics ${showcase_launch} \
        headless:=false \
        output_dir:=/workspace/${relative_capture_run} \
        plan_file:=/workspace/${relative_capture_run}/validated-plan.json \
        planning_session_file:=/workspace/${relative_capture_run}/planning-session.json \
      > /workspace/${relative_capture_run}/gazebo-console.log 2>&1
    launch_status=\$?
    set -e
    wait \"\${framing_pid}\" || framing_status=\$?
    touch /workspace/${relative_capture_run}/live-evidence-panel-stop.txt
    wait \"\${panel_writer_pid}\" || panel_writer_status=\$?
    sleep 3
    kill -INT \"\${recorder_pid}\" 2>/dev/null || true
    wait \"\${recorder_pid}\" || true
    kill \"\${panel_window_pid}\" 2>/dev/null || true
    kill \"\${openbox_pid}\" \"\${xvfb_pid}\" 2>/dev/null || true

    required_files=(
      /workspace/${relative_capture_run}/gazebo-gui.mp4
      /workspace/${relative_capture_run}/mission-result.json
      /workspace/${relative_capture_run}/images/driver-manifest.json
      /workspace/${relative_capture_run}/facility/showcase-evidence.json
      /workspace/${relative_capture_run}/planning-session.json
      /workspace/${relative_capture_run}/validated-plan.json
      /workspace/${relative_capture_run}/live-evidence-panel.txt
      /workspace/${relative_capture_run}/live-evidence-panel.png
      /workspace/${relative_capture_run}/live-evidence-panel-state.json
      /workspace/${relative_capture_run}/live-evidence-panel-events.jsonl
      /workspace/${relative_capture_run}/live-panel-window-geometry.env
    )
    for required_file in \"\${required_files[@]}\"; do
      if [[ ! -s \"\${required_file}\" ]]; then
        echo \"GUI Gazebo run did not produce \${required_file}\" >&2
        exit 3
      fi
    done
    if [[ ! -s /workspace/${relative_capture_run}/gazebo-window-visible-epoch.txt ]]; then
      echo 'Gazebo GUI window never became visible' >&2
      exit \${framing_status:-4}
    fi
    if [[ ! -s /workspace/${relative_capture_run}/gazebo-window-geometry.env ]]; then
      echo 'Gazebo GUI window geometry was not captured' >&2
      exit \${framing_status:-4}
    fi

    python3 -m flyto_robotics.cli evaluate-lab \
      --scenario /workspace/${lab_scenario} \
      --result /workspace/${relative_capture_run}/mission-result.json \
      --evidence-dir /workspace/${relative_capture_run}/images \
      --report /workspace/${relative_capture_run}/lab-report.json \
      --markdown /workspace/${relative_capture_run}/lab-report.md \
      --junit /workspace/${relative_capture_run}/lab-junit.xml
    python3 -m flyto_robotics.showcase_evidence \
      --showcase /workspace/${relative_capture_run}/facility/showcase-evidence.json \
      --mission /workspace/${relative_capture_run}/mission-result.json \
      --driver /workspace/${relative_capture_run}/images/driver-manifest.json \
      --report /workspace/${relative_capture_run}/showcase-report.json \
      --markdown /workspace/${relative_capture_run}/showcase-report.md

    ffprobe -v error -show_entries \
      format=duration,size:stream=codec_name,width,height,avg_frame_rate,nb_frames \
      -of json /workspace/${relative_capture_run}/gazebo-gui.mp4 \
      > /workspace/${relative_capture_run}/gazebo-gui-video-probe.json
    sha256sum \
      /workspace/${relative_capture_run}/gazebo-gui.mp4 \
      /workspace/${relative_capture_run}/planning-session.json \
      /workspace/${relative_capture_run}/validated-plan.json \
      /workspace/${relative_capture_run}/mission-result.json \
      /workspace/${relative_capture_run}/images/driver-manifest.json \
      /workspace/${relative_capture_run}/live-evidence-panel-state.json \
      /workspace/${relative_capture_run}/live-evidence-panel-events.jsonl \
      /workspace/${relative_capture_run}/live-evidence-panel.png \
      > /workspace/${relative_capture_run}/gui-evidence.sha256
    if ((launch_status != 0)); then
      echo \"Gazebo launch exited with status \${launch_status}\" >&2
      exit 3
    fi
    exit 0
  "
status=$?
set -e
approval_key_material=""
qr_key_material=""

if ((status == 0)); then
  python3 - "${capture_run}" <<'PY'
import datetime as dt
import json
import re
import sys
from pathlib import Path

output = Path(sys.argv[1])
capture_started = float((output / "capture-started-epoch.txt").read_text().strip())
window_visible = float(
    (output / "gazebo-window-visible-epoch.txt").read_text().strip()
)
panel_visible = float(
    (output / "live-panel-window-visible-epoch.txt").read_text().strip()
)
mission = json.loads(
    (output / "mission-result.json").read_text(encoding="utf-8")
)
generated = dt.datetime.fromisoformat(
    mission["generated_at"].replace("Z", "+00:00")
).timestamp()
elapsed = float(mission["elapsed_seconds"])
console = (output / "gazebo-console.log").read_text(
    encoding="utf-8", errors="replace"
)
odometry_match = re.search(
    r"\[(\d+\.\d+)\].*first odometry pose", console
)
first_odometry_epoch = (
    float(odometry_match.group(1)) if odometry_match else generated - elapsed
)
sim_anchor_seconds = 0.1
wall_span = max(0.001, generated - first_odometry_epoch)
simulation_time_scale = wall_span / max(0.001, elapsed - sim_anchor_seconds)
mission_started = first_odometry_epoch - (
    sim_anchor_seconds * simulation_time_scale
)
metadata = {
    "contract_version": "flyto.robotics.gui-capture.v2",
    "capture_started_epoch": capture_started,
    "gazebo_window_visible_seconds": round(
        max(0.0, window_visible - capture_started), 3
    ),
    "mission_offset_seconds": round(
        max(0.0, mission_started - capture_started), 3
    ),
    "simulation_time_scale": round(simulation_time_scale, 6),
    "live_evidence_panel": {
        "visible_seconds": round(max(0.0, panel_visible - capture_started), 3),
        "geometry_file": "live-panel-window-geometry.env",
        "state_file": "live-evidence-panel-state.json",
        "events_file": "live-evidence-panel-events.jsonl",
        "image_file": "live-evidence-panel.png",
        "planning_source": "planning-session.json",
        "runtime_source": "images/driver-manifest.json",
    },
    "source_plan": "validated-plan.json",
    "source_planning_session": "planning-session.json",
}
(output / "gui-capture-metadata.json").write_text(
    json.dumps(metadata, ensure_ascii=False, indent=2) + "\n",
    encoding="utf-8",
)
PY
fi

echo "Flyto2 real Gazebo GUI evidence: ${capture_run}"
exit "${status}"
