#!/usr/bin/env bash
set -euo pipefail

repository_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
image_name="${FLYTO_ROBOTICS_IMAGE:-flyto-robotics:jazzy-harmonic}"
run_id="${FLYTO_ROBOTICS_BRIDGE_SOAK_RUN_ID:-$(date -u +%Y%m%dT%H%M%SZ)}"
cycles="${FLYTO_ROBOTICS_BRIDGE_SOAK_CYCLES:-100}"
soak_mode="${FLYTO_ROBOTICS_BRIDGE_SOAK_MODE:-steady}"
domain_id="${FLYTO_ROBOTICS_BRIDGE_SOAK_DOMAIN_ID:-219}"
run_directory="results/bridge-shutdown-soak/${run_id}"
prespawn_delay="${FLYTO_ROBOTICS_BRIDGE_PRESPAWN_DELAY_SECONDS:-}"

if [[ ! "${run_id}" =~ ^[A-Za-z0-9][A-Za-z0-9._-]{0,79}$ ]]; then
  echo "FLYTO_ROBOTICS_BRIDGE_SOAK_RUN_ID contains unsafe characters" >&2
  exit 2
fi
if [[ ! "${cycles}" =~ ^[0-9]+$ ]] || ((cycles < 1 || cycles > 1000)); then
  echo "FLYTO_ROBOTICS_BRIDGE_SOAK_CYCLES must be between 1 and 1000" >&2
  exit 2
fi
if [[ ! "${domain_id}" =~ ^[0-9]+$ ]] || ((domain_id < 0 || domain_id > 230)); then
  echo "FLYTO_ROBOTICS_BRIDGE_SOAK_DOMAIN_ID must be between 0 and 230" >&2
  exit 2
fi
if [[ "${soak_mode}" != "steady" && "${soak_mode}" != "early" ]]; then
  echo "FLYTO_ROBOTICS_BRIDGE_SOAK_MODE must be steady or early" >&2
  exit 2
fi
if [[ -z "${prespawn_delay}" ]]; then
  if [[ "${soak_mode}" == "early" ]]; then
    prespawn_delay="0.10"
  else
    prespawn_delay="0"
  fi
fi

if ! docker image inspect "${image_name}" >/dev/null 2>&1; then
  docker build \
    -t "${image_name}" \
    -f "${repository_root}/docker/Dockerfile.jazzy" \
    "${repository_root}"
fi

source_snapshot="$(python3 - "${repository_root}" <<'PY'
import hashlib
import sys
from pathlib import Path

root = Path(sys.argv[1])
excluded = {
    ".flyto-index",
    ".git",
    ".pytest_cache",
    ".ruff_cache",
    "__pycache__",
    "build",
    "install",
    "log",
    "output",
    "results",
    "tmp",
}
digest = hashlib.sha256()
for path in sorted(item for item in root.rglob("*") if item.is_file()):
    relative = path.relative_to(root)
    if any(part in excluded for part in relative.parts):
        continue
    digest.update(str(relative).encode("utf-8"))
    digest.update(b"\0")
    digest.update(path.read_bytes())
    digest.update(b"\0")
print(digest.hexdigest())
PY
)"
container_image_id="$(docker image inspect --format '{{.Id}}' "${image_name}")"
evidence_directory="${repository_root}/${run_directory}"
mkdir -p "${evidence_directory}"

set +e
docker run --rm \
  -e "ROS_DOMAIN_ID=${domain_id}" \
  -e "ROS2CLI_DISABLE_DAEMON=1" \
  -e "BRIDGE_SOAK_CYCLES=${cycles}" \
  -e "BRIDGE_SOAK_MODE=${soak_mode}" \
  -e "FLYTO_ROBOTICS_BRIDGE_PRESPAWN_DELAY_SECONDS=${prespawn_delay}" \
  -v "${evidence_directory}:/evidence" \
  -v "${repository_root}:/workspace/flyto-robotics:ro" \
  -w /evidence \
  "${image_name}" \
  bash -lc '
    set -eo pipefail
    source /opt/ros/jazzy/setup.bash
    set -u
    ulimit -c unlimited || true
    export PYTHONPATH="/workspace/flyto-robotics:${PYTHONPATH:-}"
    printf "cycle\texit_code\tcore_dumps\tfatal_markers\tguard_buffered\tguard_forwarded\tstatus\n" > /evidence/cycles.tsv
    bridge=(
      python3
      -m
      flyto_robotics.bridge_guard
      "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock"
      "/flyto/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist"
      "/flyto/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry"
      "/flyto/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V"
      "/flyto/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan"
      --ros-args
      --disable-rosout-logs
      -r
      __node:=flyto_bridge_shutdown_soak
    )
    for ((cycle=1; cycle<=BRIDGE_SOAK_CYCLES; cycle++)); do
      printf -v cycle_label "cycle-%04d" "${cycle}"
      cycle_directory="/evidence/${cycle_label}"
      mkdir -p "${cycle_directory}"
      (
        cd "${cycle_directory}"
        "${bridge[@]}" > bridge.log 2>&1 &
        bridge_pid=$!
        started=0
        required_markers=5
        if [[ "${BRIDGE_SOAK_MODE}" == "early" ]]; then
          required_markers=0
        fi
        for _ in {1..100}; do
          marker_count="$(grep -c "Creating .* Bridge" bridge.log || true)"
          guard_armed="$(grep -c "\[flyto-bridge-guard\] armed" bridge.log || true)"
          if [[ "${BRIDGE_SOAK_MODE}" == "early" && "${guard_armed}" == "1" ]]; then
            started=1
            break
          fi
          if [[ "${BRIDGE_SOAK_MODE}" == "steady" ]] && ((marker_count >= required_markers)); then
            started=1
            break
          fi
          if ! kill -0 "${bridge_pid}" 2>/dev/null; then
            break
          fi
          sleep 0.05
        done
        if [[ "${started}" != "1" ]]; then
          kill -TERM "${bridge_pid}" 2>/dev/null || true
          set +e
          wait "${bridge_pid}"
          exit_code=$?
          set -e
          core_dumps="$(find . -maxdepth 1 -type f -name "core*" | wc -l | tr -d " ")"
          fatal_markers="$(grep -Ec "Segmentation fault|Fatal glibc error|core dumped|terminate called|RCLError" bridge.log || true)"
          guard_buffered="$(grep -c "\[flyto-bridge-guard\] buffered" bridge.log || true)"
          guard_forwarded="$(grep -c "\[flyto-bridge-guard\] forwarding" bridge.log || true)"
          printf "%s\t%s\t%s\t%s\t%s\t%s\tstartup_failed\n" \
            "${cycle}" "${exit_code}" "${core_dumps}" "${fatal_markers}" \
            "${guard_buffered}" "${guard_forwarded}" \
            >> /evidence/cycles.tsv
          exit 21
        fi
        if [[ "${BRIDGE_SOAK_MODE}" == "steady" ]]; then
          sleep 0.50
        fi
        if ((cycle == 1)) && [[ "${BRIDGE_SOAK_MODE}" == "steady" ]]; then
          ros2 node info /flyto_bridge_shutdown_soak > node-info.txt
          if grep -q "/rosout" node-info.txt; then
            kill -TERM "${bridge_pid}" 2>/dev/null || true
            wait "${bridge_pid}" 2>/dev/null || true
            printf "%s\t1\t0\t0\t0\t1\trosout_present\n" "${cycle}" \
              >> /evidence/cycles.tsv
            exit 22
          fi
        fi
        kill -INT "${bridge_pid}"
        set +e
        wait "${bridge_pid}"
        exit_code=$?
        set -e
        core_dumps="$(find . -maxdepth 1 -type f -name "core*" | wc -l | tr -d " ")"
        fatal_markers="$(grep -Ec "Segmentation fault|Fatal glibc error|core dumped|terminate called|RCLError" bridge.log || true)"
        guard_buffered="$(grep -c "\[flyto-bridge-guard\] buffered" bridge.log || true)"
        guard_forwarded="$(grep -c "\[flyto-bridge-guard\] forwarding" bridge.log || true)"
        status=clean
        if [[ "${exit_code}" != "0" || "${core_dumps}" != "0" || "${fatal_markers}" != "0" || "${guard_forwarded}" != "1" ]]; then
          status=failed
        fi
        if [[ "${BRIDGE_SOAK_MODE}" == "early" && "${guard_buffered}" != "1" ]]; then
          status=failed
        fi
        printf "%s\t%s\t%s\t%s\t%s\t%s\t%s\n" \
          "${cycle}" "${exit_code}" "${core_dumps}" "${fatal_markers}" \
          "${guard_buffered}" "${guard_forwarded}" "${status}" \
          >> /evidence/cycles.tsv
        if [[ "${status}" != "clean" ]]; then
          exit 23
        fi
      )
    done
  '
docker_exit=$?
set -e

python3 - \
  "${evidence_directory}/cycles.tsv" \
  "${evidence_directory}/report.json" \
  "${run_id}" \
  "${cycles}" \
  "${soak_mode}" \
  "${docker_exit}" \
  "${source_snapshot}" \
  "${container_image_id}" <<'PY'
import csv
import hashlib
import json
import sys
from datetime import datetime, timezone
from pathlib import Path

(
    cycles_path,
    output_path,
    run_id,
    expected_cycles,
    soak_mode,
    docker_exit,
    source_snapshot,
    image_id,
) = sys.argv[1:]
rows = []
path = Path(cycles_path)
if path.is_file():
    with path.open(encoding="utf-8", newline="") as handle:
        for row in csv.DictReader(handle, delimiter="\t"):
            rows.append(
                {
                    "cycle": int(row["cycle"]),
                    "exit_code": int(row["exit_code"]),
                    "core_dumps": int(row["core_dumps"]),
                    "fatal_markers": int(row["fatal_markers"]),
                    "guard_buffered": int(row["guard_buffered"]),
                    "guard_forwarded": int(row["guard_forwarded"]),
                    "status": row["status"],
                }
            )
clean_exits = sum(row["status"] == "clean" for row in rows)
core_dump_count = sum(row["core_dumps"] for row in rows)
fatal_marker_count = sum(row["fatal_markers"] for row in rows)
guard_buffered_cycles = sum(row["guard_buffered"] == 1 for row in rows)
guard_forwarded_cycles = sum(row["guard_forwarded"] == 1 for row in rows)
unexpected_exit_codes = sorted(
    {row["exit_code"] for row in rows if row["exit_code"] != 0}
)
expected = int(expected_cycles)
report = {
    "contract_version": "flyto.robotics.bridge-shutdown-soak.v2",
    "run_id": run_id,
    "soak_mode": soak_mode,
    "generated_at": datetime.now(timezone.utc).isoformat().replace("+00:00", "Z"),
    "expected_cycles": expected,
    "observed_cycles": len(rows),
    "clean_exits": clean_exits,
    "core_dump_count": core_dump_count,
    "fatal_marker_count": fatal_marker_count,
    "guard_buffered_cycles": guard_buffered_cycles,
    "guard_forwarded_cycles": guard_forwarded_cycles,
    "unexpected_exit_codes": unexpected_exit_codes,
    "rosout_publisher_disabled": True,
    "docker_exit_code": int(docker_exit),
    "build_provenance": {
        "source_snapshot": source_snapshot,
        "container_image_id": image_id,
        "ros_distro": "jazzy",
        "bridge_package": "ros_gz_bridge",
        "middleware": "rmw_fastrtps_cpp",
    },
    "cycles": rows,
}
report["passed"] = (
    report["docker_exit_code"] == 0
    and report["observed_cycles"] == expected
    and report["clean_exits"] == expected
    and report["core_dump_count"] == 0
    and report["fatal_marker_count"] == 0
    and report["guard_forwarded_cycles"] == expected
    and (soak_mode != "early" or report["guard_buffered_cycles"] == expected)
    and not report["unexpected_exit_codes"]
)
canonical = json.dumps(report, ensure_ascii=False, sort_keys=True, separators=(",", ":"))
report["snapshot"] = hashlib.sha256(canonical.encode("utf-8")).hexdigest()
Path(output_path).write_text(
    json.dumps(report, ensure_ascii=False, indent=2) + "\n",
    encoding="utf-8",
)
print(
    json.dumps(
        {
            "run_id": report["run_id"],
            "passed": report["passed"],
            "clean_exits": report["clean_exits"],
            "expected_cycles": report["expected_cycles"],
            "core_dump_count": report["core_dump_count"],
            "unexpected_exit_codes": report["unexpected_exit_codes"],
            "snapshot": report["snapshot"],
        },
        indent=2,
    )
)
if report["passed"] is not True:
    raise SystemExit("bridge shutdown soak failed")
PY

echo "Bridge shutdown soak evidence: ${evidence_directory}/report.json"
