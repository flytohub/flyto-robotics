#!/usr/bin/env bash
set -euo pipefail

repository_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
image_name="${FLYTO_ROBOTICS_IMAGE:-flyto-robotics:jazzy-harmonic}"
run_id="${FLYTO_ROBOTICS_STRESS_RUN_ID:-$(date -u +%Y%m%dT%H%M%SZ)}"
run_directory="results/nav2-stress/${run_id}"
domain_id="${FLYTO_ROBOTICS_ROS_DOMAIN_ID:-91}"
soak_runs="${FLYTO_ROBOTICS_STRESS_SOAK_RUNS:-5}"
profile_id="${FLYTO_ROBOTICS_STRESS_PROFILE:-}"
pressure_profile_id="${FLYTO_ROBOTICS_PRESSURE_PROFILE:-}"
campaign_rounds=1
pressure_cpu_cores=""
pressure_cpu_millicores=0
pressure_memory_mib=0
pressure_network_delay_ms=0
pressure_network_jitter_ms=0
pressure_network_loss_percent=0
sensor_timeout_seconds="0.50"
pressure_started_at="$(date +%s)"

if [[ ! "${run_id}" =~ ^[A-Za-z0-9][A-Za-z0-9._-]{0,79}$ ]]; then
  echo "FLYTO_ROBOTICS_STRESS_RUN_ID contains unsafe characters" >&2
  exit 2
fi

if [[ -n "${pressure_profile_id}" ]]; then
  expected_campaign_profile=""
  case "${pressure_profile_id}" in
    resource-r1)
      expected_campaign_profile="fault-l3"
      pressure_cpu_cores="1.5"
      pressure_cpu_millicores=1500
      pressure_memory_mib=2048
      sensor_timeout_seconds="0.55"
      ;;
    network-n1)
      expected_campaign_profile="fault-l3"
      pressure_cpu_cores="2.0"
      pressure_cpu_millicores=2000
      pressure_memory_mib=3072
      pressure_network_delay_ms=100
      pressure_network_jitter_ms=20
      pressure_network_loss_percent=1
      sensor_timeout_seconds="0.60"
      ;;
    endurance-e1)
      expected_campaign_profile="endurance-l4"
      pressure_cpu_cores="2.0"
      pressure_cpu_millicores=2000
      pressure_memory_mib=3072
      sensor_timeout_seconds="0.55"
      ;;
    *)
      echo "Unsupported FLYTO_ROBOTICS_PRESSURE_PROFILE: ${pressure_profile_id}" >&2
      exit 2
      ;;
  esac
  if [[ -n "${profile_id}" && "${profile_id}" != "${expected_campaign_profile}" ]]; then
    echo "Pressure profile ${pressure_profile_id} requires ${expected_campaign_profile}" >&2
    exit 2
  fi
  profile_id="${expected_campaign_profile}"
fi

if [[ -n "${profile_id}" ]]; then
  if [[ -n "${FLYTO_ROBOTICS_STRESS_SOAK_RUNS+x}" ]]; then
    echo "A named stress profile owns its soak count; unset FLYTO_ROBOTICS_STRESS_SOAK_RUNS" >&2
    exit 2
  fi
  case "${profile_id}" in
    baseline-l1)
      campaign_rounds=1
      soak_runs=5
      ;;
    load-l2)
      campaign_rounds=2
      soak_runs=10
      ;;
    fault-l3)
      campaign_rounds=3
      soak_runs=5
      ;;
    endurance-l4)
      campaign_rounds=5
      soak_runs=20
      ;;
    mixed-l5)
      campaign_rounds=10
      soak_runs=20
      ;;
    *)
      echo "Unsupported FLYTO_ROBOTICS_STRESS_PROFILE: ${profile_id}" >&2
      exit 2
      ;;
  esac
fi

if [[ ! "${soak_runs}" =~ ^[0-9]+$ ]] || ((soak_runs < 1 || soak_runs > 100)); then
  echo "FLYTO_ROBOTICS_STRESS_SOAK_RUNS must be between 1 and 100" >&2
  exit 2
fi
required_domains=$((soak_runs + 3))
if [[ ! "${domain_id}" =~ ^[0-9]+$ ]] || ((domain_id + required_domains - 1 > 230)); then
  echo "FLYTO_ROBOTICS_ROS_DOMAIN_ID range cannot fit this stress run" >&2
  exit 2
fi

image_rebuild=0
if ! docker image inspect "${image_name}" >/dev/null 2>&1; then
  image_rebuild=1
elif ! docker run --rm "${image_name}" bash -lc \
  "source /opt/ros/jazzy/setup.bash && ros2 pkg prefix nav2_bringup >/dev/null"; then
  image_rebuild=1
fi
if [[ "${image_rebuild}" == "1" ]]; then
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
scenario_image="${image_name}"
if [[ "${pressure_profile_id}" == "network-n1" ]]; then
  scenario_image="${image_name}-netem"
  if ! docker image inspect "${scenario_image}" >/dev/null 2>&1; then
    docker build \
      --build-arg "BASE_IMAGE=${image_name}" \
      -t "${scenario_image}" \
      - <<'DOCKERFILE'
ARG BASE_IMAGE
FROM ${BASE_IMAGE}
RUN apt-get update \
    && apt-get install -y --no-install-recommends iproute2 \
    && rm -rf /var/lib/apt/lists/*
DOCKERFILE
  fi
fi
container_image_id="$(docker image inspect --format '{{.Id}}' "${scenario_image}")"

mkdir -p "${repository_root}/${run_directory}"
docker run --rm \
  -v "${repository_root}:/workspace" \
  -w /workspace \
  "${scenario_image}" \
  bash -lc '
    set -e
    source /opt/ros/jazzy/setup.bash
    colcon build --symlink-install
    source install/setup.bash
  '

evidence_files=()
campaign_reports=()
scenario_index=0
round_directory="${run_directory}"
run_scenario() {
  local scenario="$1"
  local label="$2"
  local evidence="${repository_root}/${round_directory}/${label}.json"
  local pressure_evidence="${repository_root}/${round_directory}/${label}.pressure"
  local scenario_domain=$((domain_id + scenario_index))
  local inject_network=0
  local docker_options=(--rm)
  if [[ -n "${pressure_profile_id}" ]]; then
    docker_options+=(
      --cpus "${pressure_cpu_cores}"
      --memory "${pressure_memory_mib}m"
      --memory-swap "${pressure_memory_mib}m"
    )
  fi
  if [[ "${pressure_profile_id}" == "network-n1" && "${scenario}" != "success" ]]; then
    inject_network=1
    docker_options+=(--cap-add NET_ADMIN)
  fi
  set +e
  docker run "${docker_options[@]}" \
    -e "ROS_DOMAIN_ID=${scenario_domain}" \
    -e "RCUTILS_LOGGING_USE_STDOUT=1" \
    -e "FLYTO_NAV2_SCENARIO=${scenario}" \
    -e "FLYTO_NAV2_EVIDENCE=/workspace/${round_directory}/${label}.json" \
    -e "FLYTO_PRESSURE_EVIDENCE=/workspace/${round_directory}/${label}.pressure" \
    -e "FLYTO_INJECT_NETWORK=${inject_network}" \
    -e "FLYTO_NETWORK_DELAY_MS=${pressure_network_delay_ms}" \
    -e "FLYTO_NETWORK_JITTER_MS=${pressure_network_jitter_ms}" \
    -e "FLYTO_NETWORK_LOSS_PERCENT=${pressure_network_loss_percent}" \
    -e "FLYTO_SENSOR_TIMEOUT_SECONDS=${sensor_timeout_seconds}" \
    -v "${repository_root}:/workspace" \
    -w /workspace \
    "${scenario_image}" \
    bash -lc '
      set -eo pipefail
      network_injection_verified=false
      network_recovery_verified=false
      record_pressure() {
        status=$?
        trap - EXIT
        {
          read -r cpu_quota cpu_period < /sys/fs/cgroup/cpu.max
          printf "cpu_quota=%s\n" "${cpu_quota}"
          printf "cpu_period=%s\n" "${cpu_period}"
          printf "memory_max=%s\n" "$(cat /sys/fs/cgroup/memory.max)"
          printf "memory_peak=%s\n" "$(cat /sys/fs/cgroup/memory.peak)"
          while read -r key value; do
            case "${key}" in
              usage_usec) printf "cpu_usage_usec=%s\n" "${value}" ;;
              throttled_usec) printf "cpu_throttled_usec=%s\n" "${value}" ;;
            esac
          done < /sys/fs/cgroup/cpu.stat
          while read -r key value; do
            if [[ "${key}" == "oom_kill" ]]; then
              printf "oom_kill=%s\n" "${value}"
            fi
          done < /sys/fs/cgroup/memory.events
          printf "network_injection_verified=%s\n" "${network_injection_verified}"
          printf "network_recovery_verified=%s\n" "${network_recovery_verified}"
          printf "scenario_exit_code=%s\n" "${status}"
        } > "${FLYTO_PRESSURE_EVIDENCE}"
        exit "${status}"
      }
      trap record_pressure EXIT
      source /opt/ros/jazzy/setup.bash
      source install/setup.bash
      if [[ "${FLYTO_INJECT_NETWORK}" == "1" ]]; then
        tc qdisc replace dev lo root netem \
          delay "${FLYTO_NETWORK_DELAY_MS}ms" "${FLYTO_NETWORK_JITTER_MS}ms" \
          loss "${FLYTO_NETWORK_LOSS_PERCENT}%"
        if tc qdisc show dev lo | grep -q netem; then
          network_injection_verified=true
        else
          exit 70
        fi
      fi
      set +e
      timeout --signal=TERM --kill-after=15s 180s \
        ros2 launch flyto_robotics nav2_closed_loop.launch.py \
          headless:=true \
          scenario:=${FLYTO_NAV2_SCENARIO} \
          sensor_timeout_seconds:=${FLYTO_SENSOR_TIMEOUT_SECONDS} \
          output_file:=${FLYTO_NAV2_EVIDENCE}
      scenario_status=$?
      set -e
      if [[ "${FLYTO_INJECT_NETWORK}" == "1" ]]; then
        tc qdisc del dev lo root
        if ! tc qdisc show dev lo | grep -q netem; then
          network_recovery_verified=true
        fi
      fi
      exit "${scenario_status}"
    ' 2>&1 | tee "${repository_root}/${round_directory}/${label}.log"
  scenario_status="${PIPESTATUS[0]}"
  set -e
  test -s "${pressure_evidence}"
  if [[ "${scenario_status}" != "0" || ! -s "${evidence}" ]]; then
    failure_reason="scenario_exit_nonzero"
    if [[ "${scenario_status}" == "0" ]]; then
      failure_reason="execution_evidence_missing"
    fi
    incident="${repository_root}/${run_directory}/pressure-incident.json"
    python3 - \
      "${pressure_profile_id:-none}" \
      "${profile_id:-none}" \
      "${scenario}" \
      "${label}" \
      "${failure_reason}" \
      "${scenario_status}" \
      "${repository_root}/${round_directory}/${label}.log" \
      "${pressure_evidence}" \
      "${incident}" <<'PY'
import hashlib
import json
import re
import sys
from datetime import datetime, timezone
from pathlib import Path

(
    pressure_profile,
    campaign_profile,
    scenario,
    label,
    reason,
    exit_code,
    log_path_text,
    pressure_path_text,
    output_path_text,
) = sys.argv[1:]
log_path = Path(log_path_text)
pressure_path = Path(pressure_path_text)
log_text = log_path.read_text(encoding="utf-8", errors="replace")
pressure = {}
for line in pressure_path.read_text(encoding="utf-8").splitlines():
    key, separator, value = line.partition("=")
    if not separator or not key or key in pressure:
        raise SystemExit("invalid pressure incident evidence")
    pressure[key] = value
markers = {
    "closed_loop_pairing_not_ready": "ROS 2 pairing is not ready" in log_text,
    "lifecycle_response_timeout": "failed to send response" in log_text,
    "lifecycle_forced_kill": bool(
        re.search(r"lifecycle_manager.*process has died .*exit code -9", log_text)
    ),
    "core_dump": "core dumped" in log_text.lower(),
    "bridge_abort": "exit code 134" in log_text,
    "oom_kill": int(pressure.get("oom_kill", "0")) > 0,
}
incident = {
    "contract_version": "flyto.robotics.ros2-pressure-incident.v1",
    "pressure_profile_id": pressure_profile,
    "campaign_profile_id": campaign_profile,
    "scenario": scenario,
    "label": label,
    "failure_reason": reason,
    "scenario_exit_code": int(exit_code),
    "execution_evidence_present": log_path.with_suffix(".json").is_file(),
    "markers": markers,
    "cgroup": pressure,
    "log_sha256": hashlib.sha256(log_path.read_bytes()).hexdigest(),
    "pressure_sha256": hashlib.sha256(pressure_path.read_bytes()).hexdigest(),
    "recorded_at": datetime.now(timezone.utc).isoformat().replace("+00:00", "Z"),
}
incident["snapshot"] = hashlib.sha256(
    json.dumps(
        incident,
        ensure_ascii=False,
        separators=(",", ":"),
        sort_keys=True,
    ).encode("utf-8")
).hexdigest()
Path(output_path_text).write_text(
    json.dumps(incident, ensure_ascii=False, indent=2) + "\n",
    encoding="utf-8",
)
print(json.dumps(incident, ensure_ascii=False, indent=2))
PY
    echo "Scenario ${label} failed (${failure_reason}); evidence preserved" >&2
    exit 1
  fi
  python3 -m flyto_robotics.cli verify-ros2-execution-evidence \
    --evidence "${evidence}" \
    --scenario "${scenario}"
  evidence_files+=("${evidence}")
  scenario_index=$((scenario_index + 1))
}

for ((round=1; round<=campaign_rounds; round++)); do
  if [[ -n "${profile_id}" ]]; then
    printf -v round_label 'round-%03d' "${round}"
    round_directory="${run_directory}/${round_label}"
  else
    round_directory="${run_directory}"
  fi
  mkdir -p "${repository_root}/${round_directory}"
  evidence_files=()
  scenario_index=0

  if [[ "${pressure_profile_id}" == "network-n1" ]]; then
    for scenario in lidar_dropout odometry_freeze nav2_lifecycle_failure; do
      run_scenario "${scenario}" "${scenario}"
    done
  fi
  for ((run=1; run<=soak_runs; run++)); do
    printf -v label 'success-%03d' "${run}"
    run_scenario success "${label}"
  done
  if [[ "${pressure_profile_id}" != "network-n1" ]]; then
    for scenario in lidar_dropout odometry_freeze nav2_lifecycle_failure; do
      run_scenario "${scenario}" "${scenario}"
    done
  fi

  grant_probe="${repository_root}/${round_directory}/grant-expiry.json"
  python3 -m flyto_robotics.cli prove-ros2-expired-grant \
    --manifest "${repository_root}/examples/ros2-adapters/flyto2-standard.json" \
    --runtime "${repository_root}/examples/ros2-runtime/ready-sim.json" \
    --resource-plan "${repository_root}/examples/resource-plans/nav2-hospital-delivery.json" \
    --semantic-map "${repository_root}/examples/maps/atomic-color-route.json" \
    --output "${grant_probe}"

  report="${repository_root}/${round_directory}/report.json"
  python3 -m flyto_robotics.cli build-ros2-stress-report \
    --evidence "${evidence_files[@]}" \
    --grant-expiry-probe "${grant_probe}" \
    --soak-runs "${soak_runs}" \
    --output "${report}"
  python3 -m flyto_robotics.cli verify-ros2-stress-report --report "${report}"
  campaign_reports+=("${report}")
done

if [[ -n "${profile_id}" ]]; then
  campaign_report="${repository_root}/${run_directory}/campaign.json"
  python3 - \
    "${profile_id}" \
    "${campaign_report}" \
    "${source_snapshot}" \
    "${container_image_id}" \
    "${campaign_reports[@]}" <<'PY'
import json
import re
import sys
from pathlib import Path

from flyto_robotics.ros2_stress_evidence import (
    build_ros2_stress_campaign,
    parse_ros2_stress_campaign,
)

profile_id, output_path, source_snapshot, container_image_id, *report_paths = sys.argv[1:]
reports = [json.loads(Path(path).read_text(encoding="utf-8")) for path in report_paths]
log_paths = [path for report_path in report_paths for path in Path(report_path).parent.glob("*.log")]
death_pattern = re.compile(
    r"\[ERROR\] \[(?P<process>[^]]+)\]: process has died .*?exit code (?P<code>-?\d+)"
)
expected_terminations = 0
unexpected_exit_codes: set[int] = set()
unexpected_process_deaths = 0
for log_path in log_paths:
    for match in death_pattern.finditer(log_path.read_text(encoding="utf-8")):
        process = match.group("process")
        code = int(match.group("code"))
        expected = (process.startswith("gazebo-") and code == -15) or (
            log_path.stem == "nav2_lifecycle_failure"
            and process.startswith("lifecycle_manager-")
            and code == -9
        )
        if expected:
            expected_terminations += 1
        else:
            unexpected_process_deaths += 1
            unexpected_exit_codes.add(code)
campaign = build_ros2_stress_campaign(
    reports,
    profile_id=profile_id,
    build_provenance={
        "source_snapshot": source_snapshot,
        "container_image_id": container_image_id,
        "ros_distro": "jazzy",
        "simulator": "gazebo-harmonic",
        "execution_mode": "simulation",
    },
    runtime_hygiene={
        "scenario_log_count": len(log_paths),
        "expected_forced_terminations": expected_terminations,
        "unexpected_process_deaths": unexpected_process_deaths,
        "unexpected_exit_codes": sorted(unexpected_exit_codes),
    },
)
Path(output_path).write_text(
    json.dumps(campaign, ensure_ascii=False, indent=2) + "\n",
    encoding="utf-8",
)
verified = parse_ros2_stress_campaign(
    json.loads(Path(output_path).read_text(encoding="utf-8"))
)
print(
    json.dumps(
        {
            "campaign_id": verified["campaign_id"],
            "profile_id": verified["profile_id"],
            "passed": verified["passed"],
            "rounds": verified["round_count"],
            "success_runs": verified["total_success_runs"],
            "fault_runs": verified["total_fault_runs"],
            "max_stop_latency_ms": verified["max_safety_stop_latency_ms"],
            "max_post_stop_drift_m": verified["max_post_stop_drift_m"],
            "unexpected_process_deaths": verified["runtime_hygiene"][
                "unexpected_process_deaths"
            ],
        },
        indent=2,
    )
)
if verified["passed"] is not True:
    raise SystemExit("stress campaign failed")
PY

  if [[ -n "${pressure_profile_id}" ]]; then
    pressure_report="${repository_root}/${run_directory}/pressure-report.json"
    python3 - \
      "${pressure_profile_id}" \
      "${campaign_report}" \
      "${pressure_report}" \
      "${pressure_started_at}" <<'PY'
import json
import sys
import time
from pathlib import Path

from flyto_robotics.ros2_stress_evidence import (
    ROS2_PRESSURE_PROFILES,
    build_ros2_pressure_report,
    parse_ros2_pressure_report,
)

profile_id, campaign_path_text, output_path_text, started_at_text = sys.argv[1:]
campaign_path = Path(campaign_path_text)
output_path = Path(output_path_text)
campaign = json.loads(campaign_path.read_text(encoding="utf-8"))
profile = ROS2_PRESSURE_PROFILES[profile_id]
raw_paths = sorted(campaign_path.parent.glob("round-*/*.pressure"))
records: list[tuple[Path, dict[str, str]]] = []
for path in raw_paths:
    record: dict[str, str] = {}
    for line in path.read_text(encoding="utf-8").splitlines():
        key, separator, value = line.partition("=")
        if not separator or not key or key in record:
            raise SystemExit(f"invalid pressure evidence line: {path}")
        record[key] = value
    records.append((path, record))

required_fields = {
    "cpu_quota",
    "cpu_period",
    "memory_max",
    "memory_peak",
    "cpu_usage_usec",
    "cpu_throttled_usec",
    "oom_kill",
    "network_injection_verified",
    "network_recovery_verified",
    "scenario_exit_code",
}
if not records or any(set(record) != required_fields for _, record in records):
    raise SystemExit("pressure evidence fields are incomplete")

expected_memory = int(profile["memory_limit_mib"]) * 1024 * 1024
expected_millicores = int(profile["cpu_limit_millicores"])
cpu_limit_verified = all(
    record["cpu_quota"].isdigit()
    and record["cpu_period"].isdigit()
    and round(
        int(record["cpu_quota"]) / int(record["cpu_period"]) * 1000
    )
    == expected_millicores
    for _, record in records
)
memory_limit_verified = all(
    record["memory_max"].isdigit()
    and int(record["memory_max"]) == expected_memory
    for _, record in records
)
completed = sum(
    record["scenario_exit_code"] == "0"
    and path.with_suffix(".json").is_file()
    for path, record in records
)
injected = [
    record
    for _, record in records
    if record["network_injection_verified"] == "true"
]
network_expected = profile["mode"] == "network"
network_injection_verified = (
    len(injected) == int(campaign["total_fault_runs"])
    and all(record["network_recovery_verified"] == "true" for record in injected)
    if network_expected
    else False
)
clean_successes = [
    record
    for path, record in records
    if path.stem.startswith("success-")
    and record["network_injection_verified"] == "false"
    and record["scenario_exit_code"] == "0"
]
recovery_verified = (
    campaign["passed"] is True
    and len(clean_successes) == int(campaign["total_success_runs"])
)
observations = {
    "campaign_passed": campaign["passed"],
    "campaign_execution_runs": int(campaign["total_execution_runs"]),
    "scenario_log_count": int(campaign["runtime_hygiene"]["scenario_log_count"]),
    "completed_scenarios": completed,
    "cpu_limit_verified": cpu_limit_verified,
    "memory_limit_verified": memory_limit_verified,
    "max_memory_bytes": max(int(record["memory_peak"]) for _, record in records),
    "cpu_usage_usec": sum(int(record["cpu_usage_usec"]) for _, record in records),
    "cpu_throttled_usec": sum(
        int(record["cpu_throttled_usec"]) for _, record in records
    ),
    "oom_kill_count": sum(int(record["oom_kill"]) for _, record in records),
    "unexpected_process_deaths": int(
        campaign["runtime_hygiene"]["unexpected_process_deaths"]
    ),
    "network_injection_verified": network_injection_verified,
    "recovery_verified": recovery_verified,
    "elapsed_seconds": round(max(0.001, time.time() - int(started_at_text)), 3),
}
report = build_ros2_pressure_report(
    campaign,
    pressure_profile_id=profile_id,
    observations=observations,
)
output_path.write_text(
    json.dumps(report, ensure_ascii=False, indent=2) + "\n",
    encoding="utf-8",
)
verified = parse_ros2_pressure_report(
    json.loads(output_path.read_text(encoding="utf-8"))
)
print(
    json.dumps(
        {
            "report_id": verified["report_id"],
            "pressure_profile_id": verified["pressure_profile_id"],
            "passed": verified["passed"],
            "execution_runs": verified["observations"]["completed_scenarios"],
            "max_memory_mib": round(
                verified["observations"]["max_memory_bytes"] / 1024 / 1024,
                2,
            ),
            "cpu_throttled_usec": verified["observations"][
                "cpu_throttled_usec"
            ],
            "oom_kill_count": verified["observations"]["oom_kill_count"],
            "network_injection_verified": verified["observations"][
                "network_injection_verified"
            ],
            "recovery_verified": verified["observations"]["recovery_verified"],
        },
        indent=2,
    )
)
if verified["passed"] is not True:
    raise SystemExit("pressure campaign failed")
PY
  fi
fi

echo "Nav2 stress evidence: ${repository_root}/${run_directory}"
