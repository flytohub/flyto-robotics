#!/usr/bin/env bash
set -euo pipefail

repository_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
instance_name="${FLYTO_GAZEBO_LIMA_INSTANCE:-flyto-robot-gazebo}"
fault_scenario="${FLYTO_GAZEBO_FAULT:-none}"
start_gateway=1
follow_log=0

while (($#)); do
  case "$1" in
    --fault)
      fault_scenario="${2:?--fault requires a scenario}"
      shift 2
      ;;
    --no-gateway)
      start_gateway=0
      shift
      ;;
    --follow)
      follow_log=1
      shift
      ;;
    *)
      echo "unknown argument: $1" >&2
      exit 2
      ;;
  esac
done

case "${fault_scenario}" in
  none|lidar_dropout|odometry_freeze) ;;
  *)
    echo "fault must be none, lidar_dropout, or odometry_freeze" >&2
    exit 2
    ;;
esac

"${repository_root}/scripts/provision-lima-gazebo.sh"

limactl shell "${instance_name}" env \
  "FLYTO_GAZEBO_FAULT=${fault_scenario}" \
  "FLYTO_GAZEBO_GATEWAY=${start_gateway}" \
  bash -s <<'GUEST'
set -eo pipefail

workspace_root="${HOME}/.local/share/flyto-robot-gazebo/workspace"
runtime_root="${HOME}/.local/share/flyto-robot-gazebo/runtime"
mkdir -p "${runtime_root}"
source /opt/ros/jazzy/setup.bash
source "${workspace_root}/install/setup.bash"

managed_group_contains() {
  local managed_pgid="$1"
  local expected="$2"
  ps -eo pgid=,args= | awk -v pgid="${managed_pgid}" -v expected="${expected}" '
    $1 == pgid && index($0, expected) { found = 1 }
    END { exit !found }
  '
}

stop_managed_session() {
  local name="$1"
  local expected="$2"
  local pid_file="${runtime_root}/${name}.pid"
  local pgid_file="${runtime_root}/${name}.pgid"
  local managed_pid=""
  local managed_pgid=""
  [[ -f "${pid_file}" ]] && managed_pid="$(<"${pid_file}")"
  [[ -f "${pgid_file}" ]] && managed_pgid="$(<"${pgid_file}")"

  if [[ "${managed_pgid}" =~ ^[0-9]+$ ]] \
      && managed_group_contains "${managed_pgid}" "${expected}"; then
    kill -TERM -- "-${managed_pgid}"
    for _attempt in {1..50}; do
      kill -0 -- "-${managed_pgid}" 2>/dev/null || break
      sleep 0.1
    done
    if kill -0 -- "-${managed_pgid}" 2>/dev/null; then
      kill -KILL -- "-${managed_pgid}"
    fi
  fi
  if [[ "${managed_pid}" =~ ^[0-9]+$ ]] && kill -0 "${managed_pid}" 2>/dev/null; then
    kill -TERM "${managed_pid}" 2>/dev/null || true
  fi
  rm -f "${pid_file}" "${pgid_file}"
}

start_managed_session() {
  local name="$1"
  local expected="$2"
  shift 2
  local log_file="${runtime_root}/${name}.log"
  setsid --fork --wait "$@" >"${log_file}" 2>&1 </dev/null &
  local supervisor_pid=$!
  local session_pid=""
  local managed_pgid=""

  for _attempt in {1..50}; do
    if ! kill -0 "${supervisor_pid}" 2>/dev/null; then
      tail -n 80 "${log_file}" >&2
      return 1
    fi
    session_pid="$(pgrep -P "${supervisor_pid}" | head -n 1 || true)"
    if [[ "${session_pid}" =~ ^[0-9]+$ ]]; then
      managed_pgid="$(ps -o pgid= -p "${session_pid}" | tr -d ' ' || true)"
      if [[ "${managed_pgid}" =~ ^[0-9]+$ ]] \
          && managed_group_contains "${managed_pgid}" "${expected}"; then
        break
      fi
    fi
    sleep 0.1
  done
  if [[ ! "${managed_pgid}" =~ ^[0-9]+$ ]] \
      || ! managed_group_contains "${managed_pgid}" "${expected}"; then
    kill -TERM "${supervisor_pid}" 2>/dev/null || true
    tail -n 80 "${log_file}" >&2
    return 1
  fi
  printf '%s\n' "${supervisor_pid}" >"${runtime_root}/${name}.pid"
  printf '%s\n' "${managed_pgid}" >"${runtime_root}/${name}.pgid"
}

timeout 2s ros2 topic pub --once /flyto/cmd_vel geometry_msgs/msg/Twist '{}' \
  >/dev/null 2>&1 || true
stop_managed_session gateway "flyto_robotics.cli serve-delivery"
stop_managed_session gazebo "turtlebot3_fidelity.launch.py"

export ROS_DOMAIN_ID=30
export TURTLEBOT3_MODEL=burger
export LIBGL_ALWAYS_SOFTWARE=1
export QT_QPA_PLATFORM=offscreen

start_managed_session gazebo "turtlebot3_fidelity.launch.py" bash -lc "
  source /opt/ros/jazzy/setup.bash
  source '${workspace_root}/install/setup.bash'
  exec ros2 launch flyto_robotics turtlebot3_fidelity.launch.py \
    headless:=true fault_scenario:=${FLYTO_GAZEBO_FAULT}
"
gazebo_pid="$(<"${runtime_root}/gazebo.pid")"

ready=0
for _attempt in {1..60}; do
  if ! kill -0 "${gazebo_pid}" 2>/dev/null; then
    tail -n 80 "${runtime_root}/gazebo.log" >&2
    exit 3
  fi
  topic_list="$(timeout 3s ros2 topic list 2>/dev/null || true)"
  if grep -Fxq '/flyto/odom' <<<"${topic_list}" \
      && grep -Fxq '/flyto/scan' <<<"${topic_list}" \
      && grep -Fxq '/flyto/imu' <<<"${topic_list}"; then
    ready=1
    break
  fi
  sleep 1
done
if [[ "${ready}" != "1" ]]; then
  tail -n 120 "${runtime_root}/gazebo.log" >&2
  exit 4
fi

if [[ "${FLYTO_GAZEBO_GATEWAY}" == "1" ]]; then
  secret_file="${runtime_root}/gateway.env"
  if [[ ! -s "${secret_file}" ]]; then
    umask 077
    delivery_token="$(openssl rand -hex 32)"
    qr_signing_value="$(openssl rand -hex 32)"
    printf 'export FLYTO_ROBOTICS_DELIVERY_TOKEN=%s\n' "${delivery_token}" >"${secret_file}"
    printf 'export FLYTO_ROBOTICS_QR_SECRET=%s\n' "${qr_signing_value}" >>"${secret_file}"
  fi
  chmod 600 "${secret_file}"
  start_managed_session gateway "flyto_robotics.cli serve-delivery" bash -lc "
    source /opt/ros/jazzy/setup.bash
    source '${workspace_root}/install/setup.bash'
    source '${secret_file}'
    exec python3 -m flyto_robotics.cli serve-delivery \
      --job '${workspace_root}/examples/jobs/ai-space-delivery.json' \
      --host 127.0.0.1 \
      --port 8766 \
      --backend ros2 \
      --gazebo \
      --semantic-map '${workspace_root}/examples/maps/hospital-ward-delivery.json' \
      --semantic-map-id hospital.ward-delivery.v1
  "
  gateway_pid="$(<"${runtime_root}/gateway.pid")"
  sleep 1
  if ! kill -0 "${gateway_pid}" 2>/dev/null; then
    cat "${runtime_root}/gateway.log" >&2
    exit 5
  fi
fi

printf 'gazebo_pid=%s\nfault=%s\ngateway=%s\n' \
  "${gazebo_pid}" "${FLYTO_GAZEBO_FAULT}" "${FLYTO_GAZEBO_GATEWAY}"
GUEST

echo "TurtleBot3 Burger simulation is running in ${instance_name}."
echo "Physics and sensors: Gazebo Harmonic, 1 ms step, LiDAR + IMU noise."
echo "Stop and release its memory with: ${repository_root}/scripts/stop-lima-gazebo.sh"

if [[ "${follow_log}" == "1" ]]; then
  limactl shell "${instance_name}" bash -lc \
    'tail -F "${HOME}/.local/share/flyto-robot-gazebo/runtime/gazebo.log"'
fi
