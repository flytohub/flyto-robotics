#!/usr/bin/env bash
set -euo pipefail

instance_name="${FLYTO_GAZEBO_LIMA_INSTANCE:-flyto-robot-gazebo}"
if ! command -v limactl >/dev/null 2>&1; then
  exit 0
fi
if ! limactl list -q | grep -Fxq "${instance_name}"; then
  echo "Lima Gazebo lab does not exist: ${instance_name}"
  exit 0
fi
if [[ "$(limactl list --format '{{.Status}}' "${instance_name}")" != "Running" ]]; then
  echo "Lima Gazebo lab is already stopped: ${instance_name}"
  exit 0
fi

limactl shell "${instance_name}" bash -s <<'GUEST'
set -eo pipefail
runtime_root="${HOME}/.local/share/flyto-robot-gazebo/runtime"
workspace_root="${HOME}/.local/share/flyto-robot-gazebo/workspace"
if [[ -f "${workspace_root}/install/setup.bash" ]]; then
  source /opt/ros/jazzy/setup.bash
  source "${workspace_root}/install/setup.bash"
  timeout 2s ros2 topic pub --once /flyto/cmd_vel geometry_msgs/msg/Twist '{}' \
    >/dev/null 2>&1 || true
  timeout 2s ros2 topic pub --once /flyto/actuator_cmd_vel geometry_msgs/msg/Twist '{}' \
    >/dev/null 2>&1 || true
fi
for pid_file in "${runtime_root}/gateway.pid" "${runtime_root}/gazebo.pid"; do
  [[ -f "${pid_file}" ]] || continue
  managed_pid="$(<"${pid_file}")"
  if [[ "${managed_pid}" =~ ^[0-9]+$ ]] && kill -0 "${managed_pid}" 2>/dev/null; then
    kill -TERM "${managed_pid}"
  fi
done
sleep 1
GUEST

limactl --tty=false stop "${instance_name}"
echo "Stopped ${instance_name}; its CPU and 4 GiB memory allocation are released."
