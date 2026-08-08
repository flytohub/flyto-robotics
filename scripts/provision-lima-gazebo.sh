#!/usr/bin/env bash
set -euo pipefail

repository_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
instance_name="${FLYTO_GAZEBO_LIMA_INSTANCE:-flyto-robot-gazebo}"
config_file="${repository_root}/lima/flyto-robot-gazebo.yaml"

if ! command -v limactl >/dev/null 2>&1; then
  echo "Lima is required: brew install lima" >&2
  exit 2
fi

if ! limactl list -q | grep -Fxq "${instance_name}"; then
  limactl --tty=false start \
    --name "${instance_name}" \
    --param "Repo=${repository_root}" \
    "${config_file}"
else
  instance_status="$(limactl list --format '{{.Status}}' "${instance_name}")"
  if [[ "${instance_status}" != "Running" ]]; then
    limactl --tty=false start "${instance_name}"
  fi
fi

limactl shell "${instance_name}" bash -s <<'GUEST'
set -eo pipefail

ready_stamp="/var/lib/flyto-robot-gazebo/ros-jazzy-ready-v2"
if [[ ! -f "${ready_stamp}" ]]; then
  export DEBIAN_FRONTEND=noninteractive
  sudo apt-get update
  sudo apt-get install -y --no-install-recommends \
    ca-certificates curl gnupg locales openssl rsync software-properties-common
  sudo add-apt-repository -y universe
  curl -fsSL -o /tmp/ros2-apt-source.deb \
    https://github.com/ros-infrastructure/ros-apt-source/releases/download/1.2.0/ros2-apt-source_1.2.0.noble_all.deb
  sudo dpkg -i /tmp/ros2-apt-source.deb
  sudo apt-get update
  sudo apt-get install -y --no-install-recommends \
    python3-colcon-common-extensions \
    python3-pytest \
    ros-jazzy-geometry-msgs \
    ros-jazzy-lifecycle-msgs \
    ros-jazzy-nav-msgs \
    ros-jazzy-navigation2 \
    ros-jazzy-nav2-bringup \
    ros-jazzy-robot-state-publisher \
    ros-jazzy-ros-base \
    ros-jazzy-ros-gz-bridge \
    ros-jazzy-ros-gz-image \
    ros-jazzy-ros-gz-sim \
    ros-jazzy-sensor-msgs \
    ros-jazzy-std-srvs \
    ros-jazzy-tf2-ros \
    ros-jazzy-turtlebot3-gazebo
  sudo install -d -m 0755 /var/lib/flyto-robot-gazebo
  sudo touch "${ready_stamp}"
fi

workspace_root="${HOME}/.local/share/flyto-robot-gazebo/workspace"
mkdir -p "${workspace_root}"
rsync -a --delete \
  --exclude '.git/' \
  --exclude '.flyto-index/' \
  --exclude '.pytest_cache/' \
  --exclude '__pycache__/' \
  --exclude 'build/' \
  --exclude 'install/' \
  --exclude 'log/' \
  --exclude 'output/' \
  --exclude 'results/' \
  /mnt/flyto-robotics/ "${workspace_root}/"

source /opt/ros/jazzy/setup.bash
rm -rf \
  "${workspace_root}/build/flyto_robotics" \
  "${workspace_root}/install/flyto_robotics"
colcon --log-base "${workspace_root}/log" build \
  --base-paths "${workspace_root}" \
  --build-base "${workspace_root}/build" \
  --install-base "${workspace_root}/install" \
  --symlink-install \
  --event-handlers console_direct+
source "${workspace_root}/install/setup.bash"
ros2 pkg prefix flyto_robotics >/dev/null
ros2 pkg prefix turtlebot3_gazebo >/dev/null
GUEST

echo "Lima Gazebo lab provisioned: ${instance_name}"
