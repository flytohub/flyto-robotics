#!/bin/bash
# Record a map of the actual venue, so Nav2 can stop pretending.
#
# Run this ON THE ROBOT (ssh in first). It starts SLAM, waits while you drive
# the robot around the space, and saves the result where flyto-nav2.service
# expects it. Until that file exists, flyto-nav2.service refuses to start —
# see the ConditionPathExists in the unit and the reason above it.
#
# ## Why slam_toolbox and not cartographer
#
# Both are installed and TurtleBot3's own documentation uses cartographer, but
# it also assumes SLAM runs on a remote PC rather than the Pi. There is no
# remote PC here — the workstation is a Mac with no ROS — so this has to fit on
# a Pi 4 that is already running the camera, the MJPEG server and bringup.
# slam_toolbox in async mode is the lighter of the two and degrades by
# processing fewer scans rather than by falling behind unboundedly.
#
# ## Drive slowly
#
# Loop closure is what keeps a map square, and it needs the robot to revisit
# places it has already seen with enough overlap to match. Racing around the
# perimeter once produces a map that looks finished and is skewed.
set -euo pipefail

MAP_DIR="${FLYTO_MAP_DIR:-/home/ubuntu/.flyto/maps}"
MAP_NAME="${1:-lab}"
MAP_PATH="$MAP_DIR/$MAP_NAME"

if [ ! -f /opt/ros/jazzy/setup.bash ]; then
  echo "這台不是機器人 —— 找不到 /opt/ros/jazzy。請先 ssh 進機器人再跑。" >&2
  exit 2
fi

# shellcheck disable=SC1091
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-30}"
export TURTLEBOT3_MODEL="${TURTLEBOT3_MODEL:-burger}"

echo "=== 前置檢查 ==="

if ! systemctl is-active --quiet turtlebot3-bringup; then
  echo "  ✗ turtlebot3-bringup 沒在跑 —— 沒有 /scan 和 /odom 就無法建圖" >&2
  echo "    sudo systemctl start turtlebot3-bringup" >&2
  exit 1
fi
echo "  ✓ bringup 運作中"

# A mapping run is minutes of continuous driving. Starting one at 20% and
# having the base brown out mid-loop loses the whole run, and a LiPo taken
# below its floor is damaged rather than merely empty.
volts=$(timeout 15 ros2 topic echo /battery_state --once --field voltage 2>/dev/null | head -1 || true)
if [ -n "$volts" ]; then
  echo "  電池: ${volts} V"
  if awk "BEGIN{exit !($volts < 11.0)}" 2>/dev/null; then
    echo "  ✗ 電壓偏低。建圖要連續跑好幾分鐘，中途沒電會整份作廢。請先充電。" >&2
    exit 1
  fi
  echo "  ✓ 電壓足夠"
else
  echo "  ⚠ 讀不到電池狀態，自行確認電量"
fi

mkdir -p "$MAP_DIR"

echo
echo "=== 啟動 SLAM ==="
ros2 launch slam_toolbox online_async_launch.py use_sim_time:=False &
SLAM_PID=$!
# Stop SLAM however this script leaves, including Ctrl-C. A stray slam_toolbox
# holding /map is the next run's confusing failure.
trap 'kill $SLAM_PID 2>/dev/null || true; wait $SLAM_PID 2>/dev/null || true' EXIT INT TERM
sleep 10

if ! kill -0 $SLAM_PID 2>/dev/null; then
  echo "  ✗ slam_toolbox 沒起來" >&2
  exit 1
fi
echo "  ✓ SLAM 運作中"

cat <<'GUIDE'

=== 現在請開另一個終端機，ssh 進機器人，跑遙控 ===

    ssh ubuntu@flyto-robot.local
    source /opt/ros/jazzy/setup.bash && export ROS_DOMAIN_ID=30 TURTLEBOT3_MODEL=burger
    ros2 run turtlebot3_teleop teleop_keyboard

  開慢一點。沿著牆邊走完一圈，然後**回到起點附近再走一次**——
  回訪同一個地方是迴環閉合的依據，也是地圖不會歪掉的原因。
  把每個要送達的站點都開過去一趟。

  開完之後回到這個視窗，按 Enter 存檔。

GUIDE

read -r -p "開完了就按 Enter 存檔（Ctrl-C 放棄）… "

echo
echo "=== 存檔到 $MAP_PATH ==="
ros2 run nav2_map_server map_saver_cli -f "$MAP_PATH" --ros-args -p save_map_timeout:=10000.0

if [ -f "$MAP_PATH.yaml" ] && [ -f "$MAP_PATH.pgm" ]; then
  echo
  echo "  ✓ 存好了:"
  ls -lh "$MAP_PATH.yaml" "$MAP_PATH.pgm" | sed 's/^/    /'
  echo
  echo "  地圖資訊:"
  grep -E "resolution|origin" "$MAP_PATH.yaml" | sed 's/^/    /'
  echo
  if [ "$MAP_NAME" = "lab" ]; then
    echo "  flyto-nav2.service 的啟動條件現在滿足了。開啟導航："
    echo "    sudo systemctl enable --now flyto-nav2"
  fi
else
  echo "  ✗ 存檔失敗 —— $MAP_PATH.yaml / .pgm 沒有產生" >&2
  exit 1
fi
