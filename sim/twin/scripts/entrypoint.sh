#!/usr/bin/env bash
# Start the twin in the same order as the robot's systemd units:
# bringup (here: Gazebo) -> laser-scan-stabilizer -> slam-toolbox -> nav2,
# plus rosbridge. Each ros2 launch line repeats the robot's ExecStart arguments
# verbatim except use_sim_time, which Gazebo requires.
# ROS setup scripts read unset variables, so strict mode starts after them.
source /opt/ros/jazzy/setup.bash
set -u
LOG=/var/log/twin
mkdir -p "$LOG"

start() {  # name, command...
  local name=$1; shift
  echo "[twin] starting $name"
  "$@" >"$LOG/$name.log" 2>&1 &
}

wait_for_topic() {  # topic, seconds
  local topic=$1 limit=$2 waited=0
  until ros2 topic list 2>/dev/null | grep -qx "$topic"; do
    sleep 2; waited=$((waited + 2))
    if [ "$waited" -ge "$limit" ]; then
      echo "[twin] $topic did not appear within ${limit}s; see $LOG" >&2
      return 1
    fi
  done
  echo "[twin] $topic is up"
}

# Display for the Gazebo window, served to the browser through noVNC.
start xvfb Xvfb :1 -screen 0 1600x900x24 -nolisten tcp
sleep 1
start openbox openbox
start x11vnc x11vnc -display :1 -forever -shared -nopw -localhost -rfbport 5900 -quiet
start novnc websockify --web=/usr/share/novnc/ 6080 localhost:5900

# Bringup: Gazebo world with the burger model instead of the hardware driver.
start gazebo ros2 launch turtlebot3_gazebo "${TWIN_WORLD_LAUNCH:-turtlebot3_world.launch.py}"
wait_for_topic /scan 240 || true
wait_for_topic /odom 60 || true

start laser-scan-stabilizer /usr/local/libexec/ros2-laserscan-stabilizer.py
wait_for_topic /scan_stable 60 || true

start slam-toolbox ros2 launch slam_toolbox online_async_launch.py \
  autostart:=true use_lifecycle_manager:=false use_sim_time:=true \
  slam_params_file:=/etc/ros/slam_toolbox.yaml

start nav2 ros2 launch nav2_bringup bringup_launch.py \
  slam:=False use_localization:=False use_composition:=False use_respawn:=True \
  params_file:=/opt/ros/jazzy/share/turtlebot3_navigation2/param/burger.yaml \
  use_sim_time:=True autostart:=True

# The robot binds rosbridge to 127.0.0.1 and is reached through an SSH tunnel.
# Inside the container it must listen on all interfaces; compose publishes it
# on the host's 127.0.0.1 only, which gives the same exposure.
start rosbridge ros2 launch rosbridge_server rosbridge_websocket_launch.xml \
  port:=9090 address:=0.0.0.0 respawn:=true \
  websocket_ping_interval:=20.0 websocket_ping_timeout:=20.0

echo "[twin] all services started; logs in $LOG"
echo "[twin] Gazebo view: http://localhost:6080/vnc.html   rosbridge: ws://127.0.0.1:19090"
wait
