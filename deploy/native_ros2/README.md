# Native ROS 2 TurtleBot3 startup examples

These units are site configuration examples for a standard ROS 2 TurtleBot3.
They contain no Flyto2 code, credentials, gateways, schedulers, task state or
custom ROS nodes.

They preserve several physical findings from the lab:

- on a Wi-Fi Raspberry Pi, creating DDS participants before the interface has a
  global address/default route can leave an otherwise healthy ROS 2 stack
  undiscoverable after a cold boot;
- the ROS 2 CLI daemon can retain stale discovery state during startup, so
  readiness probes use `--no-daemon` and require a fresh odometry sample;
- SLAM and Nav2 are separate services. SLAM waits for fresh odom/scan, while
  Nav2 waits for a live `map -> odom` transform before starting navigation;
- Nav2 uses upstream non-composed/respawn mode so one navigation process does
  not take the whole composed container down;
- the USB camera publishes its native YUYV encoding instead of spending a CPU
  core converting every frame to RGB on the robot.

The example units are:

- `turtlebot3-bringup.service` — upstream TurtleBot3 base/LiDAR/TF;
- `camera-v4l2.service` — standard `v4l2_camera`;
- `slam-toolbox.service` — upstream asynchronous SLAM Toolbox;
- `nav2.service` — upstream Nav2 navigation only, with the demo robot's
  parameters `/etc/ros/nav2-burger-flyto.yaml`;
- `nav2-burger-flyto.yaml` — those parameters (see below);
- `wifi-powersave-off.service` — Wi-Fi power save off on `wlan0` at boot.

## The demo robot (burger-01)

A reinstall reproduces the robot the 2026-10-14 demo runs on with:

```bash
sudo apt install iw
sudo install -m 0644 nav2-burger-flyto.yaml /etc/ros/nav2-burger-flyto.yaml
sudo install -m 0644 *.service /etc/systemd/system/
sudo systemctl daemon-reload
sudo systemctl enable --now wifi-powersave-off.service
sudo systemctl enable turtlebot3-bringup slam-toolbox nav2 camera-v4l2
```

- `nav2-burger-flyto.yaml` and `wifi-powersave-off.service` are the robot's
  own bytes, copied read-only on 2026-10-11 (`tests/test_robot_site_config.py`
  pins their sha256 as the robot printed it). Compare them again, read-only:
  `ssh ubuntu@<robot> sha256sum /etc/ros/nav2-burger-flyto.yaml /etc/systemd/system/wifi-powersave-off.service`.
- `nav2-burger-flyto.yaml` is turtlebot3_navigation2 2.3.6's `burger.yaml`
  with three changes. Before 2026-10-08 (already in the robot's backup
  `/etc/ros/nav2-burger-flyto.yaml.bak-20261008`): `bt_navigator`
  `default_server_timeout: 200` and the global costmap's
  `transform_tolerance: 0.5`. On 2026-10-08: the global costmap became a
  rolling 6 m x 6 m window (`rolling_window: true`, `width: 6`, `height: 6`),
  `track_unknown_space: false`, and `static_layer` left its `plugins` list, so
  the map is not a layer of it and obstacles come from the live scan. The
  `static_layer` parameter block is still in the file, unused. The local
  costmap, the controller and the behaviors are stock.
- `wifi-powersave-off.service` runs `iw dev wlan0 set power_save off` once per
  boot (2026-10-10). Power save on the robot's Raspberry Pi put rosbridge
  latency over the phone hotspot at 11-177 ms; off, 9-18 ms. It has no
  `RemainAfterExit`, so `systemctl is-active` reads `inactive` after a
  successful run; check `iw dev wlan0 get power_save` instead.
- Not yet in this directory: the robot runs with SLAM off.
  `slam-toolbox.service` is disabled there, `flyto-static-map.service`
  (`/usr/local/libexec/flyto-static-map.py`) publishes the fixed map frame,
  `laser-scan-stabilizer.service` and `rosbridge.service` (127.0.0.1:9090)
  run, and `nav2.service` is the example unit plus drop-ins in
  `/etc/systemd/system/nav2.service.d/` (`flyto-params.conf` loads
  `/etc/ros/nav2-burger-flyto.yaml`; `flyto-map-source.conf` wants
  `flyto-static-map.service`; `readiness.conf` uses
  `/usr/local/libexec/ros2-turtlebot-readiness.py`), with matching drop-ins
  for `slam-toolbox` and `camera-v4l2`. The install steps above therefore
  reproduce the Nav2 parameters and Wi-Fi setting, not the whole boot
  sequence.
- The twin (`sim/twin`) still runs the stock `burger.yaml` global costmap;
  see its README.

The robot side remains replaceable. A clean upstream TurtleBot3 installation
can use its own startup mechanism instead; Flyto2 only requires the external
computer to rediscover the standard ROS 2 graph.

Site-specific values such as ROS_DOMAIN_ID, camera device path, TurtleBot3
model, LiDAR model and Nav2 parameter file should be adjusted locally.
