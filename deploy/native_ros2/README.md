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
- `nav2.service` — upstream Nav2 navigation only.

The robot side remains replaceable. A clean upstream TurtleBot3 installation
can use its own startup mechanism instead; Flyto2 only requires the external
computer to rediscover the standard ROS 2 graph.

Site-specific values such as ROS_DOMAIN_ID, camera device path, TurtleBot3
model, LiDAR model and Nav2 parameter file should be adjusted locally.
