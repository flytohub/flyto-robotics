# The demo robot's Nav2 file and Wi-Fi unit live in the repository

Owner: claude
Branch: claude/robot-site-config (off main after #66, 0.7.1)
Date: 2026-10-11

## What changed

The demo robot (burger-01) carried two changes made on the robot itself, so a
reinstall from this repository would have brought back a different robot:

- `/etc/ros/nav2-burger-flyto.yaml`, global costmap changed 2026-10-08:
  `rolling_window: true`, `width: 6`, `height: 6`, `track_unknown_space:
  false`, `static_layer` removed (robot backup `.bak-20261008`).
- `wifi-powersave-off.service` (2026-10-10): oneshot `iw dev wlan0 set
  power_save off` (flyto-cloud handoff 2026-10-10-demo-machine-and-robot.md).

Now in `deploy/native_ros2/`: `nav2-burger-flyto.yaml` (turtlebot3_navigation2
2.3.6 `burger.yaml`, read from the twin image, plus exactly those global
costmap changes), `wifi-powersave-off.service`, `nav2.service` loading
`/etc/ros/nav2-burger-flyto.yaml`, install steps in its README. The twin
README records that the twin still runs the stock global costmap.
`tests/test_robot_site_config.py` pins the recorded values.

## Verified

- `make verify`: 1653 passed, 1 skipped; `flyto-index verify . --full-scan
  --strict`: PASS.
- `tests/test_robot_site_config.py` and `tests/test_packaging.py` pass (the
  yaml and the unit ship under `share/flyto-robotics/native_ros2`).
- The yaml differs from the stock `burger.yaml` only in the global costmap
  (diff checked by hand).

## Not verified

- Neither file was compared with the robot: 192.168.0.34 and 10.222.24.10
  timed out on 2026-10-11 and `flyto-robot.local` did not resolve. Both are
  reconstructed from the owner's record. Owner action, read-only:
  `ssh ubuntu@<robot> cat /etc/ros/nav2-burger-flyto.yaml | diff - deploy/native_ros2/nav2-burger-flyto.yaml`
  and `ssh ubuntu@<robot> systemctl cat wifi-powersave-off.service`; fix the
  repository if they differ, never the robot from here.
- Whether the robot's own nav2.service uses the same arguments as the example
  unit.
- The twin was not changed to the rolling global costmap.
