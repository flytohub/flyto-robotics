# The robot's Nav2 file and Wi-Fi unit, compared with the robot

Owner: claude
Branch: claude/robot-site-config-verified (off main 59e1d2a)
Date: 2026-10-11

## What changed

#68 reconstructed `deploy/native_ros2/nav2-burger-flyto.yaml` and
`wifi-powersave-off.service` while the robot was offline. The robot came back
on 2026-10-11 (192.168.0.34, host key SHA256:pg3FeSClkQ65tAfaRK8ah+giMgnddaWbmlYIJkicWX4)
and a read-only comparison (ssh, sha256sum, scp from the robot; nothing
written there) found both differ:

- Nav2 file: the robot also has `bt_navigator` `default_server_timeout: 200`
  and global costmap `transform_tolerance: 0.5` (stock 0.2); both predate
  2026-10-08 (they are in the robot's `.bak-20261008`). The robot keeps the
  `static_layer` parameter block and only dropped it from `plugins`; #68's
  tests asserted the block was gone.
- Wi-Fi unit: `Wants=` (not `Requires=`) on the wlan0 device, no
  `RemainAfterExit`, a different Description.

Both repository files are now the robot's bytes. The test pins their sha256
as the robot printed it (nav2 `4092dd91...a76d`, unit `0269fa4c...c8cd`) and
replaces the "no StaticLayer anywhere" test with "static_layer is not a
plugin". The README describes the real changes and lists what the robot runs
that this directory does not carry.

## Verified

- Robot, read-only, 2026-10-11 14:45-14:47: power save `off`;
  `wifi-powersave-off.service` enabled, Result=success, ExecMainStatus=0
  (inactive is expected for a oneshot without RemainAfterExit); active:
  rosbridge, nav2, camera-v4l2, turtlebot3-bringup, flyto-static-map,
  laser-scan-stabilizer. Running nav2 command line loads
  `/etc/ros/nav2-burger-flyto.yaml`.
- New tests fail on main 59e1d2a's files (3 failed, 5 passed) and pass on
  this branch. `make verify`: ruff clean, 1655 passed, 1 skipped; `flyto-index verify --strict`: exit 0.

## Not verified / not in the repository

- The robot runs with SLAM off: `slam-toolbox.service` is disabled,
  `flyto-static-map.service` publishes the map frame, `nav2.service` is
  changed by drop-ins (`flyto-params.conf`, `flyto-map-source.conf`,
  `readiness.conf`, `startup-stability.conf`), and the repo's example
  `nav2.service` still `Requires=` slam-toolbox. These units, their drop-ins
  and the scripts under `/usr/local/libexec/` are not in this repository; a
  reinstall from it reproduces the Nav2 parameters and Wi-Fi setting only.
- Twin unchanged (still stock global costmap).
