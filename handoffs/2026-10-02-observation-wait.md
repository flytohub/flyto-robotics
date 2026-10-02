Owner: claude
Branch: claude/observation-wait
Date: 2026-10-02

# The adapter waits for its first LiDAR before refusing motion

## What changed

`flyto_robotics/generic_ros2_adapter.py`: both backends' `observation()` now
wait until odometry **and** LiDAR have been seen, up to
`FLYTO_ROS2_OBSERVATION_WAIT_SECONDS` (default 3, clamped to 0.25–10). The
rosbridge backend used to wait 0.5 s for odometry only; rclpy spun 0.25 s.

## Why

An execution host builds the adapter per job (`adapter_provider.build_adapter`)
and invokes it right after connecting. Odometry arrives within a few messages;
the first LiDAR revolution does not. Every Cloud-dispatched motion on the twin
was refused with `fresh LiDAR is required before motion`. Manual runs never hit
it because they slept ~2.5 s after connecting. The physical robot is the same.

## Verified

- `tests/test_rosbridge_observation_wait.py`: LiDAR arriving 0.8 s after
  odometry is waited for (fails on the old code); missing LiDAR is still
  reported once the wait ends (< 1 s with a 0.3 s wait).
- `make verify`: 963 passed.
- End to end on the twin, 2026-10-02 11:42: Space task "advance 0.1 m at
  0.05 m/s" planned by codex, started from the task site, claimed by the host
  (with flyto-cloud#415 deployed), adapter completed, odom x 0.000 -> 0.106 m,
  task `done`.

## Not verified

- On the physical robot.
- The task's verification reads `completed_unverified` with 0 execution
  receipts: direct-capability tasks are not independently verified and the
  adapter's observation bundle does not reach Cloud. Next fix.
