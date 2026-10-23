# TurtleBot3 zero-runtime and external ROS 2 adapter architecture

Date: 2026-09-21
Owner: ChatGPT
Branch: `fix/external-ros2-adapter-architecture`

## Product decision

The robot is standard ROS 2 equipment. Flyto2 execution lives on an external AI
Space computer.

```text
Cloud / War Room
  -> selected AI Space computer
  -> Generic ROS 2 Adapter
  -> standard ROS 2
  -> TurtleBot3
  -> odom / scan / camera / TF / action result
  -> Cloud independent verification
```

A ROS action result is never a mission verdict.

## Physical migration evidence

The real TurtleBot3 was cleaned of Flyto2 runtime and credentials. Native
TurtleBot3 bringup, LiDAR, camera, odometry and TF were observed without Flyto2
code on the Pi.

Cold boot originally produced active processes with an invisible DDS graph.
The lab service examples now wait for a usable global IPv4/default route before
starting ROS. A subsequent cold boot exposed the ROS graph without a manual
restart.

A separate OpenCR/Dynamixel liveness issue remains: one
`There is no status packet!` error left the upstream process alive while
odometry/TF stopped. Restarting upstream TurtleBot3 bringup restored both. This
must be made fail-visible/recoverable through generic ROS/OS behavior, not by
reinstalling the historical Flyto watchdog.

No new physical motion was sent during this migration.

## Artifact boundary

The default Python wheel no longer ships the retired robot-appliance surfaces:

- `deploy/` Pi job-runner package;
- legacy lifecycle profile registry;
- `flyto-job-runner`;
- `flyto-robot`;
- `flyto-robot-doctor`;
- `flyto-recovery-portal`.

External/lab ROS2, evidence, camera and resource tools remain installable.

The lifecycle/job-runner/recovery implementation remains in source temporarily
for historical tests and migration analysis only. The registry was renamed to
`legacy-lifecycle-profiles.json`.

## Current acceptance boundary

Passed:

- zero Flyto2 runtime on physical TurtleBot3;
- native ROS2 bringup/sensor/camera checks;
- cold-boot DDS network-readiness fix;
- external-artifact boundary preventing accidental Pi appliance installation.

Open:

- generic recovery/fail-visible behavior for silent OpenCR/odom/TF loss;
- truthful Nav2/SLAM full lifecycle readiness;
- real external adapter discovery from a second computer;
- physically cleared bounded move/cancel/safe-stop/evidence test;
- Cloud independent objective verification.

Historical handoffs below remain valuable for failure knowledge but do not
define current deployment topology.
