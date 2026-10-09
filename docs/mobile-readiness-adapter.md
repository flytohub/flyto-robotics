# Read-only ROS2 readiness in local Flyto2 App

This package provides a **standalone, optional, read-only capability adapter**
for local Flyto2 Runtime and mobile AI Space. Neither hosted Flyto2 Cloud
nor the cybersecurity Engine is imported.

The existing passive flyto_robotics.ros2_adapter observes the standard ROS2
topic graph; it writes ros2-adapter-status.json in its configured state
directory. Stable graph observations are rewritten at least every five
seconds as a health heartbeat without a repeated event log line.

The new provider can be started by an operator-installed Runtime adapter
manifest using a fixed command:

    /absolute/path/to/python3 -m flyto_robotics.mobile_provider \
      --status-file /absolute/path/to/ros2-adapter-status.json

It reads one flyto2.execution.v1 invocation on stdin with:

- capability: robot.ros2.readiness
- revision: 1
- input: an empty object
- a unique, bounded invocation_id and operation_id

It responds in the same wire contract. A fresh, structurally valid ROS2
observation returns a successful **read** containing graph readiness,
missing/mismatched topics, and a SHA-256 digest of the original bytes.
An unready graph still returns ready=false; it does NOT claim the robot
completed a movement or mission. Stale (>15 seconds), missing, malformed
or symlinked status observations return a failed read.

The host mobile gateway must separately allowlist this capability, and
phone pairing requires independent pinned TLS identity. This adapter
neither opens a motor interface nor relaxes device-side safe-stop rules.

## Not yet supported

ROS2 NavigateToPose/DriveOnHeading/BackUp/Spin and camera actuation from
the phone require a signed/approved host capability profile, real-time
sensor freshness, independent movement policy, stop-on-disconnect and
physical evidence verification. Do not infer permission to move from
successful topic-graph readiness.
