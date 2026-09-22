# Adapter provider ownership closure — 2026-09-22

Owner: ChatGPT  
Branch: `fix/external-ros2-adapter-architecture`

## Result

The external equipment implementation now has a non-Cloud home.

`flyto-robotics` owns the Generic ROS 2 / rosbridge implementation and the
physical-robot evidence/safety semantics. It also ships provider implementations
for the extracted OpenRMF and vision-stream adapters. Flyto2 Runtime discovers
and starts providers; Cloud no longer imports these transports.

The host protocol is `flyto2.adapter-provider.v1`. The canonical ROS provider
executable is `flyto2-adapter-provider-ros2-generic`, matching Runtime's
generic adapter-id-to-executable convention for `ros2.generic`.

Discovery emits `flyto.resource-manifest.v1` evidence only. It grants no
authority. Execution is separately bound by Runtime to the exact commanded
resource and approved capability allowlist for one assignment.

The Generic ROS 2 adapter preserves the existing physical safety behavior:
fresh odometry and LiDAR are required, navigation additionally requires fresh
map->odom TF, the default motion clearance floor remains 0.35 m, and failed or
timed-out physical effects are followed by cancellation/safe-stop.

## Verification

- adapter-focused ROS2/OpenRMF/vision/packaging tests: 69 passed
- `make verify`: Ruff pass, 957 tests passed, asset/dry-run/pairing/authorization checks passed
- strict Flyto2 Indexer: 17 passed, 0 warnings, 0 failures

## Physical state

A read-only rosbridge preflight against the physical TurtleBot3 observed:

- native bringup, camera, SLAM, Nav2 and rosbridge services active
- recorded service restart counts: 0
- fresh odometry, LiDAR, camera and map->odom observation
- minimum LiDAR clearance: 0.143 m

Because 0.143 m is below the unchanged 0.35 m floor, no 5 cm movement was sent.
This source extraction does not claim a new positive physical motion acceptance.
That acceptance remains conditional on moving the robot into a genuinely clear
area and rerunning preflight first.
