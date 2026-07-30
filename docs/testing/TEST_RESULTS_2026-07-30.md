# Flyto2 exact resource-binding Gazebo results — 2026-07-30

## Outcome

The complete shortcut path passed **11/11 assertions** in real ROS 2 Jazzy and
Gazebo Harmonic. This was not deterministic kinematics and the video was not
drawn or interpolated: Gazebo published the robot camera, LiDAR, odometry, and
world pose used by the runtime and independent evaluator.

The run bound `shortcut.forward.30cm.v1` to exactly:

| Field | Verified value |
|---|---|
| Resource | `flyto-rover-sim-001` |
| Endpoint | `gazebo-rover-motion` |
| Capability | `mobility.move_relative` |
| Adapter | `robotics.gazebo` |
| Space | `gazebo-lab` |
| Plan snapshot | `3c645fc02a47dcf09a9398cbc5b8e2d8b632aec145012ae75095d149ef47171d` |

The ROS node parsed the plan independently, rejected fields outside the exact
contract, checked the immutable snapshot, and emitted only payload-free binding
evidence into the terminal result.

## Measured closed loop

| Check | Result |
|---|---:|
| Strict evaluator | 11/11 passed |
| Completed workflows | 1 |
| Mission terminal states | cancelled, completed |
| Accepted heartbeats | 26 |
| Release safe-stops | 1 |
| LiDAR obstacle stops | 1 |
| Path-clear recoveries | 1 |
| Dynamic obstacle transitions | 2 |
| Runtime/driver actions | 74 |
| Gazebo source camera frames | 15 |
| Independent world displacement | 0.41464 m |
| Named visual captures | 4 |

The first key press started the verified workflow and release cancelled it with
zero velocity. The second press ran the same workflow while a Gazebo obstacle
entered the LiDAR path. The rover stopped, remained stopped until the path
cleared, and then completed. A heartbeat stream kept the input lease alive;
the workflow never accepted a velocity or PWM value from the shortcut event.

## Visual and video evidence

The generated evidence directory contains:

- `gazebo-ready.png`
- `gazebo-release-stop.png`
- `gazebo-obstacle-stop.png`
- `gazebo-completed.png`
- `gazebo-shortcut-closed-loop.mp4`
- `report.json`, `report.md`, `shortcut-result.json`
- `driver-manifest.json`, `video-probe.json`, and the MP4 SHA-256 file

The uncut H.264 artifact is 960×540 at 4 FPS, 27 frames, and 6.75 seconds. Its
SHA-256 is
`b9aa0395da4f82d136e1f570e99a3cd740b194dc68f5b5f759716947f7b3c377`.

## Reproduction

```bash
FLYTO_ROBOTICS_SHORTCUT_RUN_ID=20260730T110000Z-facility-binding \
  make gazebo-shortcut
make verify
```

`make verify` separately validates 101 Python tests, every JSON Schema/example,
the deterministic controller paths, AI plan boundaries, human-gate workflow,
and the exact facility resource contract.

## Claim boundary

This result proves the implemented Cloud-compatible resource contract,
shortcut/dead-man lifecycle, ROS adapter, Gazebo physics, LiDAR safety stop,
recovery, evidence chain, and video provenance for this scenario. It does not
claim physical robot, hospital network, hardware E-stop, medical-device, or
human-factors certification. Those remain field acceptance gates.
