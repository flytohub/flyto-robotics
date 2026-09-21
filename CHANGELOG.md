# Changelog

All notable project changes are recorded here.

## Unreleased

- Reframe production around a standard ROS 2 robot controlled from an external
  AI Space computer. The robot is commanded equipment, not a Flyto2 worker.
- Remove the complete retired Pi-appliance implementation from current source:
  job runner, delivery gateway, lifecycle installer/profile registry, robot
  doctor, recovery portal, Flyto-specific watchdog, credential provisioning,
  and Flyto-specific robot systemd units.
- Keep only optional upstream-only ROS 2 site configuration examples under
  `deploy/native_ros2/`; no Flyto2 package or credential is required on a
  TurtleBot3.
- Add `flyto.robotics.ros2-observation-bundle.v1` so simulation and physical
  resources share one pose/LiDAR/camera/map-TF observation and provenance
  contract.
- Treat uncalibrated camera frames as valid raw visual evidence while refusing
  to claim metric/geometric vision readiness without a bound calibration
  snapshot.
- Separate upstream SLAM Toolbox and Nav2 startup, run Nav2 non-composed with
  respawn, and retain standard Nav2 action semantics for external adapters.
- Use the camera's native YUYV encoding on the physical Pi to avoid unnecessary
  RGB conversion load.
- Record the physical middleware convergence from Fast DDS to CycloneDDS after
  a real data-plane stall left graph discovery visible while new local
  subscribers could not receive odom, LiDAR, or camera samples.
- Add an external native-ROS2 soak probe that can be streamed over stdin for
  hardware acceptance without installing Flyto2 code on the robot.
- Replace current installation, architecture, state, security, and contract
  guidance so a clean upstream reinstall requires only standard ROS 2
  rediscovery from the external execution host.
- Preserve retired architecture evidence only in dated handoffs and Git
  history; it is no longer executable current source.

## 0.1.0 — 2026-07-28

- Created the independent `flyto-robotics` ROS 2 package.
- Added versioned, transport-neutral job and result contracts.
- Added atomic `navigate` and `dwell` primitives with injectable workflows.
- Expanded the executable vocabulary with `follow_line`, `wait_until_clear`,
  `ask_human`, `resume`, and `safe_stop`.
- Added strict AI-plan policy for terminal stopping, line-transition
  consistency, and paired human approval/resume gates.
- Added the signed human-decision contract, HMAC-SHA256 verification, short
  expiry, job/robot binding, and nonce replay rejection.
- Added the conditional ROS `/flyto/human_decision` adapter and signing CLI.
- Added structured sequence, step, capability, and actor audit evidence.
- Added namespaced capability manifests, runtime compatibility hard filters,
  language-neutral Goal Frames, canonical affordance/effect ranking, bounded
  LLM shortlists, registry snapshots, semantic coverage, ambiguity evidence,
  and shortlist enforcement.
- Added transport-neutral integration contracts for Flyto AI, trusted
  Blueprint hints, and scoped Flyto Core discovery.
- Added atomic `save_current_location` and `navigate_to_location` abilities.
- Added map-scoped semantic location storage, optimistic revisions, bounded
  Unicode labels, atomic writes, and fail-closed map identity checks.
- Added separate full-map and coordinate-free planner-catalog contracts.
- Added multilingual Goal Frame, semantic map, teaching, navigation, CLI, and
  Gazebo launch examples without changing existing atom contracts.
- Added the deterministic controller, safety stops, ROS adapter, and CLI.
- Added the self-contained hospital world and differential-drive lidar rover.
- Added Jazzy/Harmonic container verification and CI.
- Verified a complete Gazebo pharmacy-to-ward mission on Linux ARM64.
- Verified the signed CareFlow human-gate mission in Gazebo physics on Linux
  ARM64, including successful resume and replay rejection.
- Added bounded overhead-camera frame recording and reproducible H.264 encoding
  through `make gazebo-video`.
- Verified a 19.0-second, 960×540 Gazebo evidence video covering the injected
  obstacle, blue/yellow/purple traversal, approval, and terminal safe stop.
- Added the atomic `move_relative` controller with bounded signed distance,
  odometry origin capture, speed clamping, obstacle stopping, and mandatory
  terminal `safe_stop`.
- Added the versioned `input-event.v1` shortcut boundary, validated workflow
  catalog, exact source/control bindings, replay and sequence rejection,
  heartbeat dead-man timeout, release/disconnect cancellation, and audit
  events.
- Added the `shortcut.forward.30cm.v1` workflow-card example.
- Added a deterministic 30-run shortcut soak with 30/30 completions and six
  verified obstacle stops before recovery.
