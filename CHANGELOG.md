# Changelog

All notable project changes are recorded here.

## Unreleased

- Added the loopback AI Space delivery gateway (`flyto_robotics/delivery_gateway.py`,
  CLI `serve-delivery`): an authenticated 127.0.0.1 HTTP adapter implementing the
  Flyto Cloud relay contract (`/v1/health`, `/v1/deliveries`, poll, QR
  `/confirmation`, `/safe-stop`). Deliveries run the real `MissionController`
  under a real-time deterministic tick thread; dropoff is gated behind one
  signed `F2QR1` scan verified with job/robot/approval binding; responses carry
  only the token SHA-256 fingerprint, never the raw token; WebSocket loss
  fail-closes through relay safe-stop into `mission_cancelled` evidence.

- Added a monotonic, fail-safe ROS sensor startup gate: odometry, LiDAR, and
  camera samples must remain fresh for one continuous second before the first
  control command. Bootstrap samples from an older Gazebo generation now keep
  velocity at zero; required sensor loss after motion still fails immediately.
- Upgraded the AI4ALL GUI capture to a self-contained 1920×1080 desktop with a
  CJK-safe live evidence panel, a hospital trolley obstacle, independent
  command-velocity stop/resume evidence, and event-bound story narration.
- Added a detachable Robot MCP stdio adapter with strict protocol negotiation,
  semantic capability discovery, bounded planner requests, plan validation,
  and real deterministic-controller dry runs. No raw actuator, ROS topic,
  shell, arbitrary path, or network tool is exposed.
- Added a real 101-case Robot MCP release benchmark with distinct multilingual
  and variable-depth inputs across three difficulty tiers. Every case crosses
  the production stdio process and deterministic controller; the family and
  every tier fail closed below 90%, with atomic content-addressed JSON evidence.
- Added the strict, language-neutral `ai-space-resource-plan.v1` parser and JSON
  Schema. ROS startup now requires an exact workflow, resource, endpoint,
  capability, adapter, Space, confirmation, and immutable snapshot match when
  resource binding is enabled.
- Added payload-free resource-binding evidence to shortcut results and strict
  evaluator checks that independently compare the runtime result with the
  expected plan snapshot and adapter.
- Added the Cloud-compatible Gazebo resource-plan example, launch parameters,
  packaging, asset validation, and `make facility-contract` release gate.
- Verified the complete shortcut loop in real ROS 2 Jazzy / Gazebo Harmonic:
  release safe-stop, live heartbeat dead-man, LiDAR obstacle stop, path-clear
  recovery, 0.41464 m world displacement, exact resource binding, four visual
  captures, and an uncut H.264 evidence video passed all 11 assertions.

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
