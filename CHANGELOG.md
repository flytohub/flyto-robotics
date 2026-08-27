# Changelog

All notable project changes are recorded here.

## Unreleased

- The camera gateway now serves the stream half of the vision-gateway
  contract. `GET /api/spaces/zone-camera/streams` answers
  `flyto.vision.stream-catalog.v1` instead of 404, configured by four
  operator-only `FLYTO_CAMERA_STREAM_*` settings. This removes the workaround
  that had put a ROS topic name into flyto-cloud's own configuration; the
  platform no longer needs to know what ROS is. flyto-cloud's vision adapter
  reaches the robot on its own defaults, and its conformance kit certifies the
  stream through the tunnel.
- Nav2 is installed as `flyto-nav2.service` and left **disabled**. The stack is
  verified to come up on the Pi with the LiDAR feeding both costmaps and
  `/navigate_to_pose` advertised, but the only map in this repository is a
  20x20-pixel synthetic square, and localising against it converges into a room
  that does not exist. The unit's start condition names a map recorded from a
  real venue, so it cannot start until one exists.
- Recording that map is now a job flyto-cloud can dispatch rather than an ssh
  session: `mapping.start` / `mapping.save` / `mapping.abort` on the installed
  device-executor protocol, with `flyto-slam.service` holding SLAM between the
  request that starts it and the one that saves. Nothing in the mission engine
  changes — commissioning must not become a word the AI planner can compose a
  delivery plan out of.
- Fixed: the mapping executor never spoke the registry's wire protocol, so
  every call it made failed. It wrote a trailing newline, which the registry
  rejects outright, and read `module_id` from the top of the envelope instead of
  from inside `request`. Both were invisible to hand-checks that fed it the
  shape its author imagined. It now has 66 tests, six of them modelling the
  caller.
- Fixed: `flyto-camera-v4l2.service` and `flyto-camera-stream.service` restarted
  without a start limit, which is the defect that once had `flyto-shortcut` at
  NRestarts=193 flapping a healthy lidar. `validate_unit` now runs over
  `deploy/systemd/` — the hand-written units actually installed on the robot,
  which nothing had been pointing it at.
- Fixed: `deploy/make-map.sh` launched SLAM anyway when the battery could not be
  read, while the executor refused the identical case. An unreadable pack is
  routine, so that branch was normal use rather than an edge.
- Documentation: three files said the `ros2` lifecycle profile extends `camera`;
  the data has said `generic` since the file was created and three CI-green
  tests enforce it. `docs/INSTALLATION.md` claimed in the present perfect that no
  camera deployment had been performed. Both corrected, with the dated
  `STATE.md` closure record left standing and marked superseded rather than
  rewritten.

- Added the versioned Cloud job / robot receipt join. Trace-bearing Space jobs
  now fail before gateway use unless `flyto.cloud.device-job-handoff.v1` binds
  the paired device, trace, exact workflow digest, and Cloud completion
  authority. Terminal delivery sessions return one cached,
  content-addressed `flyto.robotics.execution-receipt.v1`; the edge runner
  independently verifies its plan and receipt digests and forwards it apart
  from pose/clearance evidence. The receipt explicitly cannot complete a Task.

- Added `flyto_robotics/bringup_watchdog.py`: an `/odom`-freshness watchdog
  bundled into `turtlebot3_bringup_supervised.launch.py` and wired to
  systemd's own watchdog protocol (`Type=notify`, `NotifyAccess=all`,
  `WatchdogSec=15`). Catches the 2026-08-07 silent hang — `turtlebot3_ros`
  alive but every topic dead — that `OnProcessExit` cannot see: stale `/odom`
  starves the watchdog timer, systemd kills the cgroup, `Restart=always`
  recovers it. Decision logic is a pure function with unit tests, and the
  whole path is verified on the real robot: an induced alive-but-silent
  `turtlebot3_ros` (SIGSTOP) was detected, killed and recovered to `/odom`
  at 20 Hz in ~56s with zero manual intervention.
- Extracted `CmdVelChannel` into `flyto_robotics/ros2_cmd_vel.py` so the
  delivery runner and the shortcut controller share one velocity publisher.
  The shortcut node still hardcoded `Twist` on a hardcoded topic, which would
  have reproduced the exact silent type mismatch already fixed for delivery.
  Verified in the Jazzy container against real `TwistStamped`, `Twist` and
  no-subscriber endpoints.
- The shortcut controller now drives several workflow cards from one process
  (`plan_files`, `binding_ids`, `input_control_ids`), with configurable
  `cmd_vel_topic`, `odom_topic`, `scan_topic` and `cmd_vel_type`. Mismatched
  list lengths fail at startup rather than mid-mission, and combining multiple
  cards with a single-workflow resource plan is refused outright.
- Added `deploy/systemd/flyto-shortcut.service` with a reciprocal
  `Conflicts=flyto-delivery.service`. Two Flyto processes publishing to one
  velocity topic have no arbitration, so one node's stop can be overwritten by
  the other's motion on the next tick; eliminating the second writer is
  provable where mediating between them is not.

- Added the `turn_relative` atomic capability: rotate in place by a bounded
  signed angle from a controller-captured odometry heading, obstacle-guarded,
  and holding linear velocity at exactly zero so a rotation can never creep
  toward something the operator cannot see. Registered in the capability
  manifest, the plan contract, the compiler, and the input-runtime motion set,
  so a turn card is enforced to end with `safe_stop` twice, independently.
- Added the four keyboard-shortcut workflow cards as reviewed JSON artifacts
  (`examples/plans/shortcut-{forward,backward}-40cm.json`,
  `shortcut-turn-{left,right}-90deg.json`) plus `examples/jobs/tb3-lab-shortcut.json`.
  Each card is sized to finish inside the operator's press-and-hold window, so
  the bounded artifact is the binding constraint rather than a browser timer.
- The input gateway now fails closed when the control thread misses its ack
  deadline. A press that answered 503 previously still started its workflow
  while nothing client-side tracked it, so no dead-man would have stopped it;
  such a press is now dropped. Release, disconnect and heartbeat are always
  delivered because they only ever stop motion.

- The ROS 2 delivery backend now binds `/cmd_vel` to whatever message type the
  driver actually subscribes with. Jazzy's TurtleBot3 driver expects
  `TwistStamped` while the bundled Gazebo bridge uses `Twist`; a mismatch
  matches zero subscribers and DDS reports no error, so the robot silently
  ignored every command while the logs looked clean. Detection happens at first
  publish, the resolved type is logged, an absent subscriber is warned about
  rather than commanded into a void, and `--cmd-vel-type` forces either type
  when determinism matters.
- Topics are configurable (`--odom-topic`, `--scan-topic`, `--cmd-vel-topic`)
  so a vendor driver keeps its standard names and teleop, rviz and Nav2 keep
  working alongside the gateway.
- A range sensor is now required only when the workflow uses clearance, so a
  delivery still runs on a robot with no lidar, forfeiting obstacle waiting
  instead of refusing to move.

- Deliveries are now goal-driven: `serve-delivery --semantic-map` resolves a
  free-form operator goal (zh-TW or English, Han or Arabic digits, partial
  labels, or a literal location ID) to exactly one semantic destination, routes
  capabilities through the frozen registry, composes a
  `flyto.robotics.plan.v1`, and compiles it to an executable workflow. Unsafe
  or unusable goals fail closed with a structured
  `flyto.robotics.delivery-rejection.v1` payload — safety-override phrases and
  raw actuator vocabulary are screened before any lookup, shared label
  fragments never pick a winner, and two destinations report ambiguity with
  candidates. Every session now carries an attributed decision timeline
  (operator / rule engine / capability registry / plan contract / executor) and
  a route graph the operator UI renders. `resolve-goal` exercises the whole
  pipeline offline.

- Added a ROS 2 execution backend for the delivery gateway
  (`serve-delivery --backend ros2`, `flyto_robotics/ros2_delivery_runner.py`):
  each session runs the same `MissionController` under a fail-safe per-session
  node (`/flyto/cmd_vel`, `/flyto/odom`, `/flyto/scan`, one-second sensor
  freshness gate). Motion commands publish under the gateway lock so a
  shutdown stop can never be overtaken by an in-flight tick; the runner owns
  SIGINT so the final zero-velocity stop still reaches the robot on Ctrl+C;
  `--gazebo` switches the node to simulation time and labels the evidence.
  Terminal sessions persist `results/delivery-<session>.json` on every path,
  including gateway shutdown. `serve-delivery` installs explicit
  SIGINT/SIGTERM handlers so docker/systemd stops and background shells
  (which inherit SIGINT as ignored) still tear down through the safe-stop
  path. Verified in the ROS 2 Jazzy container: full delivery, mid-route
  safe-stop, and active-mission teardown under both signals. The default
  backend remains the deterministic planar simulation, now factored as
  `SimulatedDeliveryRunner`.
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
