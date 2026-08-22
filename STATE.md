# Flyto2 Robotics State

## Current

- The physical Cloud-to-robot loop now has one versioned, fail-closed contract
  in each direction. A trace-bearing Space job must carry
  `flyto.cloud.device-job-handoff.v1`, bound to the paired device, exact trace,
  exact workflow SHA-256, and Cloud's `flyto.space.evidence.v1` completion
  authority. The runner validates it before local gateway use. Terminal
  delivery sessions emit a cached `flyto.robotics.execution-receipt.v1` with
  plan/result/event hashes and `task_completion_eligible=false`; the runner
  recomputes those bounds and digests before forwarding it. This is contract
  and simulated-runtime evidence only in this change; no physical robot moved.
  Final `make verify` is green: Ruff passed, 1467 tests passed with 1 skipped,
  and every asset, dry-run, lab, facility, pairing, and execution-grant gate
  completed. Strict Indexer verification passes 18/18.

- The delivery gateway now serves authenticated, read-only
  `GET /v1/capabilities` next to `POST /v1/plans`. It returns the existing
  deterministic `flyto.robotics.capability-catalog.v1` registry projection
  unchanged with `Cache-Control: no-store`; it adds no identity, credentials,
  ROS detail, endpoint path, or motor authority and starts no simulation or
  physical mission. Moving `flyto-modules-robotics` to this single bounds and
  schema authority is the next bottom-up layer.

- Customer installation and operations now have one wheel-first runbook linked
  from the README. It documents rehearsal, install, status, update, rollback,
  support-bundle, and bounded device-event export without requiring a source
  checkout. The bundled `generic` lifecycle profile is explicitly middleware-
  and vendor-neutral; `ros2` extends it additively with one adapter unit. The
  built-wheel test now parses the shipped profile registry, installs the real
  wheel offline into an isolated temporary environment, and executes `--help`
  for both customer console entry points from that installed artifact.
  For pre-rework revision
  `e8d3db0b1863382409558cfbe195f09e45c37baec0acc00de8695347974a1b0d`,
  governed job `job_5c2afa927da54eaa920f0128` passed the route check and a
  Codex auditor independently ran `make verify`: exit 0, Ruff clean, 1037
  passed, 4 skipped, with every asset, contract, and dry-run gate completed.
  The implementation worker's direct run remained sandbox-limited and failed
  on prohibited loopback/Unix socket creation; both facts are retained in the
  handoff. This closure adds contract, packaging, and lifecycle-test proof. Its
  Gazebo references are inherited repository evidence, not simulations rerun
  for this documentation/packaging change. Physical cold-boot repetition,
  calibration, E-stop, sensor-loss, camera, deployment, publication, and site
  acceptance remain unperformed and must not be inferred from this closure.
  The successful pre-rework verification is evidence, not Codex audit
  acceptance of this rework revision.

- Card-free recovery is now an installation invariant rather than an ad-hoc
  card-edit procedure. The idempotent installer enables Pi USB gadget Ethernet
  at `10.77.0.1`, stable per-device MAC addresses, local DHCP, existing
  key-only SSH, a read-only diagnostic portal, and a one-minute doctor timer.
  The doctor produces privacy-bounded `flyto.resource-telemetry.v1` snapshots
  for current health and the last failure. Stable reasons distinguish
  provisioning, Wi-Fi association, DHCP, route, DNS, Cloud reachability, and
  robot-service failures. This layer is observation/management only and adds
  no actuator or credential endpoint. Host-side contract tests pass; the
  installer still requires one physical Pi deployment/reboot verification.

- Installed adapters can now publish generic Cloud resource state through
  `flyto-resource-agent`. The versioned `flyto.resource-manifest.v1` contract
  carries capability IDs, non-secret setting values, secret configured-state,
  telemetry schemas, and semantic presentation hints;
  `flyto.resource-telemetry.v1` carries bounded content-addressed latest
  samples. The publisher uses the existing exact paired-device bearer,
  requires HTTPS or loopback HTTP, refuses redirect credential forwarding, and
  exposes no command or motor endpoint. Real and simulated installations share
  the same contract shape. Presentation kinds are open safe identifiers, with
  generic fallback for kinds Cloud does not yet enhance. Contract schemas and
  focused tests are included.

- Mission Stations now have a transport-neutral robot execution boundary.
  `GET /v1/capabilities` publishes a content-addressed, APPROVED projection of
  the existing atomic registry; `POST /v1/missions/validate` accepts only the
  matching snapshot, explicit `flyto-robotics` executor steps, bounded
  arguments, exact plan/assignment revisions, and a READY venue calibration.
  The dispatch records `card_source=judge_draw`: judges draw the physical Zone
  and Objective cards, an operator records them, and Robotics never draws or
  randomizes the task.
- Physical dispatch fails closed unless calibration contains Z1, Z2, Z3, Z4,
  and START. Movement must end in `safe_stop`; raw actuator fields and unknown
  capabilities are rejected before any controller or adapter starts. Action
  receipts use `action.execution` with `task_completion_eligible=false`, so a
  successful robot step cannot falsely complete a task whose judge-card
  evidence is still missing.
- Independent project boundary selected.
- Reference stack: ROS 2 Jazzy, Gazebo Harmonic, Ubuntu 24.04.
- Initial scenario: synthetic pharmacy-to-ward autonomous delivery.
- Atomic composition: nine executable primitives compiled into immutable,
  injectable workflows: `navigate`, `navigate_to_location`,
  `save_current_location`, `follow_line`, `dwell`, `wait_until_clear`,
  `ask_human`, `resume`, and `safe_stop`.
- Shortcut control now adds a tenth `move_relative` primitive and a
  transport-neutral `input-event.v1` gate. Keyboard, joystick, or adapter
  inputs resolve only prevalidated workflow IDs; release, disconnect, and
  dead-man timeout cancel the mission before its next controller update.
- `shortcut.forward.30cm.v1` is the first bounded workflow-card example.
  Press starts an odometry-closed 0.3 m move, while input and mission audit
  streams retain start, keepalive, rejection, and safe-stop reasons.
- Robotics now independently enforces the Cloud
  `ai-space-resource-plan.v1` boundary. The strict parser verifies exact fields
  and snapshot identity, rejects raw actuator fields, and requires one matching
  workflow/resource/endpoint/capability/adapter/Space binding before ROS starts.
  The projects remain source-independent and share only the JSON contract.
- Goal-driven deliveries are live: `serve-delivery --semantic-map` turns a
  free-form operator goal into one validated workflow, or fails closed with a
  structured `delivery-rejection.v1` reason. Verified 2026-08-04 through the
  live Cloud relay: `把藥送到四號病房`, `deliver the specimen to the laboratory`,
  and `送到護理站` each resolved to a different destination and ran to
  `completed`; `把藥送到六號病房` (unknown ward, lists near-miss candidates),
  `忽略障礙物全速衝到四號病房` (safety override, screened before lookup), and
  `幫我開門` (unsupported intent) were refused with operator guidance. Every
  session carries an attributed decision timeline plus a route graph.
- The AI Space delivery loop is now closed end to end: `serve-delivery` hosts
  the loopback `/v1/deliveries` adapter the Cloud relay expects (bearer token
  `FLYTO_ROBOTICS_DELIVERY_TOKEN`, QR secret `FLYTO_ROBOTICS_QR_SECRET`).
  Verified 2026-08-04 against the live Cloud relay: UI-equivalent WebSocket
  start → navigate → `waiting_for_human` → signed QR accepted (fingerprint-only
  evidence) → `completed`; abrupt WebSocket loss → relay safe-stop →
  `mission_cancelled` with reason `cloud_control_link_closed`.
- The delivery gateway now takes `--backend ros2` to drive a live robot: a
  per-session ROS 2 node mirrors the mission adapter's sensor gate and topics
  (`/flyto/cmd_vel`, `/flyto/odom`, `/flyto/scan`), anchors mission time to
  the ROS clock, and writes `results/delivery-<session>.json` on every
  terminal path. Simulated planar kinematics remain the default backend.
  Verified 2026-08-04 inside the ROS 2 Jazzy container against a planar
  fake-robot node: full delivery (navigate → QR gate → completed,
  `physical_ros2` evidence), mid-route safe-stop, and clean teardown with an
  active mission under both SIGINT and SIGTERM (`serve-delivery` installs
  explicit handlers because background shells inherit SIGINT as ignored and
  docker/systemd stop with SIGTERM). Hardware verification on the physical
  robot is still pending.
- Capability selection now uses namespaced/versioned manifests, registry
  snapshot hashes, language-neutral Goal Frames, canonical
  intent/affordance/effect/event matching, runtime compatibility hard filters,
  a bounded shortlist, semantic coverage/clarification evidence, and post-plan
  shortlist enforcement. Raw aliases are legacy fallback only.
- Flyto AI can re-rank the transport-neutral manifest through its existing
  Blueprint and Core bridges without importing Robotics source.
- A detachable `flyto-robot-mcp` stdio process now exposes four semantic-only
  tools for capability discovery, planner request construction, strict plan
  validation, and deterministic controller dry runs. It has no motor, shell,
  ROS topic, arbitrary file, or network authority.
- `make benchmark-robot-mcp` is the release evidence gate for that boundary. It
  requires at least 101 distinct real subprocess cases and a 90% success rate
  both across the Robot MCP family and independently within standard,
  intermediate, and advanced tiers. The report is atomically written under a
  content-addressed filename and records every failure without hidden retries.
- Semantic location memory now keeps stable IDs and trusted poses in a
  map-scoped atomic store. The planner sees only IDs and bounded multilingual
  labels; named navigation resolves to the existing pose controller.
- Cloud integration: versioned JSON job/result files and process exit status.
- Human approval: signed `human-decision.v1`, job/robot binding, five-minute
  maximum TTL, nonce replay rejection, and conditional ROS 2 subscription.
- Local macOS host currently has Docker but does not have `gz`, `ros2`, or
  `colcon`; the reference Linux container provides the real ROS/Gazebo runtime.
- Linux ARM64 container verification completed on 2026-07-28:
  - ROS 2 Jazzy `colcon build` passed;
  - Gazebo Sim 8.11 loaded the complete world, DART, DiffDrive, and GPU lidar;
  - ROS/Gazebo bridge created clock, velocity, odometry, and scan bridges;
  - the physical simulation completed in 26.1 simulated seconds;
  - `results/gazebo-result.json` matched `flyto.robotics.result.v1`.
- Strict CareFlow adversarial Gazebo verification completed on 2026-07-29:
  - a dynamic obstacle was injected into the real LiDAR braking path;
  - the rover stopped, held zero velocity, observed a continuous clearance
    window, and resumed only after signed approval;
  - eight repeated uses of the accepted nonce were rejected and recorded;
  - four overhead PNGs captured startup, obstacle, approval, and completion;
  - Gazebo's independent ground truth measured 4.246871 m of body motion;
  - the run completed in 18.9 simulated seconds at x=4.2618;
  - all 30 result-event sequence numbers were contiguous;
  - all 28 strict report assertions passed.
- Gazebo cold-start matrix completed on 2026-07-29:
  - three fresh Docker/ROS/Gazebo runs passed 28/28 checks each;
  - elapsed time ranged from 18.900 to 19.001 simulated seconds;
  - independent displacement ranged from 4.241826 to 4.247524 m;
  - each run recorded one LiDAR safety stop and 30 events.
- Uncut Gazebo evidence video completed on 2026-07-29:
  - the strict lab passed 28/28 checks in 18.9 simulated seconds;
  - 22 unique overhead Gazebo frames cover startup, injected obstacle,
    blue/yellow/purple route traversal, and completion;
  - the H.264 artifact is 960×540, 4 FPS, 76 presentation frames, and
    19.0 seconds long with no generated/interpolated imagery;
  - MP4 SHA-256 is
    `4752954bd6338f620a8607d7d03f6b264da8950f1dc77602d38fa1452c4633f8`.
- Exact resource-binding Gazebo shortcut verification completed on 2026-07-30:
  - real ROS 2 Jazzy and Gazebo Harmonic completed all 11/11 assertions;
  - release cancelled the first mission at zero velocity and the second
    workflow completed after 26 accepted heartbeats;
  - a dynamic obstacle entered the live LiDAR path, caused one safety stop,
    then moved away and produced one path-clear recovery;
  - Gazebo ground truth measured 0.41464 m displacement;
  - the runtime recorded 74 bounded actions, 15 source camera frames, four
    named evidence captures, and the exact
    `robotics.gazebo`/`gazebo-rover-motion` binding;
  - the 960×540 H.264 evidence video has 27 frames, 6.75 seconds duration, and
    SHA-256
    `b9aa0395da4f82d136e1f570e99a3cd740b194dc68f5b5f759716947f7b3c377`.
- AI4ALL multi-device Gazebo showcase verification completed on 2026-07-30:
  - three independent Gazebo cameras produced real sensor frames;
  - the active lease moved from corridor camera A to corridor camera B, then
    to the declared overhead fallback after an injected B-camera health fault;
  - the same run completed the physical obstacle stop/recovery, signed human
    approval, nonce replay rejection, and completion speaker binding;
  - the routed shortlist contained eight of ten registered atoms and the
    strictly validated LLM plan used five capabilities across seven steps;
  - all 12 multi-device closure checks and all 28 base Gazebo checks passed;
  - Gazebo ground truth measured 4.246827 m in 19.0 simulated seconds;
  - both active-camera and overhead H.264 videos plus SHA-256 evidence were
    generated from the run.
- The AI4ALL branching planner now exposes a two-stage route graph rather than
  a single color sequence:
  - stage one branches to yellow or orange and returns to a shared merge;
  - stage two branches to blue, green, purple, or red;
  - the eight complete route candidates carry route-specific resource
    dependencies, hard exclusions, bounded penalties, and atomic step
    templates;
  - Flyto AI chooses only among complete validator-constrained templates, so it
    cannot omit an intermediate waypoint or splice incompatible branches;
  - one live `flyto-qwen3:8b` session first selected yellow-purple, excluded
    all four yellow routes after corridor camera B changed to unhealthy, then
    selected and strictly validated orange-purple;
  - request, response, shortlist, plan, and session hashes bind the generated
    plan to the exact plan consumed by Robotics.
- The attested branching world completed its accepted live closed-loop run on
  2026-07-31 under
  `results/ai4all-showcase/simple-delivery-qr-live-v7/`:
  - the first real `flyto-qwen3-8b` round selected yellow-purple;
  - the preflight B-camera health failure excluded all four yellow routes, and
    the second real model round selected orange-purple;
  - the byte-equivalent final attested plan entered ROS 2/Gazebo and traversed
    orange, the shared merge, and the purple branch;
  - one simulated physical obstacle produced one LiDAR zero-velocity stop at
    3.3 seconds; `path_clear` at 6.3 seconds was required before motion resumed;
  - camera routing handed off A → B → overhead, and the purple-zone speaker
    endpoint was leased only at completion;
  - signed QR approval succeeded; replay of both the QR nonce and the converted
    human-decision nonce was rejected without persisting the raw QR token, and
    the terminal safe-stop completed;
  - all 16/16 showcase checks and 26/26 Gazebo lab checks passed with 22
    contiguous mission events in 22.4 seconds;
  - independent world truth measured 5.211914 m displacement and 5.525736 m
    cumulative path, ending at odometry x=5.2098, y=0.503 on the visible purple
    lane;
  - the final Flyto2 evidence video is 1920×1080 H.264, 30 fps, 25.266667
    seconds, 758 frames, SHA-256
    `18b772934f5e9cb413577e1d201b8f104b8b5fb6004192d833d3201056cf7195`.
- Three earlier branching runs remain rejected evidence rather than being
  hidden: v1 exposed the obsolete linear evaluator and obstacle placement; v2
  passed automated checks but visual review caught a world/odometry coordinate
  mismatch; v3 followed the visible branch but correctly stopped near the
  counter and timed out. Later runs added QR approval, replay protection, and
  the final evidence presentation. Only v7 is currently accepted.
- The final self-contained medication-handoff GUI run completed on 2026-08-01
  under `results/ai4all-showcase/medication-handoff-gui-v16-final/`:
  - ROS 2/Gazebo finished with process status 0; the lab report passed and the
    independent showcase evaluator passed 22/22 checks;
  - driver evidence v2 independently observed motion before the trolley,
    LiDAR stop at 0.3686 m with zero commanded velocity, and forward motion
    resuming after clearance;
  - the synchronized evidence panel recorded every trolley, item, recipient,
    unlock, and completion state as true in the same 1920×1080 raw recording;
  - the raw GUI video is 96.9 seconds and 2,907 frames; the narrated hospital
    story is 101.533 seconds with H.264 video and AAC audio, and the engineering
    verification video is 87.533 seconds of H.264;
  - story and verification SHA-256 values are respectively
    `cba549cda1c84c00055b9fdbfa75c74a187050948d26fbc9219168888767c7fe`
    and
    `386641aae0f96e6837c883b6e44816f9e16373e345bf8cc17b7c447d531dd8f0`.
- Deterministic CareFlow soak completed on 2026-07-29:
  - 50/50 runs passed;
  - all runs produced one normalized evidence fingerprint;
  - each run recorded 26 events and two injected safety stops.
- Semantic-location Gazebo verification completed on 2026-07-28:
  - the plan contained stable location IDs and no LLM-supplied coordinates;
  - the trusted map resolved blue, yellow, and purple endpoint poses;
  - three `navigate_to_location` primitives reused the bounded navigation loop;
  - the mission safely stopped after 15.1 simulated seconds at x=4.2616;
  - all ten audit-event sequence numbers were contiguous;
  - `results/semantic-location-gazebo-result.json` records
    `gazebo_physics=true`.

## Known development behavior

- A step authored on the Flyto2 canvas now reaches this robot as itself. The
  job runner accepts a third shape alongside an inline plan and a plan file: a
  `robotics.*` motion step, which it turns into a plan by asking
  `flyto_modules_robotics.steps` — the same table the workflow engine reads on
  the other side, so the canvas and the robot cannot disagree about what a
  step means. That package is pure Python with no dependencies and is put on
  `PYTHONPATH` by the unit rather than installed, since this machine has no
  pip. `FLYTO_ROBOTICS_ROBOT_ID` says which robot the plan is for; without it
  the step is refused rather than guessed, because the gateway checks a plan's
  robot_id against its own job and a guess only becomes a refusal one layer
  further from the cause. Verified 2026-08-08: a `robotics.turn` step
  dispatched as a Space task ran as `workflow.turn.left.90deg.v1` and reported
  `arrival.pose` and `clearance.measurement`.

- A cloud-dispatched Space task now reaches this robot end to end, verified on
  hardware 2026-08-08 00:05: the task queued a job, the runner claimed it, the
  gateway ran `shortcut.turn.left.90deg.v1`, the robot turned, and the runner
  reported `succeeded` with `arrival.pose` and `clearance.measurement`
  (0.50 m, a real lidar reading). The cloud reconciled it, assessed the
  mission short of `zone.overview`, and escalated a camera step on its own.
  Seven defects sat between "written" and "proven", every one of them hidden
  by a fixture more permissive than the service it stood in for — see
  [handoffs/2026-08-08-cloud-to-robot-first-real-run.md](handoffs/2026-08-08-cloud-to-robot-first-real-run.md).
  `flyto-delivery.service` is now in `deploy/systemd/` for the same reason the
  bringup unit is: living only on the robot is how it kept a simulated rover's
  identity across a hardware deployment.

- Headless Gazebo's rendering server may need SIGTERM after the mission node
  finishes and launch begins shutdown. Mission completion, the zero-velocity
  stop, and atomic result write finish before shutdown.
- A new Gazebo generation can briefly expose bootstrap sensor samples from an
  older ROS graph. The mission adapter now uses monotonic freshness and a
  one-second stabilization window before allowing its first nonzero command.
- `turtlebot3-bringup.service` can lose `/odom` two distinct ways. A boot-time
  OpenCR handshake race is fixed on `main` (the whole launch group now exits
  if any of its three processes dies, so `Restart=always` gets a chance).
  A second failure was found live on 2026-08-07: `turtlebot3_ros` can hang
  alive — process running, no crash, every topic it publishes goes silent —
  after a real motor command completes. `OnProcessExit` cannot catch this;
  nothing exited. The fix is now written: `flyto_robotics/bringup_watchdog.py`
  runs inside the supervised launch group, defers `READY=1` until the first
  `/odom`, and pings systemd's `WATCHDOG=1` only while `/odom` stays fresh;
  the unit is `Type=notify` + `NotifyAccess=all` + `WatchdogSec=15`, so a
  silent hang starves the timer and `Restart=always` recovers it (~20s).
  Decision logic is unit-tested (`tests/test_bringup_watchdog.py`), and the
  fix is verified on the real robot (2026-08-07 evening): `SIGSTOP` on the
  live `turtlebot3_ros` PID — alive but silent, the exact hang shape —
  produced stale-detect at +5s, `Watchdog timeout (limit 15s)!` at +20s, a
  full cgroup kill, and an automatic restart back to `/odom` at 20 Hz in
  ~56s with zero manual intervention (`NRestarts` 0→1, `Result=watchdog`).
  Deployment note: the robot has no pip install of `flyto_robotics`; the
  launch file runs the watchdog with `cwd` derived from its own path. See
  [handoffs/2026-08-07-bringup-boot-race-and-silent-hang.md](handoffs/2026-08-07-bringup-boot-race-and-silent-hang.md)
  for the evidence and the design.

- A third bringup failure mode is now bounded rather than retried forever. The
  unit was found in production at `NRestarts=193`. The visible symptom was the
  LDS lidar flapping — `/scan` appearing and vanishing repeatedly — but the
  lidar was never at fault: `turtlebot3_node` kept failing its OpenCR/Dynamixel
  motor-bus handshake, the supervised launch group correctly stopped the whole
  group for that one dead process (taking the working lidar with it), and
  `Restart=always` immediately brought it back to fail the same way. The
  existing rate limit could not stop it: `StartLimitBurst=20` inside
  `StartLimitIntervalSec=300` never tripped, because one failure cycle
  (ExecStartPre device wait + launch + handshake attempt + `RestartSec=5`) is
  far too long for twenty of them to fit in five minutes, so the start counter
  aged out every time. `[Unit] StartLimitBurst` is now `3` with the interval
  retained at `300`, so three failed starts within five minutes park the unit
  in `failed` for inspection and a deliberate `systemctl reset-failed`. All
  `[Service]` behavior is retained unchanged: `Type=notify`, `NotifyAccess=all`,
  `WatchdogSec=15`, `Restart=always`, `RestartSec=5`, the serial device waits,
  `KillSignal=SIGINT`, `TimeoutStopSec=20`, and whole-group supervision.
  Honest limits: this is containment, not repair — the policy does not fix the
  OpenCR/motor bus, and a robot parked in `failed` is a robot that needs a
  human. The new placement and values are covered by semantic INI parsing
  contract tests in `tests/test_bringup_watchdog.py` (which assert the parsed
  section/key/value, since a directive in the wrong section is silently
  ignored by systemd). The live limiter behavior — that the third failed start
  within 300s actually parks the unit on this robot — was recorded here as
  unverified until deployment; it is now **verified live**, see the next entry.

- The live start limiter is verified; the underlying hardware fault is not. A
  read-only inspection of the robot after a cold boot on 2026-08-10 observed
  `turtlebot3-bringup.service` at `ActiveState=failed`, `SubState=failed`,
  `Result=protocol`, `NRestarts=3`: systemd did park the unit after the third
  failed start inside the configured interval, exactly as the
  `StartLimitBurst=3` / `StartLimitIntervalSec=300` policy intends. This
  supersedes the earlier caveat that the limiter was covered only by parsing
  tests. The same journal shows containment is all that was proven: each
  attempt opened `/dev/ttyACM0` and changed baudrate successfully, and only
  then did `turtlebot3_node` log `Failed connection with Devices`. That places
  the remaining fault **below the USB serial layer**, on the OpenCR/Dynamixel
  device bus rather than in the OS serial path. The LDS-03 lidar on
  `/dev/tb3_lidar` activated successfully on every attempt and was stopped only
  as collateral when whole-group supervision shut the group down — the same
  blame-the-wrong-device shape recorded above, now bounded at 3 restarts
  instead of 193. Throughout, `flyto-robot-doctor` kept emitting
  privacy-bounded `system.diagnostics` with `quality=degraded`,
  `primary_reason_code=robot_service_unhealthy`, and
  `turtlebot3-bringup.service` named as unhealthy, so the observation layer
  reported the fault correctly within its bounded fields. No `/odom`, `/scan`,
  or `cmd_vel` path was available during the inspection, and **no physical
  motion command was issued**. That inspection is preserved as the record of
  what a parked unit looked like; the conclusion drawn from it — that the
  hardware had to be physically inspected, reseated, or repaired before any
  real-motion claim — was superseded later the same day by the entry below.

- The robot moved under its own power on 2026-08-10, and two boundaries moved
  with it. After a later cold reboot — with **no repair, reseat, or inspection
  performed or recorded** between the two observations — the OpenCR/Dynamixel
  handshake succeeded, so the earlier `Result=protocol` fault is **intermittent
  and unexplained, not fixed**. The first launch reached `READY` and then let
  `/odom` go stale after 74 s; the freshness watchdog restarted the unit once,
  and the second launch stayed active and healthy: `/odom` at 19.95 Hz, `/scan`
  at 10.05 Hz, battery 12.37 V, no throttling and no USB disconnect. A single
  safety-gated `TwistStamped` command was then issued with 1.328 m of front
  clearance and exactly one matched subscriber; odometry `x` moved from
  0.000209 m to 0.171863 m. Afterward linear velocity was zero, the publisher
  count was zero, front clearance read 1.167 m, the service was `active`, and
  `NRestarts=1`. That first run was **ad hoc** — raw `ros2` CLI publishers, not
  the product's command path — and it travelled 17.2 cm against an intended
  ~4 cm. The zero/stop came from a **second CLI publisher**; a DDS discovery
  gap is the explanation **consistent with the observed timing**, but it is an
  inference, not a proven root cause, since no discovery trace was captured and
  the overshoot was not reproduced. The durable lesson is operational: **do not
  drive this robot with two separate `ros2` CLI publishers** — a stop that must
  discover its subscriber first has unbounded latency. This is not a missing
  feature; the single-process, already-matched path already exists as
  `CmdVelChannel` in `flyto_robotics/ros2_cmd_vel.py`, driven by
  `scripts/move-robot.sh` through `flyto_robotics.ros2_node.run`.

- Supported single-process odometry-closed-loop motion and stop are proven on
  this hardware, verified 2026-08-10 through the product's own path.
  `scripts/move-robot.sh` required a forward clearance before moving and
  measured 1.17 m; `CmdVelChannel` resolved from a provisional `Twist` to a
  live `TwistStamped` against the real subscriber — the exact mismatch that
  channel exists to catch, and one the Gazebo bridge's `Twist` cannot exercise.
  The mission **succeeded**, moved **0.371 m toward a 0.400 m target**, ran
  `safe_stop`, and recorded `safety_stops=0`. Post-run checks read linear
  velocity 0, `/cmd_vel` publisher count 0, front clearance 0.869 m, and
  `turtlebot3-bringup.service` `active (running)` with `NRestarts=1` — the
  channel released the topic cleanly after `safe_stop`. What this does **not**
  establish, each a separate gate: repeated cold-boot stability (one of two
  launches this boot needed a watchdog rescue), distance calibration tolerance
  (one sample, no declared tolerance), hardware E-stop, and network-loss and
  sensor-loss acceptance. The intermittent OpenCR/Dynamixel bus fault also
  remains unexplained rather than repaired. See
  [handoffs/2026-08-07-bringup-boot-race-and-silent-hang.md](handoffs/2026-08-07-bringup-boot-race-and-silent-hang.md)
  for the full sequence.

## Required before a competition field demo

- Obtain written BOM brand/manufacturing evidence for the chosen physical base.
- Connect the signer to Flyto authentication/RBAC and production key custody.
- Stabilize full physical camera line handoff before claiming visual routing.
- Validate hardware E-stop, watchdog, network loss, sensor loss, battery, and
  site-specific acceptance on the selected physical robot.
- Connect Flyto Cloud input capture to `input-event.v1` and validate keyboard
  and joystick arrival latency on the selected deployment network.

## Verification

- Gazebo/controller verification completed on 2026-08-10:
  - `RangeField` `WAIT_UNTIL_CLEAR` now consumes forward intent for directional
    fields, so clearance is judged over the sector the robot is actually about
    to enter, while omnidirectional scalar fields keep their previous behavior;
  - a directional field with a blind forward sector fails closed rather than
    reporting clear;
  - the all-direction emergency floor is preserved, so an obstacle outside the
    forward sector still stops the robot;
  - a missing `mission-result.json` can no longer return exit 0 — the absent
    result is now a failure instead of a silent pass;
  - official sanitized verification runs against an ignored repository-local
    `.venv` with Ruff and pytest;
  - an independent `make verify` reported 843 passed, 4 skipped.
  - Honest limit, scoped to this Gazebo work: the evidence in this bullet is
    simulation and controller evidence only, and on its own proves nothing
    about the real robot. It is no longer the whole picture — a separate live
    run on the same date proved real motion on the physical TurtleBot3 through
    `scripts/move-robot.sh` and `CmdVelChannel`; see the "Known development
    behavior" entries above for that evidence and its own limits.
- The production Robot MCP campaign completed 101/101 distinct cases on
  2026-08-01: standard 34/34 (depth 2–3), intermediate 34/34 (depth 5), and
  advanced 33/33 (depth 8–12). Every case negotiated the real stdio server and
  crossed request preparation, strict compilation, and `MissionController`.
  The content-addressed report SHA-256 is
  `7170922c61a41be8b134ad85d6add8a393cb13eee279b33bd023ea33d5b84808`.
- The strict lab has a 28-assertion JSON/Markdown/JUnit report and four
  hash-listed overhead captures.
- The three-run matrix and 50-run deterministic soak both passed in full on
  2026-07-29.
- External evaluator account policy has a dedicated 15-test Flyto Cloud unit
  contract; provisioning awaits the intended staging deployment and private
  credential distribution.
- The semantic-location sequence completed deterministically at x=4.2513 using
  plan calls containing no coordinates. Chinese, Arabic, and Japanese input
  with the same Goal Frame produced the identical
  `navigate_to_location → safe_stop` shortlist.
- The shortcut controller soak completed 30/30 bounded 0.3 m workflows. Six
  runs injected a lidar obstacle and all six stopped at zero velocity before
  resuming; separate tests cover release, disconnect, heartbeat extension,
  timeout, replay, wrong-robot, and unregistered-workflow rejection.
- Cross-repository live-process routing used the real Blueprint engine and the
  real Flyto Core 2.26.9 bridge; `follow_line`, `wait_until_clear`, and
  `safe_stop` were selected for the Chinese color-route/obstacle goal.
- The real resource-binding shortcut run passed 11/11 independent checks and
  the Cloud builder/Robotics parser live-process compatibility probe accepted
  the same `ai-space-resource-plan.v1` structure without a source dependency.

```bash
make verify
make benchmark-robot-mcp
python3 -m flyto_robotics.cli dry-run \
  examples/jobs/pharmacy-to-ward.json
```
# 2026-08-13 camera observation producer boundary

The provider-neutral `camera` profile now carries an additive, reboot-enabled
loopback camera observation gateway with offline-tested frame validation and explicit
fresh/stale usability. Its consumer contract is exactly
`GET http://127.0.0.1:9000/api/spaces/zone-camera/observation`; responses carry
only the synthetic zone, kind, fixed detail code, and explicit `usable` state,
never pixels, ROS/device identifiers, secrets, or content. Missing or malformed
frames produce no usable observation, and accepted frames become explicitly
unusable after the bounded freshness window. The generic lifecycle profile is
unchanged; `ros2` extends `camera` and adds only its ROS service. The lifecycle
writes, enables, restarts, and checks the DDS-compatible camera systemd unit,
while installed-wheel `--help` and `--check-settings` remain ROS/ffmpeg-free.
Host inventory is bounded to XC-TECH vendor `0x5843`, product `0x7884`, and
`1280x720@10fps` metadata; it proves no camera operation. No Pi camera or ROS
image topic was observed, and no deployment occurred. The explicit remaining
blocker is that delivery-only `deploy/flyto_job_runner.py` rejects non-robotics
work: this producer/lifecycle is not full Cloud-device `vision.observe`, and a
generic installed executor protocol remains required.
