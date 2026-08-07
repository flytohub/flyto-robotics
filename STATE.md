# Flyto2 Robotics State

## Current

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

## Required before a competition field demo

- Obtain written BOM brand/manufacturing evidence for the chosen physical base.
- Connect the signer to Flyto authentication/RBAC and production key custody.
- Stabilize full physical camera line handoff before claiming visual routing.
- Validate hardware E-stop, watchdog, network loss, sensor loss, battery, and
  site-specific acceptance on the selected physical robot.
- Connect Flyto Cloud input capture to `input-event.v1` and validate keyboard
  and joystick arrival latency on the selected deployment network.

## Verification

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
