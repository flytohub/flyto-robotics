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
- Capability selection now uses namespaced/versioned manifests, registry
  snapshot hashes, language-neutral Goal Frames, canonical
  intent/affordance/effect/event matching, runtime compatibility hard filters,
  a bounded shortlist, semantic coverage/clarification evidence, and post-plan
  shortlist enforcement. Raw aliases are legacy fallback only.
- Flyto AI can re-rank the transport-neutral manifest through its existing
  Blueprint and Core bridges without importing Robotics source.
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

- Headless Gazebo's rendering server may need SIGTERM after the mission node
  finishes and launch begins shutdown. Mission completion, the zero-velocity
  stop, and atomic result write finish before shutdown.

## Required before a competition field demo

- Obtain written BOM brand/manufacturing evidence for the chosen physical base.
- Connect the signer to Flyto authentication/RBAC and production key custody.
- Stabilize full physical camera line handoff before claiming visual routing.
- Validate hardware E-stop, watchdog, network loss, sensor loss, battery, and
  site-specific acceptance on the selected physical robot.
- Connect Flyto Cloud input capture to `input-event.v1` and validate keyboard
  and joystick arrival latency on the selected deployment network.

## Verification

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

```bash
make verify
python3 -m flyto_robotics.cli dry-run \
  examples/jobs/pharmacy-to-ward.json
```
