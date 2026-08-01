# Flyto Robotics

Flyto Robotics is an AI-native, composable capability runtime for robots.
A person describes an outcome in natural language; an LLM or agent selects
registered atomic abilities and returns a versioned plan; a strict validator
compiles that plan into deterministic, safety-bounded execution.

“先走藍線，再走黃線，最後走紫線並安全停止” is the first visible example,
not the product boundary. The same architecture can compose navigation,
inspection, recognition, manipulation, speech, human approval, recovery, and
C/C++/Python-backed tools.

The repository also retains its first synthetic hospital delivery mission:

1. receive a versioned job;
2. navigate from the charging area to the pharmacy;
3. confirm pickup after a configurable dwell;
4. navigate to a ward;
5. stop for obstacles and resume when the path is clear;
6. write a machine-readable result.

The repository is deliberately independent from Flyto Cloud. Cloud can dispatch
the example JSON as a device job, but neither project imports the other.

## What is included

- a self-contained Gazebo Harmonic hospital world;
- a differential-drive rover with lidar and odometry;
- a ROS 2 Jazzy bridge and mission-controller launch file;
- `flyto.robotics.job.v1`, `plan.v1`, and `result.v1` JSON Schemas;
- the strict `ai-space-resource-plan.v1` boundary that binds an exact
  workflow, resource, endpoint, adapter, capability, Space, and lease before a
  ROS/Gazebo/physical adapter may start;
- versioned capability-manifest and capability-route schemas;
- a signed `human-decision.v1` contract for short-lived approval messages;
- executable `navigate`, `navigate_to_location`, `move_relative`,
  `save_current_location`, `follow_line`, `dwell`, `wait_until_clear`,
  `ask_human`, `resume`, and `safe_stop` primitives;
- a workflow-card shortcut gate for keyboard, joystick, or adapter inputs with
  press, heartbeat, release, disconnect, and dead-man timeout handling;
- a machine-readable capability registry exposed to AI planners;
- plan-level checks for terminal safe stop, consistent line transitions, and
  paired human approval/resume gates;
- a provider-neutral AI request protocol and HTTPS planner adapter;
- a downward route camera and color-line perception adapter;
- a blue/yellow/purple Gazebo route world and AI-plan launch file;
- an adversarial Gazebo lab with dynamic obstacle injection, signed approval,
  nonce replay attempts, overhead captures, and independent world-pose truth;
- an eight-route branching showcase with attested live planning, deterministic
  resource-dependency exclusion, bounded replanning, and exact route templates;
- strict JSON, Markdown, and JUnit lab reports plus repeated-run aggregation;
- immutable workflows composed from those primitives;
- a deterministic controller shared by dry-run and ROS execution;
- tests that run without ROS 2 or Gazebo;
- asset validation for JSON, XML/SDF, bridge configuration, and launch files.

## Installation

For contract tests and deterministic development, install Python 3.9 or newer,
`pytest`, and `ruff`. The Gazebo runtime requires Ubuntu 24.04, ROS 2 Jazzy,
and the Harmonic packages listed under “Run the Gazebo demo.” No service
credentials or environment secrets are required.

## Usage

### Quick verification

The local contract and controller checks need only Python 3.9 or newer:

```bash
make verify
make soak
make gazebo-lab
make gazebo-matrix
make gazebo-shortcut
make ai4all-showcase
python3 -m flyto_robotics.cli validate-job \
  examples/jobs/pharmacy-to-ward.json
python3 -m flyto_robotics.cli dry-run \
  examples/jobs/pharmacy-to-ward.json
python3 -m flyto_robotics.cli show-capabilities
python3 -m flyto_robotics.cli validate-plan \
  examples/plans/blue-yellow-purple.json
python3 -m flyto_robotics.cli dry-run-plan \
  --job examples/jobs/pharmacy-to-ward.json \
  --plan examples/plans/blue-yellow-purple.json
python3 -m flyto_robotics.cli dry-run-plan \
  --job examples/jobs/pharmacy-to-ward.json \
  --plan examples/plans/careflow-human-gate.json
python3 -m flyto_robotics.cli validate-plan \
  examples/plans/shortcut-forward-30cm.json
python3 -m flyto_robotics.resource_binding \
  examples/resource-plans/gazebo-shortcut-forward-30cm.json \
  --workflow shortcut.forward.30cm.v1 \
  --resource flyto-rover-sim-001 \
  --capability mobility.move_relative \
  --adapter robotics.gazebo \
  --space gazebo-lab \
  --confirmed
```

`dry-run` executes the same controller against deterministic planar kinematics.
It proves the mission state transitions and result envelope; it does not claim
Gazebo physics evidence.

`make ai4all-showcase` first requests and verifies an attested initial plan and
resource-triggered replan, then executes that exact final plan in the
multi-camera hospital world. It injects obstacle and camera faults, records
active-resource handoff video, and fails unless all Physical AI closure checks
pass. A loopback Flyto AI planner must be running and its URL is supplied
through `FLYTO_ROBOTICS_PLANNER_URL`; the showcase never labels a fixture as a
live model result. See
[`docs/AI4ALL_SHOWCASE.md`](docs/AI4ALL_SHOWCASE.md) for the product narrative,
truth boundary, and evidence layout.

## Atomic and composable by default

`hospital_delivery.v1` is a workflow composition, not a hard-coded monolith:

```text
navigate.pickup
  → dwell.pickup
  → navigate.dropoff
  → dwell.dropoff
```

An AI-composed plan uses the same interpreter:

```text
goal + observations
  → language/modality adapter emits flyto.goal-frame.v1
  → runtime/robot/sensor/permission hard filters
  → exact intent/affordance/effect/event rank + bounded shortlist
  → trusted Flyto Blueprint hints + Flyto Core discovery metadata
  → LLM-selected plan JSON from the shortlist only
  → schema and registry validation
  → immutable WorkflowPlan
  → deterministic controller
  → ROS 2 → Gazebo or a physical robot
```

Each atom has a stable namespaced ID such as
`robotics.vision.follow_line@1`, while plans continue to emit the backwards
compatible executable name `follow_line`. The route records the registry
SHA-256 snapshot, Goal Frame, semantic coverage, candidate scores, reasons,
confidence, hard-filter exclusions, and whether clarification is required.
The natural-language string is never used for ranking when a Goal Frame is
present. Catalogs larger than eight atoms are reduced before provider dispatch
by default. A plan that selects an atom outside that exact shortlist is
rejected.

Semantic locations use the same separation:

```text
“記住這裡是護理站”
  → Goal Frame selects save_current_location
  → current trusted odometry is stored under a stable location ID

“去護理站”
  → Goal Frame selects navigate_to_location
  → LLM emits only the registered location ID
  → validator resolves its pose from the trusted map
  → existing deterministic navigate controller executes it
```

Labels are bounded Unicode and may contain any writing system. They are display
and language-understanding metadata, never the map key. The stable
`location_id` is independent of the label. The LLM receives a versioned
location catalog containing IDs and labels but no `x`, `y`, or `yaw`; those
coordinates remain in the map-scoped store and are resolved only during trusted
workflow compilation. Unknown IDs, a mismatched physical `map_id`, stale
revisions, malformed text, and oversized maps fail closed.

The two location atoms are additive. Existing atom manifests, arguments,
controller behavior, and workflows do not change. They are excluded from
zero-score legacy fallback, so adding them cannot displace an unrelated old
atom merely because the registry became larger. Goal Frame routing includes
only positive semantic matches and still pins `safe_stop` for motion.

The LLM decides among a bounded set of executable semantic routes. When route
candidates are present, the structured-output Schema has one complete branch
per candidate, so a chosen route cannot omit an intermediate semantic
location, mix two paths, or drop a required approval/stop atom. The independent
validator still verifies that exact sequence after model output. The model
cannot emit wheel PWM, arbitrary ROS topics, shell commands, or unregistered
tools. Steering, speed clamps, lidar stopping, sensor freshness, timeouts, and
emergency behavior remain deterministic.

Motion plans are rejected unless their final capability is `safe_stop`.
Shortcut controls resolve only a registered workflow ID. They cannot carry
linear or angular velocity fields. `release`, input disconnect, or a missing
heartbeat cancels the active mission before the next controller update, while
the input and mission event streams preserve the reason for audit.

Physical resource selection is a separate trust boundary. The LLM chooses a
verified workflow from a bounded shortlist; it does not choose a concrete
camera, robot, elevator, ROS node, or vendor adapter. Flyto2 Cloud freezes that
decision in `ai-space-resource-plan.v1` only after capability, Space,
permission, health, freshness, confirmation, priority, and lease checks. This
repository independently parses the language-neutral contract and requires one
exact matching binding before the ROS node starts. Unknown fields, a changed
snapshot, workflow mismatch, wrong adapter, wrong Space, missing confirmation,
or raw motor fields fail closed.

The two repositories do not import one another. Their shared boundary is the
versioned JSON contract, allowing the same control plane to bind a Python,
C/C++, ROS 2, browser-media, ONVIF, or vendor SDK adapter without growing the
robot controller into a device catalog.
`ask_human` always holds zero velocity and cannot be satisfied by the planner:
the controller requires an explicit matching decision from an identified
external actor. A later `resume` fails closed unless that approval exists.
Mission events include monotonic sequence numbers and structured step,
capability, and actor fields for audit and replay.

Human decisions cross an additional trust boundary. A message is accepted only
when its HMAC-SHA256 signature is valid, its job and robot match the active
mission, its lifetime is at most five minutes, and its nonce has not been used.
The signing secret is read only from `FLYTO_ROBOTICS_APPROVAL_SECRET`. It is
required only for plans containing `ask_human`.

New primitives should have one responsibility and a deterministic test before
being exposed to workflows. New hardware should implement the existing command
and observation boundary instead of changing mission contracts.

## API and contracts

The stable external API is file-based and language-neutral:

- `contracts/job-v1.schema.json` validates input jobs;
- `contracts/plan-v1.schema.json` validates AI-composed capability plans;
- `contracts/input-event-v1.schema.json` validates keyboard, joystick, and
  external input lifecycle events without accepting motor values;
- `contracts/facility-resource-plan-v1.schema.json` documents the exact
  Cloud-to-adapter resource binding and immutable snapshot;
- `contracts/capability-manifest-v1.schema.json` describes discoverable atoms;
- `contracts/capability-route-v1.schema.json` records shortlist evidence;
- `contracts/goal-frame-v1.schema.json` separates language understanding from
  capability selection;
- `contracts/semantic-location-map-v1.schema.json` protects the full,
  map-scoped ID/label/pose store;
- `contracts/semantic-location-catalog-v1.schema.json` defines the
  coordinate-free view exposed to planning;
- `contracts/ros2-adapter-manifest-v1.schema.json` binds registered semantic
  capabilities to cancellable ROS 2 actions without exposing that graph to AI;
- `contracts/ros2-runtime-snapshot-v1.schema.json` carries content-addressed
  action availability, lifecycle, freshness, and emergency-stop evidence;
- `contracts/result-v1.schema.json` validates terminal evidence;
- `contracts/human-decision-v1.schema.json` validates signed approval envelopes;
- `flyto-robotics validate-job` validates before motion;
- `flyto-robotics dry-run` executes deterministic closed-loop kinematics;
- `flyto-robotics run-ros` starts the ROS adapter for one job.
- `flyto-robotics plan-ai` calls an HTTPS planner, then validates and atomically
  writes its returned plan.

### Flyto2 + ROS 2 semantic pairing

Flyto2 owns language understanding, capability planning, resource policy, and
audit evidence. ROS 2 owns deterministic execution, feedback, cancellation,
lifecycle, and the emergency stop. The model never receives ROS graph names or
actuator values.

The standard profile maps the existing executable navigation atoms to Nav2's
cancellable `NavigateToPose` action. Every adapter must declare both simulation
and hardware support, so the mission contract cannot fork between Gazebo and a
physical robot. MoveIt 2 and ros2_control action types are allowlisted for later
adapters, but a manifest is rejected until its Flyto capability is actually
registered; the pairing report therefore cannot claim an integration that the
runtime cannot execute.

Before execution, the readiness gate requires an exact profile snapshot, robot
identity, fresh evidence, an available interface, active lifecycle nodes, and
an independent emergency stop. One failed check removes all execution
authority.

```bash
python3 -m flyto_robotics.cli verify-ros2-pairing \
  --manifest examples/ros2-adapters/flyto2-standard.json \
  --runtime examples/ros2-runtime/ready-sim.json \
  --at 2026-08-01T10:00:00Z
```

`--at` exists for deterministic evidence replay. Live checks omit it and use
the current UTC time. Robot MCP exposes only the redacted profile and readiness
report through `robot.ros2.profile` and `robot.ros2.readiness.verify`.

```bash
export FLYTO_ROBOTICS_PLANNER_URL=https://planner.example.com/v1/robot-plan
export FLYTO_ROBOTICS_PLANNER_TOKEN=...
python3 -m flyto_robotics.cli plan-ai \
  --goal "先走藍線，再走黃線，再走紫線，遇到障礙先停下" \
  --robot-id flyto-rover-sim-001 \
  --output results/ai-plan.json
```

The token is read from the environment and is never written to plan or result
files. HTTP is accepted only for loopback development.

The default `atomic_ai_demo.launch.py` uses the validated waypoint plan
`blue-yellow-purple-waypoints.json`; it is the completed Gazebo physics
baseline. `blue-yellow-purple.json` exercises the camera `follow_line` atom in
deterministic observation tests. Full physical curved-line handoff remains an
explicit next milestone rather than a claimed result.

`careflow-human-gate.json` is the hospital workflow MVP. Its deterministic run
injects an obstruction, waits for a continuous clear window, requests a human
decision, records `demo.operator` approval, verifies `resume`, and completes
with `safe_stop`.

`careflow-waypoints-human-gate.json` uses the same human gate with waypoint
navigation, so it can run in the completed Gazebo physics baseline. The ROS
node subscribes to `/flyto/human_decision` only when the selected plan contains
`ask_human`.

The original ARM64 audit baseline completed this plan in 34.4 simulated
seconds. The stricter adversarial lab now adds a dynamically injected LiDAR
obstacle, four overhead captures, an independent Gazebo world-pose oracle, and
28 explicit report assertions. The verified run completed in 18.9 simulated
seconds with 30 contiguous events and 4.246871 m of actual world displacement.
Three independent cold starts all passed, and a 50-run deterministic soak
produced one normalized fingerprint with zero failures. A fresh video run also
captured the complete 18.9-second Gazebo mission as a 960×540 H.264 artifact
with the dynamic obstacle, blue/yellow/purple traversal, and terminal stop. See
`docs/testing/TEST_RESULTS_2026-07-29.md`.

Create and publish a short-lived signed decision from a second terminal:

```bash
python3 -m flyto_robotics.cli sign-human-decision \
  --job examples/jobs/pharmacy-to-ward.json \
  --approval-id delivery.nurse_station \
  --actor-id operator.demo \
  --approve \
  --output results/human-decision.json

decision="$(python3 -m flyto_robotics.cli sign-human-decision \
  --job examples/jobs/pharmacy-to-ward.json \
  --approval-id delivery.nurse_station \
  --actor-id operator.demo \
  --approve)"
ros2 topic pub --once /flyto/human_decision std_msgs/msg/String \
  "{data: '$decision'}"
```

The environment secret must be present in both the mission-controller process
and the trusted signer. In production, end users must not receive the shared
key: an authenticated Flyto gateway should apply RBAC, derive the actor ID, and
sign the envelope after approval.

Python composition types are exported from `flyto_robotics`: `WorkflowStep`,
`WorkflowPlan`, `PrimitiveKind`, `MissionController`, and the initial
`hospital_delivery_workflow` compiler.

### Semantic location example

Build the exact provider-neutral request Flyto AI sends to a planner:

```bash
python3 -m flyto_robotics.cli planner-request \
  --goal "先去藍線終點，再去黃線終點，最後去紫線終點並安全停止" \
  --robot-id flyto-rover-sim-001 \
  --goal-frame examples/goal-frames/semantic-location-sequence.json \
  --semantic-map examples/maps/atomic-color-route.json \
  --semantic-map-id gazebo.atomic-color-route.v1
```

The returned shortlist is exactly `navigate_to_location`, `safe_stop`.
`observations.semantic_map` contains multilingual labels and stable IDs but no
pose. Execute the same location-ID-only plan deterministically:

```bash
python3 -m flyto_robotics.cli dry-run-plan \
  --job examples/jobs/pharmacy-to-ward.json \
  --plan examples/plans/semantic-location-sequence.json \
  --semantic-map examples/maps/atomic-color-route.json \
  --semantic-map-id gazebo.atomic-color-route.v1
```

`examples/plans/teach-current-location.json` demonstrates the stationary
write atom. Its pose is always supplied by current odometry, not the model.

The semantic-location sequence was also run in the reference ROS 2
Jazzy/Gazebo Harmonic 8.11 container. All three named-location primitives and
the final stop completed in 15.1 simulated seconds at x=4.2616. The result
records `gazebo_physics=true`, four expected primitive starts, and ten
contiguous audit events:
`results/semantic-location-gazebo-result.json`.

## Run the Gazebo demo

The supported reference environment is Ubuntu 24.04 with ROS 2 Jazzy and
Gazebo Harmonic:

```bash
sudo apt update
sudo apt install \
  ros-jazzy-ros-gz-bridge \
  ros-jazzy-ros-gz-sim \
  python3-colcon-common-extensions
source /opt/ros/jazzy/setup.bash

colcon build --symlink-install
source install/setup.bash
ros2 launch flyto_robotics hospital_demo.launch.py
ros2 launch flyto_robotics atomic_ai_demo.launch.py
```

The launch file starts:

- `worlds/hospital-logistics.sdf`;
- the `flyto_rover` differential-drive model;
- `ros_gz_bridge` for `/clock`, command velocity, odometry, and lidar;
- `mission_controller`, using the bundled pharmacy-to-ward example.

Override the job or output file:

```bash
ros2 launch flyto_robotics hospital_demo.launch.py \
  job_file:=/absolute/path/job.json \
  result_file:=/absolute/path/result.json
```

For a server-only smoke, append `headless:=true`.

Run the semantic-location plan in the same Gazebo world:

```bash
ros2 launch flyto_robotics atomic_ai_demo.launch.py \
  plan_file:="$(ros2 pkg prefix flyto_robotics)/share/flyto_robotics/examples/plans/semantic-location-sequence.json" \
  semantic_map_file:="$(ros2 pkg prefix flyto_robotics)/share/flyto_robotics/examples/maps/atomic-color-route.json" \
  semantic_map_id:=gazebo.atomic-color-route.v1
```

### Reproducible container

On macOS or another machine without ROS 2, build the reference environment:

```bash
docker build \
  -t flyto-robotics:jazzy-harmonic \
  -f docker/Dockerfile.jazzy .
```

The image contains ROS 2 Jazzy, Gazebo Harmonic's ROS integration, the bridge,
and ffmpeg for evidence-video encoding. Mount this repository, run
`colcon build`, and launch with `headless:=true`. The container supports both
ARM64 and AMD64 base images.

### Adversarial lab and evidence

Run one strict lab or the default three-run cold-start matrix:

```bash
make gazebo-lab
make gazebo-matrix
make gazebo-video
make gazebo-shortcut
```

Each run injects a real dynamic obstacle, verifies a LiDAR stop, removes the
obstacle, publishes one valid signed approval, attempts eight nonce replays,
and requires an authorized resume and final safe stop. Gazebo's own world-pose
publisher independently proves the rover body moved at least 3.8 m; controller
odometry alone is not accepted.

`make gazebo-video` runs the same strict lab, continuously samples the Gazebo
overhead camera, and writes `gazebo-careflow.mp4`, `video-probe.json`, and an
MP4 SHA-256 file beside the normal JSON/Markdown/JUnit evidence. The output
timeline is calibrated to the measured simulated mission duration; repeated
presentation frames are used when the ROS sensor-data QoS drops camera frames,
without generating or interpolating new visual content. Raw frames and videos
remain ignored build evidence.

The complete plan, measured results, image inventory, and external evaluator
walkthrough are indexed in `docs/testing/README.md`.

### Workflow-card shortcut closed loop

`make gazebo-shortcut` exercises the same boundary used by a Flyto2 AI Space
workflow card. It sends `press`, bounded `heartbeat`, `release`, and a second
`press` as versioned input events; it never sends velocity or motor fields. The
ROS adapter resolves `keyboard.main/ArrowUp` to the one validated workflow ID,
executes that immutable plan, and publishes deterministic velocity commands.

The evidence driver moves a real Gazebo obstacle into and out of the lidar
path. A run passes only when all of these assertions hold:

- the shortcut starts the reviewed workflow rather than a motor command;
- missing hold state or release cancels the first mission and publishes stop;
- lidar produces an obstacle stop followed by a path-clear recovery;
- the second start completes the workflow;
- the audit timeline, four labelled Gazebo captures, ground-truth displacement,
  and at least eight real camera frames are present.

The output directory contains `shortcut-result.json`, `report.json`,
`report.md`, labelled PNG captures, raw camera frames, an H.264 evidence video,
and SHA-256 files. The evaluator exits non-zero if any required artifact or
behavior is missing.

For a Cloud-connected ROS deployment, generate one short-lived local secret
outside the repositories and provide the same value to the Flyto2 local
backend and the Robotics process:

```bash
export FLYTO_ROBOTICS_INPUT_TOKEN="<at-least-32-random-bytes>"
export FLYTO_ROBOTICS_INPUT_URL="http://127.0.0.1:8765/v1/input-events"

ros2 run flyto_robotics shortcut_controller --ros-args \
  -p job_file:=/absolute/path/job.json \
  -p plan_file:=/absolute/path/validated-plan.json \
  -p result_file:=/absolute/path/shortcut-result.json
```

The gateway binds to literal loopback only and requires the bearer token for
health and input events. The Cloud browser talks only to its same-origin local
WebSocket; the backend keeps the secret off-wire and forwards the strict event
contract to Robotics. A press is shown as active only after Robotics confirms
that the exact reviewed workflow ID started. Unknown bindings, workflow
mismatches, control-thread acknowledgement timeouts, socket loss, stale
sensors, and dead-man expiry all fail closed.

## Flyto Cloud boundary

Dispatch the job JSON to a registered edge device as a normal batch execution.
The device command is:

```bash
python3 -m flyto_robotics.cli run-ros \
  --job /absolute/path/job.json \
  --result /absolute/path/result.json
```

The process exits non-zero on invalid input or a failed mission. The result
file conforms to `contracts/result-v1.schema.json`, so Cloud can upload it as
execution evidence without knowing ROS message types.

```text
Flyto Cloud
    │ versioned JSON job
    ▼
Flyto device runner
    │ starts process / captures exit code
    ▼
flyto-robotics mission controller
    │ cmd_vel                 ▲ odometry + lidar
    ▼                         │
Gazebo rover or real ROS 2 base
    │
    └── versioned JSON result ──► Flyto Cloud evidence
```

## Safety and scope

This is a competition and laboratory baseline, not a certified medical device.
It uses synthetic locations and payload identifiers. A real deployment still
needs an emergency stop, independent safety controller, access control,
infection-control review, cybersecurity review, and site acceptance testing.

HMAC proves that a trusted signer produced the decision; it does not by itself
implement hospital user authentication or authorization. Production key
custody, rotation, audit retention, RBAC, and revocation belong in the Flyto
control plane or another trusted approval gateway.

The competition supply-chain restriction must be evaluated against the final
physical BOM. Simulation assets do not establish hardware compliance.

## Contributing

See `CONTRIBUTING.md` for the pre-change exploration, atomicity, safety, and
post-change verification requirements.

See `PRODUCT.md` for the full product positioning, examples, current
implementation, differentiation, and roadmap.
