# Flyto Robotics Architecture

## Boundaries

- Flyto2 Robotics owns robot simulation, robot-side mission logic, ROS 2
  adapters, Gazebo assets, and real-hardware adapters.
- Flyto Cloud owns users, organizations, scheduling, device registration, job
  persistence, and evidence storage.
- Integration is a versioned JSON job and result envelope. There are no
  cross-repository source imports.
- The pure controller has no ROS dependency. ROS 2 is an adapter around it.
- Gazebo and physical bases expose the same velocity, odometry, and range
  semantics to the adapter.

## Installed resource registration and telemetry

`flyto-resource-agent` is the outbound-only installation boundary between any
adapter and Flyto Cloud. It publishes two independent JSON contracts:

```text
local adapter
  → flyto.resource-manifest.v1
      identity + adapter version + capabilities
      non-secret settings (secret settings expose configured=true only)
      telemetry schemas + bounded presentation semantics
  → paired-device HTTPS identity
  → Flyto Cloud resource inventory

local adapter observations
  → flyto.resource-telemetry.v1
      resource + channel + monotonic sequence + quality + bounded payload
  → Flyto Cloud latest-sample read model
```

Real hardware and simulation emit the same envelopes. Only
`deployment_mode=real|simulation|hybrid` and adapter provenance differ. Cloud
does not receive ROS topic names, implementation modules, network endpoints,
credentials, or raw actuator commands. Presentation kinds are bounded safe
identifiers, so an adapter may publish a namespaced future kind such as
`partner.timeline.v2`. Cloud may enhance known kinds with reusable primitives;
unknown kinds remain inert data rendered through the generic fallback.

The paired-device credential is read from an owner-only installation file at
request time, sent only in the HTTPS Authorization header, never included in a
manifest or telemetry body, and never followed across redirects. Manifest
revisions and telemetry sequences advance independently. Telemetry is a
read-only observation path; mission dispatch, approval, leasing, safe-stop,
and evidence continue through their existing versioned control contracts.

## Mission Stations dispatch boundary

```text
judge physically draws Zone + Objective cards
  → operator records exact card IDs in the control plane
  → immutable Task / Plan revision / Assignment revision
  → venue calibration revision (Z1–Z4 + START)
  → flyto.robotics.mission-dispatch.v1
  → registry snapshot + schema + policy validation
  → deterministic controller / perception adapter
  → action.execution receipt (never task-completing evidence)
  → separate card-defined evidence evaluation in the control plane
```

Robotics never draws cards and has no random-task endpoint. It consumes one
already selected judge-card contract and validates only the execution slice.
`executor_kind` is explicit; the gateway does not infer which engine should
run a step. The capability catalog projects only locally registered atomic
capabilities as `APPROVED` and content-addresses both each argument schema and
the complete reviewed snapshot.

Portable stations have stable marker identities rather than fixed venue
coordinates. A READY calibration must contain Z1, Z2, Z3, Z4, and START, with
its content hash verified before dispatch. AprilTag, overhead-camera, and
manual calibration sources share that contract. The same dispatch is consumed
for simulation and physical hardware; only the lower adapter changes.

Raw velocity, PWM, ROS-topic, and arbitrary-command fields are absent from the
schema and rejected by the parser. Any movement-bearing dispatch ends in the
existing `safe_stop` primitive. A step success is emitted as a nested
Cloud-compatible `action.execution` observation inside a versioned Robotics
envelope; it is explicitly marked `task_completion_eligible=false`. Only the
control plane can decide whether independent judge-card evidence completes the
Task.

## AI-native atomic composition

The dependency direction is one-way:

```text
goal in any language/modality + robot observations
    ↓ language adapter
flyto.goal-frame.v1
    ↓ hard compatibility and safety filters
versioned capability manifests
    ↓ exact semantic affordance/effect ranking (bounded top-k)
trusted Blueprint hints + scoped Core discovery
    ↓ AI sees only registered shortlisted capabilities
versioned, untrusted plan
    ↓ schema + registry validation
immutable WorkflowPlan
    ↓ orders
single-purpose WorkflowStep atoms
    ↓ interpreted by
pure closed-loop controller
    ↓ adapted by
ROS 2 → Gazebo or physical base
```

The executable reference vocabulary is intentionally small:

- `navigate`: close the odometry/heading/range loop toward one station;
- `navigate_to_location`: resolve a stable location ID outside the LLM, then
  reuse the same bounded pose controller;
- `save_current_location`: atomically store current trusted odometry under a
  stable ID and bounded Unicode labels while holding zero velocity;
- `follow_line`: follow a semantic route ID using camera observations;
- `dwell`: remain stopped at one station for a bounded interval;
- `wait_until_clear`: require continuous lidar clearance while holding zero
  velocity;
- `ask_human`: request an external decision and hold zero velocity;
- `resume`: fail closed unless the corresponding decision was approved;
- `safe_stop`: explicitly hold zero motion for a bounded interval.

The registry publishes a stable canonical ID, executable runtime name, version,
intent IDs, affordances, effects, handled events, arguments, ranges, required
observations/resources/permissions, compatible robots, safety class, side
effects, and recovery abilities. Aliases and examples remain legacy recall
metadata. It is a runtime allowlist, not merely prompt documentation. Unknown
abilities and arguments are rejected after the LLM responds.

The complete catalog belongs to the deterministic router. The LLM receives at
most the configured top-k, plus a registry snapshot and selection evidence.
Runtime, robot, sensor, resource, permission, and source scope are hard filters.
Trusted Blueprint experience may boost an installed atom; it cannot introduce
an unavailable atom or bypass the trust gate. Core discovery flows through the
Flyto AI Core bridge and remains out of scope for robot motion unless the
caller explicitly allows that source.

The LLM chooses semantic steps in a slow loop and never participates in the
10 Hz motor loop. Obstacle stopping remains a cross-cutting deterministic
safety guard applied to every motion primitive.

## Detachable Robot MCP

`flyto_robotics.mcp_server` publishes a small stdio MCP contract for Flyto2 AI
or any protocol-compatible client. Its four tools list the versioned capability
registry, prepare a bounded planner request, validate and compile an untrusted
plan, and run a job/plan pair through the real deterministic controller. The
MCP process does not expose raw actuators, ROS topics, shell execution,
arbitrary file paths, or network access.

The MCP is an adapter over the existing safety boundary, not an alternate
robot runtime. Plan and job payloads still pass the same strict parsers,
terminal `safe_stop` policy, registry checks, speed limits, obstacle handling,
and result contract used by the CLI, Gazebo, and physical ROS adapter. The
stdio process can be removed without changing planning, simulation, or robot
control, and the robot stack can run without Flyto2 AI.

The release benchmark launches that production stdio process and negotiates MCP
before every campaign. At least 101 distinct cases each prepare a routed planner
request, validate a variable-depth semantic plan, and execute it with the real
deterministic `MissionController`. Standard, intermediate, and advanced tiers
exercise bounded waits, clearance sensing, human approval/resume, short motion,
and terminal safe-stop. The family and every tier must independently reach a
90% success rate. Results are written atomically as owner-readable,
content-addressed JSON evidence; failed stages remain explicit instead of being
retried into a passing count.

## Attested branching-plan boundary

```text
versioned route graph + live resource observations
  → deterministic hard exclusion and ranking
  → bounded executable route candidates
  → Flyto AI structured completion
  → one Schema branch per full semantic route
  → provider attestation and independent Robotics validation
  → resource change
  → recompute candidates and request a second completion
  → byte-equivalent attested plan handed to Gazebo
```

Route attributes and device dependencies are data, not prompt-only prose.
Dependency severity is derived from separate safety, task, evidence,
substitution, confidence, freshness, recovery, and phase axes. If no route
survives the hard checks, planning stops before contacting the executor.

The model chooses a candidate but cannot invent its geometry. A route template
constrains every `navigate_to_location` ID in order, followed by any required
approval pair and terminal `safe_stop`. Trusted coordinates stay in the
map-scoped store. The planning session hashes both requests, both returned
plans, both provider attestations, the resource change, and the selected final
round. Gazebo evidence is accepted only if its input plan is byte-canonically
equal to that final attested plan.

## Workflow shortcut trust boundary

```text
keyboard / joystick / external adapter
  → flyto.robotics.input-event.v1
  → exact source + control binding
  → registered workflow ID
  → normal plan validator + immutable WorkflowPlan
  → MissionController
  → velocity command
```

Input events contain lifecycle only: `press`, `heartbeat`, `release`, or
`disconnect`. The schema has no velocity, PWM, ROS topic, or arbitrary command
field. A press arms a prevalidated workflow such as
`shortcut.forward.30cm.v1`; only the controller's next observation tick may
produce motion. Release, source disconnect, or heartbeat expiry cancels the
active mission before the next controller update and therefore yields zero
velocity. Replayed event IDs, non-increasing session sequences, unknown
bindings, wrong-robot workflows, and concurrent shortcut starts fail closed
with bounded audit evidence.

`move_relative` is a single-purpose primitive. It captures the current trusted
odometry pose on entry, projects displacement along that starting heading,
clamps speed, applies the global obstacle guard, and completes at a bounded
distance tolerance. Positive distance moves forward and negative distance
moves backward. It does not add a second device-specific motor controller.

## Semantic-location trust boundary

```text
speech / text / UI
  → Flyto AI emits language-neutral Goal Frame
  → deterministic registry chooses location atom
  → LLM sees location IDs + multilingual labels only
  → validated plan contains location_id only
  → map_id-scoped store resolves trusted pose
  → existing navigate control loop
```

The full `semantic-location-map.v1` never crosses into the planner request.
Only `semantic-location-catalog.v1` does. A label is not an identity and can be
added in Chinese, Arabic, Japanese, or another script without changing the
atom or storage contract. Physical maps are isolated by exact `map_id`; a map
copied to the wrong environment is rejected before execution. Writes use the
current odometry pose and an atomic JSON replacement. Optional expected
revisions prevent lost updates.

This layer is injected into workflow compilation and mission execution.
Existing workflows receive `None` by default and follow their original code
paths. `navigate_to_location` compiles down to the already tested navigation
controller instead of adding another motor-control implementation.

## Human-approval trust boundary

```text
authenticated Flyto gateway
  → RBAC and approval policy
  → signed human-decision.v1
  → ROS /flyto/human_decision
  → HMAC + scope + expiry + nonce verification
  → MissionController.submit_human_decision
  → ask_human completes
  → resume verifies the same approval ID
```

The signed envelope is bound to the current job and robot. It expires within
five minutes and each nonce is accepted once. The ROS topic is therefore an
untrusted transport rather than an authorization mechanism. The shared secret
is present only when the workflow contains `ask_human` and is read from the
process environment.

HMAC authenticates the trusted gateway, not an individual hospital user.
Identity, RBAC, key custody, rotation, and revocation remain control-plane
responsibilities.

## Closed loop

```text
job contract
   │
   ▼
mission state machine ── decision ──► velocity command
   ▲                                      │
   │                                      ▼
odometry + camera + minimum range ◄──── Gazebo / physical robot
```

The bundled `hospital_delivery.v1` composition produces these states:

```text
accepted
  → navigating_to_pickup
  → waiting_for_pickup
  → navigating_to_dropoff
  → waiting_for_dropoff
  → completed
```

Invalid jobs fail before motion. A range reading below the configured stop
distance produces a zero-velocity command. Missing or stale odometry causes the
ROS adapter to stop and fail the mission. A line-follow plan also requires
fresh camera frames. A step marked `request_replan` emits a replan event and
stops before a higher-level agent creates a replacement plan.

## Contract

`flyto.robotics.job.v1` describes:

- stable job and robot identifiers;
- a synthetic hospital-delivery task;
- pickup and drop-off poses;
- payload classification without patient data;
- speed, obstacle-stop, pose-tolerance, timeout, and dwell limits.

`flyto.robotics.result.v1` records:

- terminal status and reason;
- timestamps and elapsed time;
- final controller state and pose;
- transition and safety-event evidence.

`flyto.robotics.plan.v1` records the natural-language goal, target robot,
planner identity, ordered registered capability calls, bounded arguments,
timeouts, and `abort` or `request_replan` failure policies.

`flyto.robotics.human-decision.v1` records a short-lived decision, actor ID,
job/robot/approval scope, nonce, and HMAC-SHA256 signature.

`flyto.robotics.input-event.v1` records an input source, control, session,
phase, and monotonic sequence. Arrival time and dead-man policy are trusted
runtime state rather than caller-provided timestamps.

## Simulation

The self-contained SDF world contains a pharmacy, a ward, walls, and a static
obstacle. The rover model uses Gazebo Harmonic's built-in physics, sensors,
differential drive, and joint-state systems. No Fuel or other online asset is
required.

ROS 2 Jazzy communicates through `ros_gz_bridge`:

- ROS `/flyto/cmd_vel` → Gazebo rover velocity;
- Gazebo rover odometry → ROS `/flyto/odom`;
- Gazebo lidar → ROS `/flyto/scan`;
- Gazebo route camera → ROS `/flyto/camera/image`;
- Gazebo world pose → ROS `/flyto/ground_truth`;
- Gazebo clock → ROS `/clock`.

## Facility resource handoff

Physical devices remain outside the LLM's direct choice boundary:

```text
robot ground-truth zone + resource health
  → FacilityResourceCatalog exact-zone filter
  → declared fallback filter
  → deterministic priority ordering
  → release previous lease
  → acquire exact adapter + endpoint
  → append replayable evidence event
```

The catalog is data-driven and accepts arbitrary device kinds, zone IDs,
adapters, and endpoints. The AI plan composes semantic task atoms; the facility
runtime binds the exact camera, robot, speaker, elevator, or gateway only from
the current AI Space resource document. No healthy declared match means no
lease and no execution.

The AI4ALL world publishes three independent camera topics. Camera A covers
the blue zone, camera B covers yellow and purple, and the overhead camera is an
explicit fallback. A test fault makes camera B unhealthy after the rover enters
purple; the active video sequence then consumes only the newly leased overhead
stream. The obstacle, approval, controller, and resource timelines retain a
shared simulated clock for replay.

## Verification trust boundary

Controller odometry is an execution input, not sufficient proof that the
simulated body moved. The adversarial lab separately consumes Gazebo's
`OdometryPublisher` output on `/flyto/ground_truth` and requires at least
3.8 m of world displacement.

```text
versioned scenario + hashed world/model/plan
  → fresh ROS 2 / Gazebo process
  → dynamic obstacle + signed approval + replay attempts
  → mission result / audit events
  → independent Gazebo pose + overhead PNG manifest
  → strict 28-check report
  → repeated cold-start matrix
```

Generated reports are JSON, Markdown, and JUnit. The report hashes scenario
inputs, mission result, and driver manifest. A deterministic soak separately
normalizes timestamps and requires identical evidence fingerprints.

This separation caught a real model defect: wheel odometry advanced while the
rover body barely translated. The wheel joint axis frame was corrected and
world-displacement became a mandatory acceptance gate.

## Real hardware replacement

A real base replaces only the launch-time adapter and topic mappings. The job
schema, pure controller, result schema, and Cloud process contract remain
unchanged. Higher-level Nav2 or vendor navigation may later replace the simple
pose controller behind the same mission interface.

C, C++, Python, a ROS action server, or a vendor SDK can implement an atom as
long as its adapter honors the registered argument and observation contract.
The planner sees abilities, not implementation languages.
