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
