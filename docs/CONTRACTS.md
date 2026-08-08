# API and contracts

The JSON contracts, the semantic pairing with ROS 2, and the stress gate.

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

#### Flyto2 + ROS 2 semantic pairing

Flyto2 owns language understanding, capability planning, resource policy, and
audit evidence. ROS 2 owns deterministic execution, feedback, cancellation,
lifecycle, and the emergency stop. The model never receives ROS graph names or
actuator values.

The standard profile maps the existing executable navigation atoms to Nav2's
cancellable `NavigateToPose` action. Every adapter must declare both simulation
and hardware support, so the mission contract cannot fork between Gazebo and a
physical robot. MoveIt 2 and ros2_control action types are allowlisted for later
adapters, but a manifest is rejected until its Flyto2 capability is actually
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

The stronger live path uses the `flyto-ros2-readiness-probe` ROS executable. It
resolves the declared action or service type through ROS 2, waits for the real server,
queries every managed node through `lifecycle_msgs/GetState`, and requires a
`std_srvs/Trigger` emergency-stop service owned by a different ROS node. The
persisted snapshot contains only redacted pass/fail facts and observation
hashes; ROS graph names stay inside trusted deployment configuration.

```bash
ros2 run flyto_robotics flyto-ros2-readiness-probe \
  --manifest examples/ros2-adapters/flyto2-standard.json \
  --output results/ros2-runtime.json \
  --deployment-mode hardware \
  --emergency-stop-node /safety/emergency_supervisor \
  --emergency-stop-service /safety/emergency_stop
```

Readiness alone does not authorize motion. `authorize-ros2-execution` also
requires an immutable AI Space resource plan and binds its workflow, Space,
robot, endpoint, semantic capability, adapter, graph evidence, and expiry into
`flyto.robotics.ros2-execution-grant.v1`. The grant is safe to return through
MCP because it contains no ROS action, service, or actuator values. Only the
deterministic adapter can resolve the private graph target, and any expired or
cross-context grant fails closed.

```bash
python3 -m flyto_robotics.cli authorize-ros2-execution \
  --manifest examples/ros2-adapters/flyto2-standard.json \
  --runtime examples/ros2-runtime/ready-sim.json \
  --resource-plan examples/resource-plans/nav2-hospital-delivery.json \
  --workflow hospital_delivery.v1 \
  --resource flyto-rover-sim-001 \
  --capability robotics.motion.navigate@1 \
  --space gazebo-nav2-lab \
  --at 2026-08-01T10:00:00Z
```

The final closed loop runs the same semantic contract through a real Nav2
`NavigateToPose` Action and the Gazebo rover. AI selects only a registered
location ID. The trusted adapter resolves its pose after checking the live ROS
graph and issuing the short-lived grant. A separate ROS node owns the latched
emergency stop and is the only node allowed to publish actuator velocity. It
forwards authorized Nav2 commands while reset and continuously publishes zero
velocity while stopped.

```bash
make nav2-closed-loop
```

This rebuilds the Jazzy/Harmonic image when full Navigation2 is missing, then
runs three isolated headless launches: successful navigation, an accepted goal
that is canceled after measured displacement, and an accepted goal canceled by
the external emergency-stop service. Every run must observe Action feedback,
real odometry movement, the expected terminal result, and a content-addressed
redacted evidence document. Cancellation and emergency-stop runs also wait for
post-stop odometry and fail if the rover drifts more than 5 cm. Replay one
document with:

```bash
python3 -m flyto_robotics.cli verify-ros2-execution-evidence \
  --evidence results/nav2-closed-loop/<run>/success.json \
  --scenario success
```

The evidence binds the resource plan, Space, robot, adapter, capability, live
runtime and grant snapshots without exposing action names, message types or
velocity commands.

#### Nav2 fault-injection stress gate

Run the real Jazzy/Harmonic stack repeatedly and inject sensor and lifecycle
failures only after the rover has started moving:

```bash
make nav2-stress
```

The default gate runs five independent successful navigation containers, then
one container each for LiDAR dropout, frozen odometry, and a deactivated Nav2
controller. Every container uses a unique ROS domain. Raw Gazebo sensors pass
through `ros2_sensor_guard`; the safety supervisor independently watches action
execution state, odometry, LiDAR, and command freshness and owns the only
actuator output.

A fault passes only when the action was accepted, feedback and physical motion
were observed, the injected fault was observed, the exact safety reason was
latched, stop latency was at most 750 ms, and post-stop physical drift was at
most 5 cm. The run also proves an expired grant is rejected before the private
ROS action endpoint is resolved. Evidence remains under ignored
`results/nav2-stress/` output.

```bash
FLYTO_ROBOTICS_STRESS_SOAK_RUNS=50 make nav2-stress
```

```bash
export FLYTO_ROBOTICS_PLANNER_URL=https://planner.example.com/v1/robot-plan
export FLYTO_ROBOTICS_PLANNER_TOKEN=...
python3 -m flyto_robotics.cli plan-ai \
  --goal "follow the blue line, then the yellow, then the purple, stopping for obstacles" \
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
key: an authenticated Flyto2 gateway should apply RBAC, derive the actor ID, and
sign the envelope after approval.

Python composition types are exported from `flyto_robotics`: `WorkflowStep`,
`WorkflowPlan`, `PrimitiveKind`, `MissionController`, and the initial
`hospital_delivery_workflow` compiler.

#### Semantic location example

Build the exact provider-neutral request Flyto2 AI sends to a planner:

```bash
python3 -m flyto_robotics.cli planner-request \
  --goal "go to the end of the blue line, then the yellow, then the purple, and stop safely" \
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
