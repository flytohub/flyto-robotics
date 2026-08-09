# Capabilities: atomic and composable

How abilities are registered, matched and composed.

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
  → trusted Flyto2 Blueprint hints + Flyto2 Core discovery metadata
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
"Remember this place as the nurse station"
  → Goal Frame selects save_current_location
  → current trusted odometry is stored under a stable location ID

"Go to the nurse station"
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
