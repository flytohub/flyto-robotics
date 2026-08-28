# Decisions

## 2026-08-27 — Commissioning is a dispatched device job, never a mission primitive

Decision: recording a venue map is `mapping.start` / `mapping.save` /
`mapping.abort` on the installed device-executor protocol, dispatched by
`deploy/flyto_job_runner.py`'s non-`robotics.`-prefixed path. No `start_slam`
enters `workflow.PrimitiveKind`, and the mission engine is untouched. The
driving between start and save stays on the already-approved
`robotics.motion.*` capabilities with their sensor gates.

Reason: a primitive is a word the AI planner composes delivery plans out of. A
plan able to emit "begin remapping the building" halfway through carrying
something would have to be *refused* at validation rather than being
inexpressible, and every refusal is a rule someone can get wrong. Mapping
happens once per venue, before deliveries, so it belongs on the generic
executor path the architecture already says remains required. Rejected: giving
the executor its own way to move the robot — a second, unreviewed motion path
alongside one that already carries the safety envelope.

## 2026-08-27 — Nav2 ships disabled rather than localised against a fiction

Decision: `deploy/systemd/flyto-nav2.service` is installed and not enabled. Its
`ConditionPathExists` names a map recorded from the actual venue, so the unit
cannot start until one exists.

Reason: `maps/nav2_lab.pgm` is a 20x20-pixel synthetic 8 m square with a
drawn-on perimeter at 0.4 m per cell, present so the launch files load. AMCL
against it converges, the costmap looks plausible, and every goal is computed in
a room that does not exist — it fails silently, which is worse than not
navigating at all. Rejected: enabling the unit and treating the synthetic map as
a placeholder, which makes "Nav2 is running" true and "the robot knows where it
is" false at the same time. State as of this date: unit installed, `disabled`,
`inactive`, and `/home/ubuntu/.flyto/maps/` does not exist, so the condition has
never once been satisfied.

## 2026-08-22 — Cloud dispatch and robot execution use two separate proofs

Decision: every trace-bearing Space job must carry
`flyto.cloud.device-job-handoff.v1`, binding the exact paired device, trace,
workflow digest, and Cloud-owned completion authority before the runner reaches
the local gateway. A terminal delivery session emits one cached
`flyto.robotics.execution-receipt.v1`; the runner validates its closed shape,
plan binding, bounded terminal facts, and canonical digest before returning it.

Reason: a queue lease says who may claim a job, but it did not prove that the
workflow and device seen at the edge were the assignment Cloud scheduled. In
the other direction, pose and clearance were useful evidence but there was no
single content-addressed record of which plan and terminal controller result
produced them. Separate handoff and receipt contracts close those two joins
without importing Cloud source or moving mission authority onto the robot.

Boundary: SHA-256 detects content drift and the paired-device channel
attributes transport; it is not a TPM-backed hardware attestation. The receipt
is always `task_completion_eligible=false`. Only Cloud's independent evidence
rules may complete a Task, and the deterministic controller and safe-stop
policy remain the only motion authority.

## 2026-08-13 — The AVFoundation provider is a topology, not a duplicate

Decision: `camera_sources.PROVIDERS` carries both `ros_image` and
`avfoundation`, and neither supersedes the other.

Reason: the camera was first mounted externally, on the operator's Mac rather
than on the robot, and `avfoundation` is the capture path for that arrangement —
the only one that works when the device is attached to the workstation. Mounting
a UVC camera on the robot on 2026-08-27 made `ros_image` the deployed provider;
it did not make the other redundant. Reading it as redundant is the specific
mistake this entry exists to prevent, and it has already been made once: on
2026-08-27 the provider was assessed as duplicating flyto-cloud's own macOS
capture path, because the original topology was recorded nowhere and the cause
had to be recovered from the person who chose it. The profile is described as
provider-neutral in three documents; what was missing was the history.

## 2026-08-13 — Plan delivery and capability discovery share one authority

Decision: the authenticated loopback delivery gateway that accepts
`POST /v1/plans` also exposes read-only `GET /v1/capabilities`. The body is the
existing `flyto.robotics.capability-catalog.v1` execution projection generated
by the default capability registry; the gateway owns no duplicate definitions,
schemas, bounds, or hashes. Discovery is uncached and cannot start or mutate a
mission.

Reason: a plan producer must validate against the same language-neutral
authority the robot enforces. Keeping this surface discovery-only prevents
catalog access from becoming motor authority and keeps simulation and physical
execution behind the existing guarded plan route. Consumer migration in
`flyto-modules-robotics` remains the next layer.

## 2026-08-08 — The device credential is protected by permissions, not encryption

Decision: the robot's device secret is stored as clear text with owner-only
permissions set at creation, an atomic write, and a refusal to load it if those
permissions have been widened. It is not encrypted at rest on this hardware.
Where a host provides systemd credentials, the runner prefers them and writes
nothing.

Reason: the lab robot is a Raspberry Pi 4 with no TPM or secure element. It
pairs itself and must read its own secret at boot with no operator present, so
any key it can use unattended is a key the SD card also holds. Encrypting
against a key stored beside the ciphertext would satisfy a scanner and protect
nothing, and a control that only appears to work is worse than a documented
limit — someone will plan around it.

What is enforced instead is the boundary that can actually hold: no other
account on the machine, no backup, and no stray chmod yields the secret, and a
credential whose permissions were widened is treated as already disclosed
rather than used. Physical possession of the card still yields it, which is
why SECURITY.md says so outright and why a lost robot should be unpaired from
Cloud rather than trusted.

CodeQL alert 1 (py/clear-text-storage-sensitive-data) went from open to fixed
across this change. That is not evidence the secret became encrypted — the
sink pattern changed. The real gains were the exposure window between create
and chmod, the non-atomic write, and the missing permission check on read.

## 2026-08-08 — Installed resources describe themselves to Cloud

Decision: an installation publishes `flyto.resource-manifest.v1` and
`flyto.resource-telemetry.v1` through the exact existing paired-device
identity. The manifest carries adapter identity, generic capability IDs,
bounded setting descriptors, telemetry schemas, and presentation semantics.
Secret settings may report only whether they are configured; their values are
invalid contract data. Cloud stores a bounded latest sample per declared
channel and exposes no command or actuator route on this surface. Simulation
and real hardware use the same contracts and differ only by deployment
provenance.

Reason: a TurtleBot-, ROS-topic-, or fixed-field Cloud screen would have to be
rewritten for every camera, arm, vehicle, simulator, and future adapter. A
resource-owned schema lets one installation populate Cloud while keeping
credentials and real-time control local, preserving the existing deterministic
mission and safe-stop boundary.

Presentation kinds are deliberately open, bounded identifiers rather than an
enum. This lets software-workflow and future hardware adapters add semantics
without changing the transport contract or forcing a Cloud release. Unknown
kinds must remain inert and readable through the generic Cloud fallback.

## 2026-08-08 — Judges draw mission cards; Robotics validates execution only

Decision: Zone and Objective cards are physical competition inputs drawn by
judges. An operator records the exact result as `card_source=judge_draw`.
Robotics exposes no draw, shuffle, or random-task API and accepts only a
versioned dispatch bound to immutable plan and assignment revisions, a current
approved capability snapshot, and a READY Z1–Z4 plus START calibration.
Movement dispatches end in `safe_stop`. Robotics action receipts are
content-addressed `action.execution` observations and explicitly cannot satisfy
the separate card evidence contract.

Reason: allowing the product to draw its own task would weaken the competition
challenge and make demonstrations irreproducible. Separating human challenge
selection, control-plane planning, resource assignment, deterministic robot
execution, and evidence-based completion also prevents action success from
being misreported as mission success.

## 2026-08-01 — Robot MCP evidence is per-case, tiered, and content-addressed

Decision: a release campaign contains at least 101 distinct cases. Every case
uses the production stdio process to prepare a request, validate a semantic
plan, and run the real deterministic controller. The overall family and each
difficulty tier must independently reach 90%. A case is counted once, without
automatic retries, and retains stage results plus input/output digests in an
atomic owner-readable report whose filename contains the evidence digest.

Reason: a pooled unit-test count can conceal an unusable protocol, an easy-only
workload, or selective retrying. The tiered contract proves negotiation,
validation, safety behaviors, controller execution, and evidence persistence
as one repeatable boundary while keeping generated workload data separate from
mocked runtime behavior.

## 2026-08-01 — Robot MCP is an adapter, never a second control plane

Decision: the detachable stdio MCP exposes only semantic capability discovery,
bounded planner request construction, strict plan compilation, and execution
through the existing deterministic dry-run controller. It accepts complete
job and plan documents, never caller-selected paths. Raw actuators, ROS topics,
shell execution, arbitrary network access, and direct physical motion are not
tools. The same job, plan, workflow, and result contracts remain authoritative
for MCP, CLI, Gazebo, and physical adapters.

Reason: Flyto2 AI and other MCP clients need a composable robot-development
surface without entering the real-time safety boundary. Keeping protocol I/O
outside the controller makes the agent replaceable and prevents an apparently
convenient tool from bypassing plan validation, safety stops, or evidence.

## 2026-07-30 — AI chooses a complete constrained route, not arbitrary waypoints

Decision: a data-driven route graph first applies deterministic resource
dependency checks and produces a bounded list of complete semantic-location
sequences. Flyto AI chooses among those candidates through structured output.
For route sessions, the JSON Schema contains one exact step template per
candidate, including the full location order, required human approval pair,
and terminal safe stop. Robotics independently verifies the provider
attestation, shortlist, route sequence, hashes, and equality with the plan
actually given to Gazebo.

Reason: natural-language prompting alone allowed a model to skip intermediate
locations even after repair feedback. Constraining the choice while keeping
route feasibility and physical control deterministic preserves useful AI
decision-making without allowing mixed paths, omitted atoms, or direct motor
authority.

## 2026-07-30 — Exact physical resources are bound outside the LLM

Decision: Robotics accepts an `ai-space-resource-plan.v1` document only through
its strict resource-binding boundary. Before ROS starts, the parser requires
the exact contract fields and SHA-256 snapshot, then selects exactly one
matching workflow/resource/capability/adapter/Space binding. Confirmation is
enforced when requested. Unknown fields, raw motor commands, ambiguous
endpoints, and any identity mismatch fail closed.

Reason: capability selection and physical resource authority are different
problems. An LLM can compose a semantic workflow, but it cannot safely infer
which camera, elevator, robot, or vendor adapter currently owns a hospital
location. A small transport-neutral binding atom lets Cloud perform live
authorization and leases while Robotics independently verifies the frozen
decision without importing Cloud or tying the contract to Python.

## 2026-07-30 — Facility handoff is a small deterministic resource runtime

Decision: device kinds, zones, priorities, adapters, endpoints, and fallback
coverage live in a versioned facility-resource document. A generic selector
prefers the exact healthy zone, then only declared fallbacks. Health changes
release the active lease before a replacement is acquired, and every choice is
appended to replayable evidence.

Reason: embedding hospital geography or camera names in one giant robot
workflow would make small demos unnecessarily complex and large deployments
fragile. Keeping semantic AI planning, facility resource binding, ROS control,
and evidence as separate atoms lets one-camera projects stay simple while
multi-floor deployments add detail through data.

## 2026-07-30 — Shortcuts select workflows, never motor commands

Decision: keyboard, joystick, and external-device inputs use the versioned
`flyto.robotics.input-event.v1` lifecycle contract. Exact bindings resolve to
prevalidated workflow IDs for the active robot. `press` starts a workflow;
`release`, source disconnect, or heartbeat timeout cancels it before the next
controller update. The first jog behavior is the atomic `move_relative`
primitive followed by mandatory `safe_stop`.

Reason: a shortcut is useful for rapid testing and manual operation, but
mapping it directly to velocity or PWM would bypass plan validation, capability
policy, obstacle guards, and replayable evidence. Keeping input lifecycle,
workflow selection, and deterministic motion as separate small components
preserves the same safety boundary for keyboard, joystick, Gazebo, and physical
hardware.

## 2026-07-28 — Semantic labels are not location identity

Decision: location memory uses a stable, map-scoped `location_id`; bounded
Unicode labels are aliases for people and language understanding. The full map
stores trusted poses, while the LLM receives a separate coordinate-free
catalog. `save_current_location` writes only current odometry and
`navigate_to_location` resolves only registered IDs.

Reason: deriving identity or coordinates from natural-language wording would
bind the system to selected languages and let the model cross the motion trust
boundary. Separate ID, catalog, map, and controller layers make multilingual
teaching possible while preserving deterministic execution.

## 2026-07-28 — New atoms cannot enter unrelated zero-score fallback

Decision: additive location atoms require a positive legacy relevance score or
an exact Goal Frame semantic match. Semantic-frame routes contain positive
matches only, with deterministic terminal `safe_stop` insertion for motion.

Reason: a growing registry must not change an old shortlist merely because a
new zero-score manifest sorts ahead of an existing atom. This preserves old
atom behavior while still allowing explicit discovery of new abilities.

## 2026-07-28 — Language understanding and capability selection are separate

Decision: arbitrary language, speech, UI, schedule, or sensor input is converted
to `flyto.goal-frame.v1`. The capability router ranks exact canonical
intent/affordance/effect/event IDs and ignores the raw wording whenever that
frame exists. Aliases remain only for backwards-compatible fallback.

Reason: a router-owned synonym list merely replaces one language lock with
several language locks. A stable semantic contract makes equivalent meaning
produce identical routing evidence without weakening deterministic safety.

## 2026-07-28 — LLMs receive a verified shortlist, never catalog authority

Decision: capability manifests use stable namespaced IDs and executable runtime
names. A deterministic router applies robot, observation, resource, permission,
domain, and source filters before semantic-frame ranking. The LLM receives only
a bounded shortlist and its plan is checked against that same shortlist.
Blueprint can boost only trusted installed abilities; Core discovery stays
behind Flyto AI and cannot enter robot scope implicitly.

Reason: sending hundreds of similarly named modules to a model makes selection
unstable and allows irrelevant discovery scores to dominate. Hard scope,
deterministic evidence, and fail-closed ambiguity keep model reasoning focused
without putting the model in the motor or authorization loop.

## 2026-07-28 — Robotics is an independent repository

Decision: create `flyto-robotics` instead of adding ROS and Gazebo dependencies
to Flyto Cloud.

Reason: Cloud is a deployment and scheduling control plane. Keeping robot
middleware outside it preserves Cloud startup, edition, provider, and
dependency boundaries while allowing simulation and physical hardware to
evolve independently.

## 2026-07-28 — Contracts are transport-neutral

Decision: Cloud integration uses versioned JSON job and result documents plus
process exit status. ROS messages remain internal to Robotics.

Reason: the same Cloud job can target Gazebo, TurtleBot, another ROS 2 base, or
a C firmware gateway without changing the control-plane data model.

## 2026-07-28 — Jazzy and Harmonic are the reference pair

Decision: target ROS 2 Jazzy on Ubuntu 24.04 with Gazebo Harmonic.

Reason: this is the supported Jazzy pairing, remains compatible with the
TurtleBot3 migration path, and is more suitable for a short competition
delivery window than adopting a newer, less-tested combination.

## 2026-07-28 — The controller must run without ROS

Decision: mission parsing and the closed-loop state machine use only the Python
standard library. ROS 2 is loaded only by the runtime adapter.

Reason: contracts, safety transitions, and mission completion must remain
testable on development machines and CI runners without installing the full
robotics stack.

## 2026-07-28 — Robot capabilities are atomic and composable

Decision: scenario compilers produce immutable workflows from single-purpose
primitives. The controller receives a workflow instead of embedding one
business process, and hardware remains behind command/observation adapters.

Reason: pharmacy delivery, inspection, charging, and future C-based device
actions must be reusable and independently testable without forking the
controller or coupling a workflow to Gazebo.
