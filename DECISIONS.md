# Decisions

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
