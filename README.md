<p align="center">
  <img src="docs/assets/flyto2-logo.png" alt="Flyto2" width="96" height="96">
</p>

# Flyto2 Robotics

External ROS 2 adapter, deterministic control, simulation, safety, and evidence
toolkit for Flyto2.

**A robot is standard ROS 2 equipment. It is not a Flyto2 appliance.**

This library is the host-side driver layer next to the equipment. It runs on
the execution host or on a companion computer beside the robot and drives the
robot's own native ROS 2 / Nav2 stack. It is never firmware, and nothing from
Flyto2 is installed on the robot: the lab robots stay stock TurtleBot3. It does
not declare capabilities to Flyto2 either; the `flyto-modules-robotics` pack
does that through flyto-core's `@register_module` capability contract. See
`DECISIONS.md` (2026-10-04).

Production topology:

```text
Flyto2 Cloud / War Room
        |
        | goal, policy, approved capability
        v
AI Space execution host or companion computer
(Mac / laptop / Steam Deck / mini PC)
        |
        | flyto-modules-robotics pack (declares capabilities)
        | Generic ROS 2 Adapter (this library: drives the robot)
        v
DDS / Zenoh / rosbridge / standard ROS 2
        |
        v
TurtleBot3 / other ROS 2 robot
        |
        | odom / scan / camera / TF / action result
        v
external adapter evidence
        |
        v
Flyto2 independent verification
```

The Raspberry Pi / robot should contain only its normal operating system, ROS 2,
upstream robot packages, sensors, Nav2/SLAM when needed, and generic ROS
transport. It does **not** need a Flyto2 credential, job runner, gateway,
scheduler, agent, task database, evidence database, or Flyto2-specific ROS node.

## What this repository owns

- deterministic ROS 2 motion/controller utilities;
- Nav2 / standard ROS 2 adapter contracts;
- fail-closed sensor, clearance, stop, and execution-grant logic;
- versioned execution/evidence contracts;
- Gazebo Harmonic worlds and adversarial lab scenarios;
- physical-robot regression and acceptance helpers;
- camera/resource adapters intended for an **external execution computer**;
- simulation and hardware evidence tooling.

The repository does **not** own Flyto2 scheduling or task-completion authority.
A ROS action completing successfully is execution evidence, not proof that the
user's objective was achieved.

## Architecture

Equipment adapters are installed on the AI Space execution host, or on a
companion computer beside the robot, and loaded through a host-neutral adapter
boundary. The built-in Python AI Space host uses
`flyto2.external_adapters` / `flyto2.resource_discoverers` entry points; process
hosts such as optional Flyto2 Runtime can use the versioned
`flyto2.adapter-provider.v1` protocol. Adapter identity maps to the provider,
for example `ros2.generic` maps to `flyto2-adapter-provider-ros2-generic` on the
process-host path. The adapter owns ROS 2, rosbridge, OpenRMF, camera-stream and
other transport details; the selected execution host owns assignment-scoped
authority and lifecycle; Cloud owns resource/capability inventory, approval,
routing, evidence and verification. Capabilities reach Flyto2 through the
`flyto-modules-robotics` pack's `@register_module` declarations, not through
this repository.

Provider discovery is passive. An installed provider may report a
`flyto.resource-manifest.v1`, but discovery never grants motion or other
effects. A host that passes `manifest_extensions=["module_pack"]` to the
`ros2.generic` discoverer (or `--manifest-extension module_pack` to
`flyto2-adapter-provider-ros2-generic --discover`) also receives
`"module_pack": "robotics"`, the `flyto.modules` entry-point name of the pack
that drives the resource. The discoverer offers the same name as its
`module_pack` attribute. Without the request the manifest keeps its released
shape, because released hosts reject unknown fields. For execution, Runtime binds the exact commanded resource and approved
capability allowlist to one assignment and exposes that authority only through a
loopback endpoint to Flyto2 Core. The robot itself remains standard equipment
with no Flyto2 credential or scheduler.

## Robot-side rule

The current source tree contains no Flyto2 Pi job runner, robot lifecycle
installer, recovery portal, doctor, delivery gateway, robot credential
provisioner, or robot-side scheduler.

Those appliance components are intentionally gone from the current tree. Git
history and dated handoffs retain the old failure evidence; current product code
does not retain an executable path back to that topology.

Do not reintroduce Flyto2 runtime as a TurtleBot3 requirement.

## Installation

A clean TurtleBot3 may be installed from upstream ROS 2 / TurtleBot3
documentation. The physical lab currently uses ROS 2 Jazzy and a Burger base
with LDS-03 lidar.

Example upstream-only systemd units are in `deploy/native_ros2/`:

- `turtlebot3-bringup.service`
- `camera-v4l2.service`
- `slam-toolbox.service`
- `nav2.service`

They invoke `/opt/ros/jazzy` packages directly and contain no Flyto2 code.
They are site-configuration examples, not a proprietary robot runtime.

## Usage

Install this package on the computer that can see the robot's ROS 2 graph:

```bash
python3 -m venv .venv
.venv/bin/pip install -e .
```

Useful installed commands are external/lab tools such as:

```bash
flyto-robotics --help
flyto-ros2-readiness-probe --help
ros2_closed_loop_lab --help
gazebo_lab_driver --help
flyto-camera-gateway --help
flyto-resource-agent --help
```

The Generic ROS 2 Adapter on the selected AI Space execution host consumes standard interfaces such as:

- `nav2_msgs/action/NavigateToPose`
- `nav2_msgs/action/DriveOnHeading`
- `nav2_msgs/action/BackUp`
- `nav2_msgs/action/Spin`
- `geometry_msgs/msg/Twist` / `TwistStamped`
- `nav_msgs/msg/Odometry`
- `sensor_msgs/msg/LaserScan`
- standard camera, TF, and map topics

Besides the motions above it declares, when the graph has them:

| Capability | Interface | What it does |
| --- | --- | --- |
| `vision.observe` | `sensor_msgs/msg/CompressedImage` (`FLYTO_ROS2_CAMERA_COMPRESSED_TOPIC`, default `/camera/image_raw/compressed`) | returns one JPEG frame, at most 2 MB |
| `sensing.map` | `nav_msgs/msg/OccupancyGrid` (`FLYTO_ROS2_MAP_TOPIC`, default `/map`) | returns the latched map's cells, size, resolution and origin |
| `places.list` | host file (declared with `motion.navigate`) | returns the named places saved for this robot's map, as `evidence.places` and a `places` JSON artifact |
| `places.mark` | host file + `map_pose` | saves the robot's current map-frame pose (the same `map_pose` navigation arrival is judged on) under a name |

### Which resource an adapter serves

One computer's `ros2.generic` adapter serves exactly one resource: its
`FLYTO_ROS2_RESOURCE_ID` (else `ros2-<host>-<ROS_DOMAIN_ID>`), in its
`FLYTO_ROS2_DEPLOYMENT_MODE` (`simulation`, anything else is hardware). The
physical robot and its twin can sit behind the same endpoint, so neither the
id a host asks for nor the configured mode alone says which machine a job
would move.

- `adapter_provider.build_adapter(resource_id)` (and the process protocol's
  `--resource-id`) refuses any other id with `ResourceNotServed`, before any
  transport is opened, instead of labelling the adapter with the id it was
  handed.
- `GenericROS2Adapter.served_identity()` (process op `served_identity`)
  returns `{"resource_id", "deployment_mode"}`: the configured resource and
  `simulation` or `real`, confirmed against the live graph (a simulator iff it
  publishes `FLYTO_ROS2_SIM_MARKER_TOPIC`, default `/clock`). It raises
  `ServedIdentityError` when the mode and the graph disagree, when the graph
  shows no interfaces, or when simulation is claimed with the marker check
  disabled. A Flyto2 Cloud host asks it before every job and refuses unless
  both match what Cloud scored.

### Reaching a loopback-bound robot: the managed SSH forward

Everything the robot serves (rosbridge, camera) binds to `127.0.0.1` on the
robot, so the SSH local forward is the transport, not a workaround. The
adapter owns it (`flyto_robotics/ssh_transport.py`); nobody starts a tunnel
by hand, and a robot reboot or a Wi-Fi drop is recovered on its own.

| Variable | Meaning |
| --- | --- |
| `FLYTO_ROS2_SSH_HOST` | `[user@]host`, e.g. `ubuntu@flyto-robot.local`. Setting it turns the forward on and implies `FLYTO_ROS2_TRANSPORT=rosbridge` (an explicit `rclpy` is refused). |
| `FLYTO_ROS2_SSH_IDENTITY` | optional private key (`-i`, `IdentitiesOnly=yes`) |
| `FLYTO_ROS2_SSH_KNOWN_HOSTS` | optional known_hosts file (default: ssh's own) |
| `FLYTO_ROS2_SSH_FORWARDS` | remote loopback ports by name, default `rosbridge=9090`; e.g. `rosbridge=9090,camera=8080`. `rosbridge` is always forwarded. |
| `FLYTO_ROS2_SSH_LOCAL_PORTS` | optional pinned local ports, e.g. `rosbridge=19090`; otherwise an ephemeral `127.0.0.1` port |
| `FLYTO_ROS2_SSH_READY_TIMEOUT_S` | how long the first build waits for the forward (10) |
| `FLYTO_ROS2_SSH_LINGER_S` | how long the forward outlives its last adapter (30), so per-job adapters reuse it |

`FLYTO_ROSBRIDGE_URL` is derived from the forward (`ws://127.0.0.1:<local>`);
an explicit value that differs is refused rather than silently ignored.

Security: `BatchMode=yes` with password and keyboard-interactive auth off
(key only); `StrictHostKeyChecking=yes`, never `accept-new` — an unknown or
changed host key is a `failed` state whose `last_error` names how to trust the
robot once (`ssh-keyscan` compared with `ssh-keygen -lf` on the robot); every
forward is `127.0.0.1 -> 127.0.0.1` and any other address on either side is
refused; `-N` (no remote command). A private `ControlMaster` socket lets a
restarted host stop a master a crashed one left holding the ports.

Lifecycle: the adapter starts ssh (`ExitOnForwardFailure`, `ServerAliveInterval`),
supervises it, re-resolves the host (mDNS) on every attempt and reconnects
with exponential backoff and jitter (1 s doubling to 30 s, each delay drawn
from its upper half). Host-key and authentication failures do not retry on
their own; they wait for an operator `reconnect()`. The forward stops with
the last adapter that holds it (after the linger) and when the process exits.

While the forward is down every capability call fails fast — `refused`, with
`evidence.reason_code: "transport_unavailable"` and `evidence.transport` —
and nothing is sent; `safe_stop()` and `cancel()` return `failed` with the
same evidence.

Status: `GenericROS2Adapter.transport_status()` (process op
`transport_status`; `adapter_provider.transport_status()`, also
`discover_resource_manifests.transport_status`; manifest extension
`transport` on request) returns
`{transport, state: connected|reconnecting|failed|stopped, since, attempts,
last_error, error_code, host, resolved_address, forwards, accepting_calls,
rosbridge_ok}`. The presence watch notifies the host `transport_<state>` on
every change and retries the robot at once when the forward returns.

Operator refresh: `GenericROS2Adapter.reconnect()` (process op `reconnect`;
`adapter_provider.reconnect_resource(resource_id)`, also
`discover_resource_manifests.reconnect`) pauses new calls on the link, safe
stops any actuating call in flight (zero velocity, then cancel) and refuses
with `reason_code: "safe_stop_unconfirmed"` if that stop is not confirmed;
otherwise tears the forward and the rosbridge session down, establishes both
again, re-reads the graph and `served_identity()`, and returns the status
above plus `refused`, `topics_seen`, `served_identity` /
`served_identity_error`, `rosbridge_error` and `safe_stop`. It never raises
and may be repeated.

### Named places

Places are kept on the execution host, never on the robot and never in Flyto2
Cloud (`flyto_robotics/places.py`). Each is `{name, frame: "map", x, y, yaw}`;
names are free text (any script, 1 to 64 characters, no control characters),
unique per map ignoring case. Every write is atomic. A file that cannot be
read exactly as written is refused, never guessed at or overwritten.

| Setting | Default | Meaning |
| --- | --- | --- |
| `FLYTO_ROS2_PLACES_FILE` | unset | the places file, named outright |
| `FLYTO_ROBOTICS_DATA_DIR` | `$XDG_DATA_HOME/flyto-robotics` (`~/.local/share/...`; `%LOCALAPPDATA%\flyto-robotics` on Windows) | data dir; the file is `places/<resource id>/<map id>.json` under it |
| `FLYTO_ROS2_MAP_ID` | `default` | which map the places belong to. The SLAM map lives on the robot, so the host names it; change it when the map is rebuilt |

`motion.navigate` takes either `x` and `y` (with optional `yaw_radians`) or
`place`, never both. A place is resolved before anything moves: an unknown
name is refused with `evidence.known_places`, and an unreadable file is
refused, both with no motion. The goal sent to Nav2 is the stored pose,
heading included. Every navigate result carries `evidence.navigation_target`
(the goal, in the map frame); a call by place also carries
`evidence.resolved_arguments` (`x`, `y`, `yaw_radians`), the arguments its
arrival evidence is judged against.

### Leaving an obstacle's inflation before navigating

A robot that starts closer to an obstacle than Nav2's `inflation_radius` plus
its own radius sits in the costmap's cost gradient, and the stock controller
can fail to make progress from there ("Failed to make progress", recoveries,
an abort minutes later). Before sending a navigate goal the adapter decides
this from facts (`flyto_robotics/inflation_escape.py`): the nearest return in
the LiDAR's front sector against both radii, read from the costmaps' own
parameters. A pinned start backs off (only as far as the rear keeps the
clearance floor), passes one lateral waypoint on the side with more room, then
goes to the original goal, which stays the final pose. If no escape keeps the
floor the call is refused at once with `reason_code: no_escape_room` and the
measured clearances. The floor itself is never lowered.

| Setting | Default | Meaning |
| --- | --- | --- |
| `FLYTO_ROS2_INFLATION_ESCAPE` | `on` | `off` sends every goal unchanged |
| `FLYTO_ROS2_COSTMAP_NODES` | `/local_costmap/local_costmap,/global_costmap/global_costmap` | costmaps whose `plugins`, inflation layer and `robot_radius`/`footprint` are read; the largest radius of each kind is used |
| `FLYTO_ROS2_ESCAPE_MAX_BACKOFF_M` | `0.30` | longest back-off |
| `FLYTO_ROS2_ESCAPE_MARGIN_M` | `0.10` | clearance past the obstacle's edge beyond the robot radius |
| `FLYTO_ROS2_ESCAPE_MAX_LATERAL_M` | `1.0` | widest lateral offset before the escape is refused |
| `FLYTO_ROS2_NAVIGATE_THROUGH_ACTION` | `/navigate_through_poses` | used when present; otherwise the legs are sequential `NavigateToPose` goals |

Captures are read-only, read one message through rosbridge (the rclpy backend
refuses them), and return it as `evidence.capture`; the execution host keeps
and shows it.

### Evidence the adapter produces itself

A result keeps every key it always had (`capture`, `odom`, `motion_outcome`)
and adds what a host used to work out on its own (`flyto_robotics/provider_evidence.py`):

| Key | When | What |
| --- | --- | --- |
| `evidence_items` | every motion that ran | `passage.clearance`: the clearance the motion started with against this adapter's own floor (`FLYTO_ROS2_MIN_CLEARANCE_M`, 0.35 m) |
| | a completed motion | `robot.arrival`: odometry before, after and once settled (advance, retreat, rotate; `usable: false`, the host judges it), or the pose a navigation ended at. Emitted only when odometry says the robot settled; otherwise the host observes it as before |
| `artifacts` | a completed capture | `[{"kind", "media_type", "data_base64"}]`, the `flyto.capability-contract.v1` artifact transport: the photo as the camera sent it, and the map drawn as a picture (free white, walls black, unknown grey, north up, at least 600 px wide). JPEG with Pillow installed (`pip install flyto-robotics[capture]`), PNG otherwise |
| `recovery_context` | an advance or retreat that failed or timed out | why it stopped, distance asked / travelled along its heading / remaining, the nearest return at the stop, the floor, and the LiDAR sweep at the stop. The pack's declared `recovery` names it |

The item shapes are the ones Flyto2 Desktop projected until now, so a host that
passes them through reaches the same verdicts. Nothing waits on a failed
motion: the host's safe stop comes next.

### OpenRMF through the same contract

`open_rmf.fleet` is a second `flyto2.external_adapters` entry point (and
`flyto2.resource_discoverers`, active once `FLYTO_RMF_API_URL` is set). The
commanded resource is a fleet, `fleet:<name>`; the request names that fleet and
never a robot, so Open-RMF's dispatcher still runs the bid. A call waits, up to
its deadline, for Open-RMF to report the task `completed` (completed),
`failed`/`canceled`/`killed` (failed) or still running (timeout; the host's
cancel withdraws it). A finished `motion.navigate` or `motion.dock` reports a
`robot.arrival` item naming the fleet's own claim. Open-RMF has no fleet-wide
stop, so `safe_stop` is refused and the capabilities declare
`requires_safe_stop: false`: stop a machine on its own path (`motion.halt`).
The steps are the `fleet` pack in flyto-modules-robotics.

### Motion safety basis

Each robot declares what its motions rest on, with `FLYTO_ROS2_SAFETY_BASIS` on
the execution host:

| Value | Before a motion | Bounds |
| --- | --- | --- |
| `lidar_clearance` (default) | LiDAR clearance of at least `FLYTO_ROS2_MIN_CLEARANCE_M` (0.35 m) along the path the motion sweeps | the declared argument ranges |
| `operator_present` | no LiDAR; odometry is still required so Cloud can verify the motion | 0.05 m/s and 0.3 m per advance or retreat; π/2 per turn, at Nav2's own rotation speed (Spin takes no speed); navigation refused |

The floor is checked before a motion starts along the path that motion
sweeps (`flyto_robotics/path_clearance.py`), chosen by the capability
contract's `motion_kind`, not its name:

| `motion_kind` | Capability | Swept path | Starts when |
| --- | --- | --- | --- |
| `advance` / `retreat` | `motion.advance` / `motion.retreat` | corridor in the direction of travel: returns with `x > 0` and `abs(y) < floor + FLYTO_ROS2_ROBOT_RADIUS_M` (0.1) | the nearest of them is at least the floor away |
| `rotate` | `motion.rotate` | the footprint turning in place, all around | every return is at least `floor + FLYTO_ROS2_ROBOT_RADIUS_M` away |
| `planned` | `motion.navigate` | left to Nav2, the inflation escape and the in-motion guard | (not judged on a static distance) |

A refusal carries `evidence.reason_code: "path_blocked"` and
`evidence.path_clearance` (`motion_kind`, `shape`, `threshold_m`,
`clearance_m`, `limiting_side`, `limiting_bearing_rad`, and `sectors`: the
nearest return `ahead`, `left`, `behind`, `right`, `null` where there is
none), and its detail names the side, e.g. `LiDAR clearance 0.334m behind on
the retreat path is below the 0.350m motion safety minimum`. Without a sweep
the nearest return in any direction is judged as before.

Under `lidar_clearance` a
straight drive (`motion.advance`, `motion.retreat`) is then guarded on every
scan by its braking envelope (`flyto_robotics/braking_envelope.py`). A moving
robot cannot stop where it is told to: it covers `v * t` before the stop takes
effect and `v^2 / (2a)` while braking, so the clearance it has to stop at is

    floor + v * t_latency + v^2 / (2 * a_decel)

along the way it is going. The room it measures is, for every return at
`(x, y)` ahead in the frame of travel, `x - sqrt(floor^2 - y^2)` (a return beside
the path that it would pass closer than the floor counts too). The drive starts
at no more than `0.8 * v_max(room)`, with `v_max(room) = a(-t + sqrt(t^2 +
2 room / a))`. How its speed is governed after that is decided once per drive
(`braking_envelope.decide_governance`) from what the running server says
implements it, looked up in `PREEMPTION_BY_IMPLEMENTATION`: the behavior plugin
class it loaded (`<behavior>.plugin` on `FLYTO_ROS2_BEHAVIOR_SERVER`, default
`/behavior_server`), else the action type it serves; and from whether the
transport can send a second goal:

- `planned_before_send`: the server cannot take a preempting goal. Nav2's
  `DriveOnHeading` and `BackUp` cannot in any release (its behavior server
  answers one with "feature is currently not implemented. Aborting and
  stopping", then restarts from rest), nor can any undeclared type or the rclpy
  transport. The distance is planned with the speed, as `min(requested, room -
  stopping_distance(v))`, so the drive ends by itself where it can still come
  to rest at the floor. Nothing is sent while it runs. A drive that ends there,
  short of the request, reports `obstacle_blocked` like a guard stop.
- `preemptive_resend`: a server declared preemptible gets the whole distance
  and is re-sent slower (a preempting goal with the rest of its distance) as
  the room shrinks.

Either way the drive is stopped (zero velocity, cancel, zero) while the room is
still at least its stopping distance, or when the LiDAR is unreadable or quiet
for `FLYTO_ROS2_STOP_SCAN_TIMEOUT_S`. A drive that could not stop at the floor
even at 0.02 m/s, or would have under 0.01 m left to drive, is refused before
it moves.

| Setting | Default | Meaning |
| --- | --- | --- |
| `FLYTO_ROS2_STOP_LATENCY_S` | `0.5` | end-to-end stop latency. The measured budget (worst recent scan interval + scan delivery delay, twice, for the scan in and the cancel out + `FLYTO_ROS2_STOP_CONTROL_PERIOD_S` 0.1 + `FLYTO_ROS2_STOP_ACTUATION_S` 0.1) replaces it only when larger |
| `FLYTO_ROS2_STOP_DECEL_MPS2` | `0.5` | achievable deceleration. The robot's declared limit (`FLYTO_ROS2_DECEL_PARAMETER`, default `/velocity_smoother:max_decel`) replaces it only when smaller |
| `FLYTO_ROS2_STOP_SCAN_TIMEOUT_S` | `0.5` | longest gap between scans before a guarded drive stops (at least three measured scan periods) |

A straight drive's `motion_outcome` carries `braking` (the profile and where
each value came from, the requested and commanded speeds, `governance` (mode,
action type, whether it preempts, and the fact that decided it), `plan` (the
speed and distance sent against those asked, from the room at send), every
slowdown, `ended: planned_stop` for a drive that ended at its planned stop
point, and the trip with its room, speed, stopping distance and the distance
the robot actually covered after it) and every motion's carries `stop_clearance`: the
nearest return at rest, its bearing and side (`ahead`, `left`, `right`,
`behind`) from the way the robot was going, and whether the floor held, over
every direction and along the path.

The basis is part of each motion capability's declared observations (`/scan`
or `operator:present`), so it is shown when the capability is approved and a
changed basis needs a new approval. `operator_present` relies on Cloud holding
every actuating capability for an operator's Run. Any other value refuses
motion.

No adapter URL or Flyto2 credential is stored on the robot. The installed
package exposes `ros2.generic` to the built-in AI Space plugin host and also
ships `flyto2-adapter-provider-ros2-generic` for process-based hosts such as
optional Flyto2 Runtime.

## Simulation and deterministic verification

The self-contained Gazebo world remains a first-class test surface. It lets the
same controller/safety/evidence code be exercised without claiming physical
acceptance.

Common checks:

```bash
make verify
make soak
make gazebo-lab
make gazebo-matrix
make gazebo-shortcut
make ai4all-showcase
```

Generated reports and simulation output are evidence about simulation only.

## Physical acceptance

A physical closure is stronger than "nodes are running".

The acceptance ladder is:

1. robot contains zero Flyto2 runtime;
2. native ROS 2 survives cold boot and exposes fresh odom/lidar/camera/TF;
3. Nav2/SLAM standard action surface is active without fabricated localization;
4. an external computer discovers the standard capabilities;
5. in a physically clear area, one bounded motion is executed;
6. interruption/cancel produces a safe stop;
7. odometry/LiDAR/camera evidence is collected independently;
8. Flyto2 Cloud verifies the original objective;
9. only then may War Room mark the task complete.

Current physical status is recorded in the 2026-09-21 handoff in
`flyto-cloud`. No new motion should be sent merely to prove source code.

## API

The supported production-facing boundary is semantic ROS 2 execution and
evidence, not Pi lifecycle management. Start with
`flyto_robotics.ros2_action_executor`, `flyto_robotics.ros2_execution`,
`flyto_robotics.ros2_execution_evidence`, `flyto_robotics.ros2_pairing`,
and the versioned JSON contracts under `contracts/`. Robot-appliance
lifecycle, delivery and recovery modules have been removed from the current
source tree.

## Development

Use the repository gates before landing behavior or packaging changes:

```bash
make verify
flyto-index verify . --strict
```

For physical work, source checks never substitute for hardware acceptance.
Generated ROS/Gazebo/evidence output stays untracked.

## Contributing

Read `AGENTS.md`, `ARCHITECTURE.md`, `DECISIONS.md`, and `STATE.md`
before changing runtime boundaries. New integrations must preserve the split
between execution computer and commanded equipment and must not require Flyto2
software on the robot.

## Documentation

| Document | Purpose |
|---|---|
| [Architecture](ARCHITECTURE.md) | current external-adapter boundary |
| [Installation](docs/INSTALLATION.md) | standard robot + external computer setup |
| [Capabilities](docs/CAPABILITIES.md) | capability/controller vocabulary |
| [Contracts](docs/CONTRACTS.md) | versioned execution/evidence contracts |
| [Demo](docs/DEMO.md) | Gazebo and lab workflows |
| [Virtual Robot Lab](docs/VIRTUAL_ROBOT_LAB.md) | simulation matrix |
| [Showcase evidence](docs/SHOWCASE_EVIDENCE.md) | evidence interpretation |
| [Historical handoffs](handoffs/_registry.md) | old Pi/runtime work and findings |

Dated handoffs and Git history preserve retired-appliance evidence; current
installation documentation describes only the external-adapter architecture.

## Security

This is a research/lab integration toolkit, not a certified safety controller.
Real deployments need an independent emergency stop and site-specific safety
engineering.

Flyto2 policy, permission, leases, evidence binding, and objective verification
remain separate from low-level ROS control. The model never earns authority by
claiming that it has a capability, and the robot never earns task-completion
authority by reporting action success.

## License

Apache-2.0. See `LICENSE`.
