<p align="center">
  <img src="docs/assets/flyto2-logo.png" alt="Flyto2" width="96" height="96">
</p>

# Flyto2 Robotics

External ROS 2 adapter, deterministic control, simulation, safety, and evidence
toolkit for Flyto2.

**A robot is standard ROS 2 equipment. It is not a Flyto2 appliance.**

Production topology:

```text
Flyto2 Cloud / War Room
        |
        | goal, policy, approved capability
        v
AI Space computer
(Mac / laptop / Steam Deck / mini PC)
        |
        | Generic ROS 2 Adapter
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

Equipment adapters are installed on the AI Space execution computer and loaded
through a host-neutral adapter boundary. The built-in Python AI Space host uses
`flyto2.external_adapters` / `flyto2.resource_discoverers` entry points; process
hosts such as optional Flyto2 Runtime can use the versioned
`flyto2.adapter-provider.v1` protocol. Adapter identity maps to the provider,
for example `ros2.generic` maps to `flyto2-adapter-provider-ros2-generic` on the
process-host path. The adapter owns ROS 2, rosbridge, OpenRMF, camera-stream and
other transport details; the selected execution host owns assignment-scoped
authority and lifecycle; Cloud owns resource/capability inventory, approval,
routing, evidence and verification.

Provider discovery is passive. An installed provider may report a
`flyto.resource-manifest.v1`, but discovery never grants motion or other
effects. For execution, Runtime binds the exact commanded resource and approved
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
