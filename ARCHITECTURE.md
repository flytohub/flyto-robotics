# Flyto2 Robotics Architecture

## Current product boundary

Flyto2 Robotics is the **external physical-execution adapter and verification
layer**, not software that must be installed inside a robot.

```text
Intent / Space Task
       |
       v
Flyto2 Cloud
- capability approval
- permission / policy
- assignment / lease
- task state
- evidence binding
- verification
       |
       v
AI Space computer  <---- execution host
       |
       | approved capability call
       v
Generic ROS 2 Adapter
       |
       | standard ROS 2
       v
Robot resource      <---- commanded equipment
       |
       +--> odom / scan / TF / camera
       +--> Nav2 action result
       |
       v
Execution evidence
       |
       v
Cloud independent verifier
```

Execution host and commanded equipment are deliberately different resources.

A TurtleBot3 can be erased, reinstalled from upstream documentation, and brought
back into Flyto2 by rediscovering its standard ROS 2 graph. No Flyto2
provisioning on the robot is part of that recovery.

## Robot-side contract

Allowed robot-side components include:

- Ubuntu / Raspberry Pi OS;
- ROS 2;
- upstream TurtleBot3 packages;
- lidar and camera drivers;
- odometry, TF, robot_state_publisher;
- Nav2;
- SLAM Toolbox;
- generic DDS / Zenoh / rosbridge transport;
- other general-purpose open-source ROS 2 components.

The accepted architecture does not require:

- a Flyto2 job runner;
- Flyto2 credentials;
- a Flyto2 HTTP delivery gateway;
- a Flyto2 scheduler or task database;
- a Flyto2 evidence database;
- a Flyto2-specific ROS node.

## External adapter

The adapter computer is allowed to know Flyto2 policy and task context. The
robot is not.

The shallow southbound surface is standard ROS 2:

| Capability intent | Standard ROS 2 surface |
|---|---|
| navigate | Nav2 `NavigateToPose` |
| advance | Nav2 `DriveOnHeading` |
| retreat | Nav2 `BackUp` |
| rotate | Nav2 `Spin` |
| halt | zero `Twist` / `TwistStamped` |
| pose evidence | `/odom`, TF |
| obstacle evidence | `/scan` |
| visual evidence | standard camera topics |
| world/navigation state | `/map`, Nav2 state |

The adapter may discover a capability, but discovery is not approval.

## Control authority

Autonomous work has one authority path:

```text
Space Task
 -> workflow on selected computer
 -> approved adapter
 -> standard ROS 2 capability
 -> robot
```

There is no autonomous browser-to-robot relay and no Pi-side Flyto2 queue
consumer.

Manual dead-man / emergency control is a distinct operator authority. It may
use the same standard ROS 2 interfaces, but it cannot grant or widen autonomous
capabilities and cannot become a second scheduler.

## Controller and safety code

The deterministic controller, scan-clearance logic, cmd_vel type negotiation,
sensor freshness gates, Nav2 action executor, and evidence builders remain in
this repository because they are useful on an external adapter computer and in
simulation.

Important invariants:

- stale/missing observations fail closed;
- motion is bounded;
- stop is unconditional;
- cancellation must actually withdraw/stop the active action;
- retries cannot create duplicate physical effects;
- actuator receipt is never objective evidence by itself.

For a production deployment, independent hardware safety/E-stop remains outside
this software.

## Evidence boundary

The runtime distinguishes:

1. **execution receipt** — what action ran and how it terminated;
2. **observations** — odometry, range, camera, TF/map state;
3. **mission evidence** — observations bound to the exact task/goal/execution;
4. **verdict** — Cloud's independent conclusion.

```text
Nav2 SUCCEEDED
   !=
Task COMPLETED
```

A successful action may still fail the user objective.

Existing schemas such as ROS 2 execution/evidence/grant contracts remain useful
for this separation even though the Pi-side delivery transport is retired.

## Simulation

Gazebo Harmonic remains an independent execution/evidence environment.

Simulation and physical hardware should exercise the same controller and
evidence semantics where practical, but a Gazebo pass never upgrades a physical
acceptance result.

The adversarial lab intentionally tests:

- obstacle and sensor faults;
- cancellation;
- duplicate/replayed commands;
- safe-stop behavior;
- independent world-pose truth;
- evidence integrity and reproducibility.

## Native robot startup examples

`deploy/native_ros2/` contains standard systemd examples built during the
2026-09-21 TurtleBot3 migration. They launch only upstream ROS 2 packages.

A key physical finding is preserved there: on the lab Raspberry Pi, DDS
participants created before Wi-Fi had a global IPv4/default route could remain
undiscoverable after cold boot. The examples wait for a usable route before
starting ROS.

This is system configuration, not a Flyto2 robot runtime.

## Retired appliance implementation

The Pi-side Flyto2 appliance implementation has been removed from the current
source tree: job runner, delivery gateway, lifecycle installer/profile registry,
doctor, recovery portal, robot credential provisioning, and their robot-side
systemd units are no longer product code.

Historical behavior and failure evidence remain available in Git history and
dated handoffs. Current integrations must not recreate those components.

## Repository role

This repository should evolve toward:

- external ROS 2 adapter implementations;
- physical evidence contracts;
- deterministic safety/control utilities;
- robot/VLA adapter conformance;
- Gazebo and real-hardware acceptance;
- compatibility bridges to standard robotics ecosystems.

It should not evolve toward a proprietary OS layer on the robot.
