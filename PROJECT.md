# Flyto2 Robotics Project

## Purpose

Provide the external robotics execution, deterministic safety/control, simulation,
and evidence layer for Flyto2 without requiring proprietary runtime software on
the robot.

## Product position

- TurtleBot3 / robot: standard ROS 2 executor.
- AI Space computer: Flyto2 execution host and adapter location.
- Flyto2 Cloud / War Room: authority, routing, task state, evidence binding and
  independent verification.

## Owned surfaces

- ROS 2 action/topic adapters and deterministic controller utilities.
- Physical execution/evidence contracts.
- Sensor freshness, clearance and safe-stop logic.
- Gazebo worlds, labs, fault injection and independent ground truth.
- Physical acceptance helpers and regression evidence.
- External camera/resource adapters where appropriate.
- Standard-ROS robot startup examples used by the lab.

## Non-goals

- Installing a Flyto2 scheduler or job runner on a robot.
- Treating a robot as a Flyto2 queue worker.
- Replacing ROS 2, Nav2, SLAM, MHS, VLA or vendor motor-control stacks.
- Storing Flyto2 task state or credentials on a robot.
- Letting action success directly complete a Flyto2 task.

## Historical source

Pi-side job-runner/lifecycle/recovery source remains temporarily for regression
and historical evidence. It is not shipped as the production robot runtime.
