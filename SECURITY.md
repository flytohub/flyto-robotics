# Security

## Reporting a vulnerability

Email **security@flyto2.com**. Include what you observed, how to reproduce it,
and the expected impact. Do not test against a robot or installation that is not
yours; physical machines can move.

Do not open a public issue for an exploitable defect.

## Current trust boundary

This repository contains external-computer robotics adapters, deterministic
controller/safety utilities, Gazebo labs, ROS 2 readiness/execution code, and
evidence contracts.

The production robot is standard ROS 2 equipment. It does not hold a Flyto2
credential, claim Flyto2 jobs, run a Flyto2 scheduler, expose a Flyto2 delivery
gateway, or decide whether a Flyto2 task is complete.

The authority path is:

```text
Flyto2 Cloud
  -> approved capability / policy / resource binding
  -> external AI Space computer
  -> standard ROS 2
  -> robot
  -> observations / execution evidence
  -> independent Cloud verification
```

A model, adapter, simulator, or robot cannot grant itself authority by declaring
that it has a capability. A ROS/Nav2 action result is execution evidence, not a
mission verdict.

## High-value reports

Particularly useful reports include:

- a path that bypasses capability approval, permission, bounds, or execution
  grants;
- replay or retry behavior that can duplicate a physical effect;
- cancellation that reports success without withdrawing/stopping the action;
- stale/missing odometry, LiDAR, camera, TF, or Nav2 state being accepted as
  fresh;
- a stop path that can be overtaken by later motion;
- evidence that can be rebound to the wrong resource, task, goal, or execution;
- simulation evidence being accepted as physical evidence without explicit
  provenance;
- camera or sensor evidence being treated as calibrated when calibration is
  absent;
- any path that lets action success directly complete the user objective.

## Robot-side security rule

A clean robot must be reinstallable from upstream ROS 2 / vendor documentation
without cloning or installing this repository.

Do not put Flyto2 credentials, task state, adapter secrets, Cloud endpoints,
job queues, or Flyto2-specific daemons on the robot.

Generic site configuration for ROS 2, Nav2, SLAM, camera drivers, DDS/Zenoh, and
independent hardware E-stop/safety systems remains outside Flyto2 task
authority.

## Secrets and logs

Examples, tests, logs, evidence, and configuration committed to this repository
must contain no production credentials or customer data. External adapters must
not expose raw secrets in evidence or diagnostic output.

## Scanning

CodeQL runs on pushes and pull requests to `main`, plus scheduled scans.
Secret scanning, push protection, and dependency security updates are enabled.

A quiet scanner is not proof of physical safety. Physical acceptance remains a
separate gate.
