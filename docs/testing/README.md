# Flyto Robotics verification package

This directory is the human-readable entry point for the repeatable test
evidence. Generated Gazebo output remains under `results/` and is intentionally
untracked; every report can be regenerated from versioned scenarios, schemas,
models, launch files, and scripts.

## Documents

- [Gazebo test plan](GAZEBO_TEST_PLAN.md): layers, adversarial scenario,
  independent oracles, acceptance gates, and exact commands.
- [Verified results — 2026-07-29](TEST_RESULTS_2026-07-29.md): measured results,
  image inventory, provenance, and discovered failures.
- [External evaluator guide](EVALUATOR_GUIDE.md): safe, bounded hands-on
  walkthrough and feedback checklist.

## Visuals

- [Closed-loop verification ladder](../images/test-closure.svg)
- [Gazebo lab topology](../images/gazebo-lab-topology.svg)
- [Evidence and trust chain](../images/evidence-chain.svg)

## One-command entry points

```bash
make verify
make soak
make gazebo-lab
make gazebo-matrix
```

`make verify` requires Python only. `make gazebo-lab` and
`make gazebo-matrix` use the reference ROS 2 Jazzy / Gazebo Harmonic container.
The matrix performs three independent cold starts by default.

Passing simulation proves the implemented contracts and scenarios. It is not
medical-device certification and does not replace physical robot, site, or
human-factors acceptance testing.
