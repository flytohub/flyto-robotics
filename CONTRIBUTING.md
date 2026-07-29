# Contributing

## Before changing code

1. Read `AGENTS.md`, `ARCHITECTURE.md`, `DECISIONS.md`, and `STATE.md`.
2. Use flyto-indexer search, structure, task planning, and impact analysis.
3. Identify the contract, primitive, workflow, adapter, and test boundaries.
4. Confirm that no patient data, credentials, or production endpoint is added.

## Design requirements

- Keep capability primitives atomic, deterministic, and reusable.
- Compose business scenarios with immutable workflows.
- Keep ROS 2 and hardware imports out of contracts and the pure controller.
- Make invalid input, stale sensors, and nearby obstacles stop motion.
- Preserve backward compatibility for versioned JSON contracts.

## Verification

Run:

```bash
make verify
```

For model, world, bridge, or launch changes, also build with `colcon`, validate
the model with `gz sdf -k`, and load the complete world in a Jazzy/Harmonic
environment. Run flyto-indexer task validation, unstaged impact analysis, and
full strict verification after changes.

Generated evidence belongs in `results/` and must remain untracked.
