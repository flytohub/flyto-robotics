# The twin's demo world lives on main

Owner: claude
Branch: claude/twin-demo-world (off origin/main ff17794)
Date: 2026-10-07

## What changed

- `sim/twin/worlds/flyto_demo_room.world`: the demo room (5 m x 4 m, one box
  whose near face is 0.75 m ahead of the start pose -1.8, 0 facing +x).
- `sim/twin/compose.yaml`: mounts `./worlds` read-only at `/opt/flyto-twin/worlds`.
- `sim/twin/README.md`: how to start the demo room.
- `tests/test_twin_demo_world.py`: pins the mount and the box geometry.

## Why

Both files existed only as uncommitted work in the `.claude/worktrees/sim-twin`
worktree, yet the running `flyto-turtlebot3-twin` container bind-mounts that
worktree's `sim/twin/worlds`, and `flyto-demo-erp/scripts/demo-reset.sh`
defaults `TWIN_DIR` to that worktree and requires the world file there. The
demo depended on files no branch carried.

## Verified

- `make PYTHON=.venv/bin/python verify`: exit 0, 1623 passed, 1 skipped.
- `flyto-index verify . --full-scan --strict`: exit 0.
- The world file is byte-identical to the one the running container mounts.

## Not verified

- The twin was not restarted on this world from the main checkout.
- `flyto-demo-erp/scripts/demo-reset.sh` still defaults `TWIN_DIR` to the
  `sim-twin` worktree; the worktree is kept (detached at main) so the running
  container's bind mount and that default keep resolving. Pointing the
  default at `flyto-robotics/sim/twin` is a flyto-demo-erp change, not made.
