# Host-side driver layer clarification and module pack join

Owner: claude
Branch: claude/host-driver-layer
Date: 2026-10-04

## What changed

- `DECISIONS.md`: 2026-10-04 clarification entry (not a reversal).
  `flyto-robotics` is the host-side driver layer next to the equipment
  (execution host or companion computer), driving the robot's native ROS 2 /
  Nav2 stack; never firmware; nothing from Flyto2 on the robot (owner
  confirmed 2026-10-04: robots stay stock TurtleBot3). It owns the 0.35 m
  floor, refuse-never-clamp, sim/physical mismatch refusal and safe stop.
  States the link-loss limit honestly. Capabilities are declared to Flyto2 by
  the `flyto-modules-robotics` `@register_module` pack, not by this repo.
- `README.md`, `ARCHITECTURE.md`, `STATE.md`: same clarification; STATE now
  says the 0.143 m clearance reading was superseded by the 2026-10-02 motion.
- `flyto_robotics/adapter_provider.py`: `MODULE_PACK = "robotics"` and
  `MANIFEST_EXTENSIONS = ("module_pack",)`. `_manifest` adds
  `"module_pack": "robotics"` only when the host asks for it:
  `discover_resource_manifests(..., manifest_extensions=["module_pack"])`,
  the process `describe` op's `manifest_extensions`, or
  `--discover --manifest-extension module_pack`. The discoverer also carries
  `.module_pack` and `.manifest_extensions` attributes (like `.watch`).
- `tests/test_module_pack_join.py`: new tests for the opt-in join.

## Why

The task asked for an unconditional additive `module_pack` field. That would
break every released Desktop: flyto-cloud's `ResourceManifestDTO` is
`extra="forbid"`, and `local/resource_discovery.py` validates each discovered
manifest with it inside a broad `except`, so an unknown field silently drops
the robot from discovery. Old Desktops must keep working (admission rule 2),
so the field is opt-in and the default manifest is byte-for-byte the released
shape. A new host should check `"module_pack" in
getattr(discover, "manifest_extensions", ())` and then pass the kwarg (an
older flyto-robotics would raise TypeError on it), or simply read
`getattr(discover, "module_pack", None)`.

No ROS behaviour, argument bound or safety value changed.

## Verified

- `make verify` (ruff, 1058 pytest, assets, dry runs, contracts, pairing,
  execution grant) against this worktree's code: pass.
- `flyto-index verify --strict` in the worktree: 20/20 PASS, no WARN or FAIL.

## Not verified

- No host consumes `module_pack` yet; the flyto-cloud side (DTO accepting it,
  passing the extension) is not part of this change.
- No physical or simulator run; nothing at the ROS layer changed.

## Follow-ups

- flyto-cloud host: opt in via the attribute check above and allow the field
  in its DTO, keeping released Desktops on the default shape.
