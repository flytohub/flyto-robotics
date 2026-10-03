# Adapter-produced evidence and artifacts; Open-RMF as a fleet resource

Owner: claude
Branch: claude/provider-evidence
Date: 2026-10-04

## What changed

- `flyto_robotics/provider_evidence.py` (new, pure): `clearance_item`
  (`passage.clearance` against the adapter's own `FLYTO_ROS2_MIN_CLEARANCE_M`
  floor), `arrival_item` (`robot.arrival`, odometry before / after / post_stop,
  `usable: false` for judged motions; the end pose for navigate), the
  occupancy-map drawing (`render_map`: JPEG via Pillow, stdlib PNG otherwise),
  `capture_artifacts` and `recovery_context`.
- `generic_ros2_adapter.py`: `GenericROS2Adapter.invoke` adds, additively,
  `evidence_items`, `artifacts` (captures, beside the legacy `capture`) and
  `recovery_context` (failed/timed-out advance/retreat). It observes `before`
  itself (kept per call id across a timeout), and for a completed judged
  motion waits for odometry to show the robot still (`SETTLE_SECONDS` 1.0)
  before reading `post_stop`; a backend that cannot report stillness gets no
  arrival item (the host observes as before). Nothing waits after a failure.
  A repeated call id returns the same enriched result. New
  `_ObservationState.held_observation()` (no-wait snapshot).
- `open_rmf_adapter.py`: `build_adapter(resource_id)` for a `fleet:<name>`
  resource (entry point `open_rmf.fleet`), `fleet_name` pinned in the
  dispatch request (still never a robot), waiting for RMF's terminal task state
  up to the deadline (completed / failed / timeout), no second dispatch for a
  repeated call id, `robot.arrival` for a finished navigate/dock,
  `deployment_mode` from `FLYTO_RMF_DEPLOYMENT_MODE`, and
  `discover_fleet_manifests` (entry point, active only when
  `FLYTO_RMF_API_URL` is set; `module_pack: "fleet"` only on request). A fleet
  resource declares `motion.navigate_to_waypoint` instead of `motion.navigate`
  (new metadata row in `adapter_contract.py`); the unbound conformance
  `build()` is unchanged.
- `setup.py`: both entry points, `capture` extra (`Pillow>=10`), 0.2.0
  (`package.xml`, `__version__`). README sections "Evidence the adapter
  produces itself" and "OpenRMF through the same contract"; CHANGELOG 0.2.0.

## Why

Cloud/Desktop projected these items from adapter observations with the
robot's own numbers copied in (the 0.35 m floor, the map shading). The adapter
now hands them over so a host can pass them through (Cloud's
`claude/provider-neutral` dispatcher merges `evidence_items` additively and
keeps its own floor only for kinds the adapter did not report). The shapes are
transcriptions of Desktop's, so verdicts do not change.

`motion.navigate_to_waypoint`: with the fleet pack and the robotics pack both
installed, two different contracts under `motion.navigate` make flyto-core's
capability host report `ambiguous` and fail closed (default 60 s deadline,
no evidence), and the manifest entry goes to the lowest module id
(`fleet.navigate`). Observed in a run before the rename: the fleet call got
`deadline_seconds: 60.0`; after it, 600.0.

## Verified

- `make verify` (ruff 0.16.4, pytest 1131 passed, assets, dry runs, lab,
  facility, pairing, grant): exit 0, Python 3.11 venv.
- `tests/test_provider_evidence.py`: items equal Desktop's projection
  (transcribed); the map JPEG is byte-for-byte Desktop's `_map_jpeg`; the PNG
  fallback decodes to the same shades; adapter enrichment with a fake backend.
- `tests/test_open_rmf_adapter.py`: fleet waiting, timeouts, no re-dispatch,
  discovery, the new id.
- `flyto-index verify . --strict`: 20 pass, 0 warn, 0 fail.
  `flyto-index verify-workspace` with this worktree and the
  flyto-modules-robotics worktree: PASS.
- Cross-repo, ad hoc (not committed): real `OpenRmfAdapter` (fake RMF API) and
  real `GenericROS2Adapter` (fake backend) through flyto-core main's
  `CapabilityHost` and the flyto-modules-robotics 1.1.0 steps: fleet navigate
  completed with deadline 600 s; map artifact kept as JPEG by the host;
  advance records the adapter's clearance and arrival items and Core's judge
  verdicts.
- Pre-change: `flyto-index context` / `impact` on `GenericROS2Adapter` and
  `OpenRmfAdapter` (index was stale; read the source directly as well).

## Not verified

- No robot, simulator or Open-RMF deployment contacted (fakes only). The
  rosbridge/rclpy backends' real `held_observation` and stillness under load
  are untested on hardware.
- Open-RMF's acceptance of `fleet_name` in `dispatch_task_request` and the
  exact `/tasks/{id}/state` status strings against a live API server.
- MCP `task(action='validate')`: the MCP server is pinned to another root.
- Cloud consuming `artifacts` (its `claude/provider-neutral` branch reads a
  singular `evidence.artifact` with `mime`; Core's spec is `artifacts[]` with
  `media_type`). Cloud still works through the legacy `capture`.
- Payload: a released Desktop with an old pack (1.0.0) puts the photo's
  `artifacts` base64 into the step output; pack 1.1.0 strips it.

## Follow-ups

- Cloud: read `evidence.artifacts` per the Core spec; drop its own map
  rendering and clearance/arrival projection once released Desktops carry this
  adapter.
- Open-RMF live acceptance.
