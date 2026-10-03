# Adversarial review of #47: fleet cancel, task ids, waypoints, map upscale

Owner: claude
Branch: claude/evidence-hardening
Date: 2026-10-04

## Confirmed defects fixed

- **A declined cancel read as done.** `OpenRmfAdapter.cancel` ignored the
  dispatcher's answer, so `{"success": false, "errors": [...]}` returned
  `completed`. For a fleet task this cancel is the only stop there is (the
  adapter refuses `safe_stop`, and Core's host calls cancel then safe_stop on
  every timeout/failure), so the host recorded `cancel=completed` while the
  task kept running. It now returns `refused` with RMF's errors.
- **A dispatch with no task id.** A `dispatch_task` answer with a state but no
  `booking.id` was reported `completed` (conformance build) or polled
  `/tasks//state` until the deadline (fleet build), and the following cancel
  sent `task_id: ""` and reported `completed`. Such a call is now `failed`
  (it can be neither followed nor withdrawn) and cancel refuses it.
- **Task id in the URL path.** The dispatcher's id is now quoted as one path
  segment (`/tasks/..%2Ffleets/state`, not `/tasks/../fleets/state`).
- **Waypoint validated only by the pack.** The adapter turned any value into a
  place with `str()` (a list became `"['ward_3']"`). It now refuses non-text,
  empty, over 128 characters, and C0/C1 control characters, and trims, the
  same rule as the pack, because a host can call the adapter directly.
- **Map upscale without a bound.** `provider_evidence._scaled` scaled any map
  to at least 600 px wide. A 1 x 4,000,000 occupancy grid (within the
  adapter's `MAX_MAP_CELLS`) asked for 1.44e12 bytes; 1 x 40,000 asked for
  14.4e9. The upscale now stops at `MAP_MAX_PIXELS` (16,000,000). Ordinary
  maps are unchanged (384 x 384 still becomes 768 x 768, so the Desktop
  byte-for-byte test still passes).

## Verified

- New tests fail on the #47 code and pass now (11 RMF cases; the thin-map
  case was checked by arithmetic only on the old code, since running it
  allocates ~14 GB).
- `make verify`: ruff clean, 1144 passed, 1 skipped.
- `flyto-index verify --strict`: 20 pass, 0 warn, 0 fail.

## Not verified

- No real Open-RMF: the shape of a declined `cancel_task` answer
  (`success: false`) is from the RMF API schema, not observed.
- No robot, no Gazebo. MCP `task(action='validate')` not run (the MCP server
  is pinned to another repo); ruff and pytest were run directly.
