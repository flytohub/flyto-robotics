# The adapter owns the SSH forward; the floor applies along the swept path

Owner: claude
Branch: claude/managed-robot-transport
Date: 2026-10-06

## What changed

Two changes in one 0.6.0 release.

### 1. Managed SSH transport (`flyto_robotics/ssh_transport.py`)

The robot's services start on boot, but the Mac reached its loopback-bound
rosbridge only through an SSH local forward someone started by hand; after a
robot reboot (2026-10-06 01:00) it died and nothing restored it. The old
`deploy/robot-tunnel.sh` (removed in #32) was manual too. The design rule from
`2026-08-27-camera-stream-half-nav2-and-mapping-as-a-job.md` stands: everything
the robot serves binds to loopback; the tunnel is the transport.

- Config by data: `FLYTO_ROS2_SSH_HOST` (implies rosbridge; an explicit
  `FLYTO_ROS2_TRANSPORT=rclpy` with it is refused), `FLYTO_ROS2_SSH_IDENTITY`,
  `FLYTO_ROS2_SSH_KNOWN_HOSTS`, `FLYTO_ROS2_SSH_FORWARDS` (default
  `rosbridge=9090`), `FLYTO_ROS2_SSH_LOCAL_PORTS` (else ephemeral),
  `FLYTO_ROS2_SSH_READY_TIMEOUT_S` (10), `FLYTO_ROS2_SSH_LINGER_S` (30).
  `FLYTO_ROSBRIDGE_URL` is derived; a contradicting explicit value is refused.
- Security: options table `SSH_OPTIONS` (BatchMode, StrictHostKeyChecking=yes,
  password/kbd-interactive off, ExitOnForwardFailure, ServerAliveInterval 5 x3,
  private ControlMaster socket in a 0700 per-user dir, no agent/X11 forwarding),
  `-N -T`, host validated and placed after `--`, forwards only
  `127.0.0.1:<local> -> 127.0.0.1:<remote>`; any other address refused.
  stderr is classified by `_ERROR_TABLE`: host key unknown/changed and auth
  refused are fatal (`failed`, no automatic retry, `last_error` says how to
  trust the host once); unresolved/refused/timeout/busy port are transient.
- `ManagedTunnel`: supervisor thread, re-resolves the host every attempt,
  backoff 1 s doubling to 30 s drawn from the upper half (jitter), re-picks an
  ephemeral local port that was busy, `refresh()` (operator), refcounted
  `acquire()`/`release()` with linger, `atexit` teardown, `CallGate` (admitted
  calls, actuating or not; paused during an operator refresh).
- `generic_ros2_adapter`: `build()` acquires the forward and builds the
  rosbridge backend on its URL (`url_source`, re-read on every reconnect); the
  first build waits for the forward once, later builds while it is down return
  at once. `invoke` fails fast with `refused` + `evidence.reason_code:
  "transport_unavailable"` + `evidence.transport`; `safe_stop`/`cancel`
  return `failed` with the same. A rosbridge session the forward dropped is
  reopened on the next call. `transport_status()`; `reopen()` (ROS link only,
  what the old `reconnect()` did); `reconnect()` is now the operator refresh.
- `adapter_provider`: `transport_status()`, `reconnect_resource(resource_id)`
  (both also attributes of `discover_resource_manifests`), process ops
  `transport_status` and `reconnect`, opt-in manifest extension `transport`;
  discovery is enabled by `FLYTO_ROS2_SSH_HOST` alone; the presence watch uses
  `reopen()`, is woken when the forward returns and notifies the host
  `transport_<state>`.

Operator refresh interface (for the Cloud host route and Mission Station):

- In-process: `adapter.reconnect() -> dict`, or
  `adapter_provider.reconnect_resource(resource_id) -> dict` (uses the
  presence watch's adapter, else a presence-only one for the call; raises
  `ResourceNotServed` for another id). Process protocol:
  `{"id": n, "op": "reconnect"}` -> `{"ok": true, "result": {...}}`.
- Result: `transport` (`ssh`|`direct`), `state`
  (`connected`|`reconnecting`|`failed`|`stopped`), `since` (ISO UTC, null for
  direct), `attempts`, `last_error`, `error_code`, `host`,
  `resolved_address`, `forwards`, `accepting_calls`, `rosbridge_ok`,
  `rosbridge_error`, `topics_seen`, `served_identity`
  (`{resource_id, deployment_mode}` or null), `served_identity_error`,
  `refused`, `safe_stop` (null, or `{calls, outcomes, confirmed}`); when
  refused also `reason_code: "safe_stop_unconfirmed"` and `detail`.
- Behaviour: pauses new calls on the link; if an actuating (movement) call is
  in flight on any adapter sharing the forward, `safe_stop()` on each (zero
  velocity, then cancel) and refuse unless every stop returned `completed`;
  else tear down ssh + rosbridge, re-resolve, re-establish, re-read the graph
  and `served_identity()`. Never raises; repeatable.

### 2. Preflight clearance along the swept path (`flyto_robotics/path_clearance.py`)

Live 2026-10-06 01:3x: rear 0.334 m at ~168 deg, front 1.445 m, a forward
advance was refused with 0 m travelled because `_motion_preflight` compared
the omnidirectional minimum with the floor. The contract metadata now has
`motion_kind` (`advance`, `retreat`, `rotate`, `planned`) and `SWEPT_PATHS`
maps it to a shape: corridor (`x > 0`, `|y| < floor + FLYTO_ROS2_ROBOT_RADIUS_M`,
nearest return >= floor), footprint (all around >= floor + radius; 0.45 m by
default, stricter than before for rotation), planned (not judged; Nav2 +
inflation escape + in-motion guard). The floor is not lowered. A refusal is
`reason_code: "path_blocked"` with `path_clearance` (`limiting_side`,
`clearance_m`, `threshold_m`, `sectors` ahead/left/behind/right). Without a
sweep the old omnidirectional check applies. Missing bins are not treated as
blocking (an open hall beyond LiDAR range reads None), matching
`braking_envelope.room_to_floor`.

## Why

Rejected: a launchd/systemd unit on the Mac (a second owner of the link the
adapter cannot see or report), `StrictHostKeyChecking=accept-new` (silent
trust on first use), and a readability requirement on the travel sector for
the preflight (would refuse every advance into open space beyond LiDAR range).

## Verified

- `make verify` (ruff, tests, assets, dry runs, contracts, pairing,
  execution grant) on this branch: ruff clean, 1550 passed, 1 skipped.
- New tests: `tests/test_ssh_transport.py` (config refusals incl. non-loopback
  local/remote binds and option-injection hosts, argv, error classification,
  backoff, connect, exit -> backoff -> re-resolve -> reconnect, unresolvable
  host, fatal host key waits for refresh, busy port re-pick, refresh, close,
  linger teardown, call gate), `tests/test_managed_transport_adapter.py`
  (fail-fast typed outcomes, lazy reopen, status, operator refresh,
  refresh during a motion stops it first, unconfirmed stop refuses, paused
  gate, env build implies rosbridge on the forwarded port, build while down,
  rclpy+ssh refused, provider status/process ops, presence watch wake),
  `tests/test_path_clearance.py` (live numbers: advance admitted, retreat
  refused behind; rotate refused on any side < floor + radius; corridor edge
  cases; navigate left to planner; contract-derived kind; adapter refusal
  evidence). All with a fake ssh process; no network.
- `ssh -G` with the generated argv: OpenSSH accepted every option
  (`sessiontype none`, `stricthostkeychecking true`, both `localforward`s on
  127.0.0.1).
- Live, read-only, against the physical robot (no command sent, no motion):
  real `ManagedTunnel` to `ubuntu@flyto-robot.local` connected in 0.76 s,
  `/rosapi/topics` read 99 topics through the forward; killing the ssh child
  -> `reconnecting` -> reconnected in 1.03 s, 99 topics; `refresh()` ->
  reconnected in 0.42 s; `close_all()` left no ssh process. With an empty
  known_hosts file: `failed`, `host_key_unknown`, 1 attempt, no retry, the
  trust-once text in `last_error`.
- `flyto-index task validate`: pass (ruff pass, pytest pass). Through the
  `flyto-index` shim it fails to collect (`No module named flyto_robotics`):
  the shim sets `PYTHONSAFEPATH=1` and its own `PYTHONPATH`, which the pytest
  subprocess inherits. It passed when the indexer was run with the project on
  `PYTHONPATH` (`PYTHONPATH=<indexer>:<worktree> <indexer>/.venv/bin/python -m
  src.cli task validate`). Tooling issue, not this change.
- Unstaged impact (`flyto-index pr-risk .`): 65/high; flags
  `_motion_preflight removed` (false positive: its signature was rewrapped)
  and the test fake's `reconnect` renamed to `reopen` (intended). Symbol
  impact before the change: `build` has 5 references, all in
  `adapter_provider`; `GenericROS2Adapter.reconnect` has one in-repo caller
  plus flyto-cloud `local/adapter_registry._try_reconnect` (see Follow-ups).
- `flyto-index verify --strict`: PASS, 20 pass, 0 warn, 0 fail.

## Not verified

- No robot reboot was induced; the reconnect was exercised by killing the
  local ssh child. A real reboot additionally changes mDNS timing.
- The adapter's own rosbridge session over the managed forward, the presence
  watch end to end, and the operator refresh were run with fakes only, not
  against the robot or the twin. No motion was run (path clearance included).
- `FLYTO_ROS2_ROBOT_RADIUS_M` defaults to 0.1 m (Nav2/burger); it is not read
  from the costmap for the preflight.
- Whether rosbridge cancels an action goal when its client socket closes was
  not checked: a refresh with nothing in flight never has a goal to lose, but
  a goal sent by another job between the drain and the teardown is not
  covered beyond the paused gate.

## Follow-ups

- flyto-cloud (pin bump to 0.6.0): `local/adapter_registry._try_reconnect`
  calls `adapter.reconnect()`, which is now the operator refresh (safe-stops
  any moving call on the shared forward, rebuilds the forward). Switch it to
  `getattr(adapter, "reopen", adapter.reconnect)()` and keep `reconnect()` for
  the operator route. Add the route + Mission Station button calling
  `discover_resource_manifests.reconnect(resource_id)` (or process op
  `reconnect`) and show `discover_resource_manifests.transport_status()`.
- Desktop env: set `FLYTO_ROS2_SSH_HOST=ubuntu@flyto-robot.local`, unset
  `FLYTO_ROSBRIDGE_URL` (or pin `FLYTO_ROS2_SSH_LOCAL_PORTS=rosbridge=19090`
  to keep `ws://127.0.0.1:19090`), and stop starting the tunnel by hand.
