# Fetch-live lane: robot-wide halt hold, site speed ceiling, simulation-only cargo

Owner: claude
Branch: claude/fetch-live-sim
Date: 2026-11-02

## State of the branch

Base `main` 0f283a0. Commits, oldest first:

- bd26172: the first simulated tray bridge (rejected; its files are deleted
  again by 4c5922e, see below).
- c5efcf3, 0de1963: `sensing.cargo_present` (cherry-picked from the deleted
  `claude/fetch-live-cargo-sensor`).
- 7bdf9df, 95d9ff2, 982cc3f, ee24745: first round of review fixes (cargo
  reading drop on bad input, confirmed halt, contract speed) and this
  handoff's first version.
- 2270c4b, 4065417, 16f814d, 4c5922e: second round, the reviewer findings
  below. This file was rewritten in the commit after them (15f882c).
- 2973fc0, 9015cd3: third round (repair round 1), see "What changed in the
  third round". This file and the registry row were updated after them.

`claude/fetch-live-sim-bridge` (the rejected second bridge, 7c5947e and
386f565, never pushed) and its worktree `.claude/worktrees/flyto-robotics-cargo-bridge`
were removed after its useful pieces were brought in: `mobile_cargo.py`,
`arm_reports.py`, the `CargoHandoff` / `CargoBridge` structure and its tests,
the `run.py` report hooks, and the `_advertise_once` extraction. Its
`publish_state` (latched, no simulation gate) was not kept.

## What changed in the second round

### Halt and hold (2270c4b)

- `_ObservationState._halt` always waits `HALT_STILL_WAIT_S` (2 s) for
  fresh odometry that shows the robot still before `completed`, also with no
  tracked goal. `wait_until_stationary` now reports `odometry_messages`; zero
  is `failed`, `reason_code: effect_unknown`, detail "no fresh odometry".
- The hold is `HALT_HOLD` (`_HaltHold`), one per process (an adapter host
  serves one robot), not the goal tables. Every `effect_unknown` halt arms
  it (reason + held goals with a cancel token: rosbridge goal id + action,
  rclpy goal handle). While armed: `_motion_preflight` refuses every motion
  with `effect_unknown`, each commanding connection re-sends zero every
  0.1 s, and a goal whose cancel was not confirmed keeps its handle.
- `RosbridgeROS2Backend.reconnect` and `RclpyROS2Backend.reconnect` still
  forget goals (they belong to the old socket / executor) but call
  `_resume_hold()`: zero, the held goals' cancels re-sent, zero, the zero
  hold restarted. A new backend in the same process does the same on
  connect. A presence-only connection never publishes; it only sees the
  hold.
- Only a halt that sees the robot still releases it
  (`halt_confirmation.released_hold`). There is no other release (no
  operator release exists in this repo). A failed halt re-sends the held
  goals' cancels (`reissued_cancels`) and keeps the hold as it was.
- `RclpyROS2Backend._cancel_for_halt` catches any exception from `cancel()`
  as an unconfirmed cancel (was: escaped `safe_stop`).
- `held_goals()` is replaced by `halt_hold()`. `tests/conftest.py` releases
  the process hold between tests.

### Speed (4065417)

- `FLYTO_ROS2_MAX_LINEAR_SPEED_MPS` is gone (code, `.env.example`, tests).
  The one ceiling above the contract is the site safety profile's
  `max_linear_speed` (`CONSTRAINABLE`, `at_most`), read through the new
  `safety_profile.configured_profile()` (`$FLYTO_SAFETY_PROFILE`, default
  `/etc/flyto/safety-profile.json`), which `contracts._constrained_by_site`
  and the mission gateway now also use. `declared_arguments` folds the
  `speed_mps` maximum through `apply_profile` and caps the default; the
  contract keeps the declared bounds and defaults.
- Preflight refuses motion when the profile exists but cannot be read, or
  its ceiling is below the slowest declared drive speed (0.02 m/s).
- RTF (finding 5): a simulated rosbridge drive's evidence already carries
  `motion_outcome.sim_time` (`wall_s`, `sim_s`, `rtf_mean`, `rtf_min`,
  `sim_slow`). New test pins it for `motion.advance`. No code change; no
  safety behaviour changed. The rclpy transport reports
  `sim_time.measured: false` ("does not read the simulator clock").

### Cargo topic (16f814d)

- `cargo_sensor.configured_topic()` returns only an explicit
  `FLYTO_ROS2_CARGO_TOPIC` (else None). `cargo_sensor.served_topic()` is the
  one resolver: explicit topic, else `/flyto/cargo_present` only when the
  served deployment is `simulation` and the graph carries the marker
  (`/clock` as `rosgraph_msgs/msg/Clock`), else None. It decides the
  rosbridge subscription (hardware without a configured topic does not even
  subscribe), the discovered interface and the read.
- A read with no topic is `refused` with `cargo_reading.reason`
  `cargo_topic_not_configured` and `deployment_mode`. Every cargo result
  carries `deployment_mode`.
- `MIN_PUBLISH_HZ = 2.0` and the volatile-subscription contract are written
  once, in `cargo_sensor.py`'s docstring.

### Bridge (4c5922e)

- One tray: `sim/fetch_return/mobile_cargo.py` `MobileCargoSimulator`
  (`unload()` added); `run.py` imports it and unloads at home (new
  `unload` evidence step; the e2e stage list is now 12).
  `cargo_tray.py`, `rosbridge_link.py`, `arm_place_signal.py` and
  `tests/test_cargo_bridge_sim.py` are deleted.
- `RosbridgeROS2Backend.publish_simulated_state(topic, value)` is the only
  publisher: literal bool, absolute topic, not `/cmd_vel` or a command-watch
  topic, volatile advertise (no latch). Raises `SimulationOnlyError` unless
  `simulation_publish_refusal()` is None (configured `simulation`, marker
  check on) and the graph carries the marker (re-read once before refusing).
  `cargo_bridge.main` returns 2 on the same refusal before opening any
  connection.
- `arm_reports.py` (`flyto.sim-arm-report.v1`): `place` now requires
  `started_at <= at` (taken before `manipulation.place` is commanded); new
  `unload`, `loss`, `mission_end` kinds.
- `CargoHandoff`: parked window is `[place.started_at, now]` (the decision
  time), station tolerance, < 1 cm spread, no unknown pose, no gap > 1.5 s,
  history must reach back to `started_at`. Empties on `unload`, `loss`,
  `mission_end` (a place followed by a mission end never loads), a loss read
  before departure, `max_carry_s` (default 600 s, `--max-carry-s`), and
  `CargoBridge.stop()` (publishes `false` last). A refused publish
  (`SimulationOnlyError`) is reported and not retried as a reconnect.
- `--rate-hz` 2-10 (default `MIN_PUBLISH_HZ`); `CargoBridge.run` raises
  below it. `--arm-reports` is renamed `--reports` (bridge and `run.py`).

## What changed in the third round

### Halt admission (2973fc0)

- `_HaltHold` also carries the halt epoch. `admit(call_id)` records the
  epoch a motion was accepted in (the adapter's `_invoke`, before its
  preflight; escape legs `admit(leg, like=call)`; a backend called
  directly admits on entry, `admit_if_new`). `halting()` starts a new
  epoch and counts a halt in progress; it wraps the backends' `_halt`, the
  adapter's `motion.halt` and `safe_stop`. `motion_refusal(call_id)` is the
  one check: hold armed (the `effect_unknown` text), a halt in progress, or
  a halt started after admission (an unknown parent is never current).
- `sending(call_id)` holds the gate `halting()` takes before a halt reads
  the running goals and yields the refusal. Every goal send runs inside it
  and records the goal as running inside it: rosbridge `invoke`, rosbridge
  `_slow_down` (the preempt; the `#slowN` rename happens inside, so a held
  cancel token is the driving goal), rclpy `invoke`. The adapter re-checks
  right before `backend.invoke` (`_send_motion`) for the first goal, every
  escape leg and the final goal leg. `_motion_preflight` now takes the
  call id and uses the same check.
- rclpy: a goal sent but not yet accepted is kept in `_pending_goals`; a
  halt sees it, waits up to 2 s for the answer and cancels it, or reports
  it unconfirmed with the pending future as its token.
- Kept hold: `HALT_HOLD` writes `flyto.ros2-halt-hold.v1` to
  `FLYTO_ROS2_HALT_HOLD_FILE` (default `<data dir>/halt-hold/<resource id>.json`)
  on arm and removes it on release; a new process reads it and starts held
  (rosbridge tokens kept, rclpy handles not). An unreadable file holds. A
  failed write is reported in the snapshot (`kept_across_restart: false`).
- Escape backoff speed: `min(ESCAPE_BACKOFF_SPEED_MPS, declared speed_mps
  maximum)`, so the site ceiling applies; below the declared minimum the
  escape is refused.
- Rosbridge `reconnect` clears `_topic_types` and `_action_names`; the
  `publish_simulated_state` docstring now says what it does (the graph
  read on this connection, re-read once before refusing). The cargo
  subscription guard drops a redundant presence-only test (the presence
  path keeps only odometry; a test pins that).

### Cargo bridge (9015cd3)

- A place whose `at` or `started_at` is later than the decision time plus
  `MAX_REPORT_SKEW_S` (0.5 s) is retired with `reason:
  report_from_the_future`; `PoseHistory.parked` is False when `start > end`.
- `_apply_lifecycle` retires every place before an `unload`, `loss` or
  `mission_end` report (`_consumed`), which replaces the separate
  mission-end check in `_try_load`.
- `CargoBridge.start()` runs the loop on a thread; `tick()` and `stop()`
  run under one lock and no tick publishes after stop, so `false` is last.
  A join was tried as well and removed: with the lock it changed nothing
  observable (its mutant survived), so the lock is the one mechanism.

## Verified

All in `.claude/worktrees/flyto-robotics-fetch-live`, nothing pushed. Third
round (tip after 9015cd3):

- Tests first. Against the code at 15f882c (a scratch copy, conftest shim
  for the new hold API):
  - halt: the slow-down preempt while held, racing a halt and after a
    completed halt sent `send_action_goal adv-1#slow0` (3 tests); the
    adapter sent `motion.advance` after a hold armed, or a halt completed,
    during the preflight; a motion was admitted while a halt ran; the escape
    sent its waypoint and goal legs after a halt during the backoff; the
    backoff went at 0.05 m/s under a 0.03 m/s ceiling; the hold did not
    survive a new `_HaltHold` (restart) or an unreadable file; rclpy sent a
    goal while held and a halt did not see an unaccepted goal (3 tests);
    rosbridge `invoke` sent a goal while held; `adapter.safe_stop` during
    the preflight did not stop the motion. All failed on behaviour (the
    admission and restart tests also on the missing API).
  - cargo: a reconnect kept the old graph's cargo topic; a future-stamped
    place loaded (`parked` True over `[now+50, now]`); a loss or unload
    after the place still loaded; `stop()` published false while a tick
    was still publishing true. Characterisation tests that passed before
    (they pin existing guards the reviewers found unpinned): retract
    stamped before the place, jaws still holding, hardware with no topic
    subscribes no cargo topic, presence-only subscribes none,
    `cargo_topic()` discovers on demand, presence-only reconnect while held
    publishes nothing, mission end after the place.
- Mutation checks (37 guards, each removed in turn, then a suite of 12 test
  files run, file restored; runner kept in the session scratchpad): first
  run 31 killed, 6 survived (adapter `safe_stop` epoch, unadmitted leg
  parent, rosbridge first-goal refusal, presence-only cargo guard, bridge
  join, tick lock). Tests were added for the first three and the tick
  lock; the presence guard was redundant and removed; the join was
  removed (see above). One mutant first counted as killed was a syntax
  error and was redone. Final: all 37 killed, including the four review
  survivors (`retract.at >= place.at`, `object_held is False`,
  `_resume_hold`'s presence-only skip, the hardware no-subscribe guard)
  and `cargo_topic`'s discover-on-demand.
- `PYTHONPATH=$PWD make verify PYTHON=<repo>/.venv/bin/python`: rc 0, ruff
  clean, 2094 passed, 20 skipped.
- MuJoCo-backed: `tests/test_sim_cargo_bridge.py tests/test_fetch_return_sim.py`
  with the e2e host venv's interpreter (mujoco 3.17.0, read only,
  `PYTHONDONTWRITEBYTECODE=1`): 68 passed.
- `flyto-index verify --strict` in the worktree: Status PASS, 20 pass, 0
  warn, 0 fail.
- Indexer: CLI `context` (slow-down, hold) and `impact` on `_HaltHold`
  before editing. MCP `task(plan)` and `task(validate)` were called this
  round, but the MCP server is pinned to the flyto-indexer root: the plan
  resolved no robotics symbols and `validate` ran flyto-indexer's own
  ruff + pytest (pass, 2738), not this repository's. They are not
  evidence for this repo; `make verify` and the CLI strict verify are.

Earlier rounds' results (2063 passed at 15f882c, the round-2 mutation
counts) are superseded by the above; the round-2 counts were incomplete
(the reviewers found four survivors), which this round closed.

## Not verified

- Nothing ran against the live twin, Gazebo, rosbridge on 19090 or the
  robot. The halt's stillness wait, the zero hold, re-sent cancels and the
  halt epoch are tested with fake transports only.
- Release can free an orphaned goal (residual risk, not fixed). A halt
  that sees the robot still releases the hold even when held goals from an
  earlier connection never had a confirmed cancel. On rosbridge a
  `cancel_action_goal` re-sent for a goal another (dead) socket sent is
  very likely a no-op; on rclpy a handle from a destroyed node likewise.
  If Nav2 pauses (a wait or recovery, or the zero hold briefly winning), a
  halt can see stillness, release, and the orphaned goal can drive again.
  Closing it needs a confirmed end of those goals (Nav2's action status
  topic, or a cancel-all through `<action>/_action/cancel_goal`), which
  was not built or tested here.
- The kept hold file: written and read in tests only; the default path's
  permissions on a Desktop install, and two adapter hosts for one robot on
  one computer (they would share the file by resource id), were not tried.
  A restarted host only re-sends rosbridge cancels (rclpy handles do not
  survive a process).
- The admission gate holds a lock while a goal is sent; a rosbridge send
  that blocks would delay a halt's start by that long. Not measured.
- Nav2 `motion.navigate` is not bounded by the site's `max_linear_speed`;
  only `motion.advance` / `motion.retreat` (and now the escape backoff)
  are. Nav2's own controller limits apply there.
- Every halt still waits up to 2 s for odometry; a robot without odometry
  can never complete a halt and stays held (and, now, stays held across a
  restart).
- The 52 s advance, Cloud's schema-hash/approval question, Cloud reading
  `effect_unknown` and `cargo_topic_not_configured`, and the live bridge
  sequence in `sim/fetch_return/README.md`: as in round 2, not checked.

## Follow-ups

- Decide whether an operator release is wanted; today only a halt seen
  still releases the hold (now also across a restart).
- Close the orphaned-goal release risk above (confirmed end of held goals
  before release).
- Re-run the 2.0 m twin advance and read `sim_time` before changing any
  default speed.
