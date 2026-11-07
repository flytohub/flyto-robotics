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
  third round". This file and the registry row were updated after them
  (ab69ad9).
- a279e61: fourth round (repair round 2), see "What changed in the fourth
  round". This file and the registry row were updated after it (b2da84e).
- 7babfd7: fifth round (repair round 3, the review of b2da84e), see "What
  changed in the fifth round". This file and the registry row were updated
  in the commit after it.
- Sixth round (repair of the review of 623a8d6), see "What changed in the
  sixth round": the code commit and the handoff commit after it.

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
  also uses. Correction (round 5): the mission gateway did not; it resolved
  the path itself. Since round 5 both go through
  `safety_profile.configured_profile_path()`. `declared_arguments` folds the
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

## What changed in the fourth round (a279e61)

### A halt never fails open on a transport error

- `_ObservationState._zero_published` catches any exception (was only
  `RuntimeError`): an `OSError` or a closed websocket is a stop not
  published, so `zero_velocity_published: false` and the halt ends
  `effect_unknown` "standard cmd_vel stop unavailable" with the hold armed,
  also when the robot is then seen still. The zero-hold thread uses the
  same call, so it no longer dies on such an error.
- `RosbridgeROS2Backend._cancel_for_halt` catches any exception from
  `cancel()` (was only `RuntimeError`), like the rclpy one: the goal is
  `unconfirmed` "cancel not sent: <type>: <message>" and the halt goes on.
- `_halt` wraps the whole halt: anything that still raises inside it
  (`_halt_raised`) arms the hold with the running goals' tokens, starts the
  zero hold and returns `failed`, `reason_code: effect_unknown`,
  `halt_confirmation.error`. It never raises out of `safe_stop`.
- Round-3 statement corrected: the rosbridge halt did **not** guard every
  cancel error before this round (only `RuntimeError`), and its guard had
  no test.

### The hold is released only on a confirmed end of the held goals

- After a halt sees the robot still, every goal in the hold that this halt
  did not itself cancel (held from an earlier connection or process) must
  be confirmed ended by its action server (`_held_goal_end`), else the halt
  is `effect_unknown`, the hold stays, and the evidence lists
  `held_goals_unconfirmed`; confirmed goals are listed in
  `held_goals_ended`.
- rosbridge: `call_service` `<action>/_action/cancel_goal`
  (`action_msgs/srv/CancelGoal`, zero goal id and stamp = cancel all goals
  on that server), waiting `HELD_GOAL_END_WAIT_S` (2 s). Confirmed only on
  `return_code` 0 with an empty `goals_canceling`. A token with no action
  name is never confirmed.
- rclpy: the kept handle's `cancel_goal_async()`; confirmed only on
  `return_code` 2 (unknown goal) or 3 (terminated). No kept handle (a
  restarted host) is never confirmed.

### A simulation with the marker check disabled is never served as real

- `_unmarked_simulation_refusal()` is the one reason, used by
  `served_identity` (unchanged behaviour) and now by `_deployment_mismatch`,
  so motion preflight and `sensing.cargo_present` refuse a configured
  simulation whose `FLYTO_ROS2_SIM_MARKER_TOPIC` is empty.
- `_served_deployment_mode()` returns `unconfirmed` (was `real`) for that
  configuration, so a backend read never labels it real.
- `cargo_sensor.refused_result` is the one refused-read shape;
  `no_topic_result` uses it. A deployment mismatch refusal now carries
  `cargo_reading` with `deployment_mode: unconfirmed`, `reason:
  deployment_not_confirmed`; a disconnected rosbridge read carries
  `reason: transport_disconnected` and the served mode.

### Settle span pinned

- `arm_reports.settled_run`'s `SETTLE_S` simulated-time span had no test;
  one is added (no code change).

## What changed in the fifth round

### Stillness is the full twist AND the pose, over a minimum duration

- `wait_until_stationary` (one implementation, used by the halt and the
  settle waits) counts a run of odometry messages received after the call as
  still only when every one has (a) all six twist axes present, finite and
  within `STILL_LINEAR_MPS` / `STILL_ANGULAR_RADPS`, and (b) a pose within
  `STILL_POSE_DELTA_M` / `STILL_YAW_DELTA_RAD` of the run's first message.
  The run must have `STILL_SAMPLES` (3) messages AND span
  `STILL_MIN_SECONDS` (0.5 s) of their receive times. A message that fails
  either check restarts the run.
- `odometry_twist_axes()` parses both transports' twist axis by axis. A
  missing or non-numeric axis is `None` (unknown), never 0. A message with no
  twist at all is never still. That is a liveness change: a driver that
  publishes no twist (the old pose-only fallback) can no longer complete a
  halt, so it stays held. `_velocity` (linear x, angular z, for the braking
  guard and stall tracks) is now `None` unless both of those axes are present.
  It used to default a missing axis to 0.
- Evidence: `stillness` adds `still_seconds`, `min_still_seconds` and
  `unknown_twist_messages`.
- The duration does not cover a long pause such as Nav2's Wait recovery
  (5 s default). That case is covered by the goal cancels and the held-goal
  end check, not by stillness.
  Correction (round 6): only for goals this process knows; see "Residual
  risk, stated plainly" in the sixth round.

### A reconnect carries every running goal into the hold

- `RosbridgeROS2Backend.reconnect` and `RclpyROS2Backend.reconnect` call
  `_hold_running_goals()` before they forget the goal tables. Every goal
  still running is armed into `HALT_HOLD` with its cancel token. For
  rosbridge that is a goal in `_active_actions` with no terminal status, as
  (goal id, action). For rclpy it is a pending goal or an accepted one whose
  result has not arrived, as the handle or the pending future. The resume
  then re-sends its cancel. A later halt completes only when it sees the
  robot still and `_held_goal_end` confirms the goal ended. Motion is refused
  in the meantime. Operational cost: any reconnect while a goal runs now
  needs a confirmed halt before motion is allowed again.

### Every halt that cannot run holds

- `GenericROS2Adapter.invoke("motion.halt")` goes to `_invoke_halt` and never
  through `_invoke`. A presence connection, a link or forward that is down,
  an operator refresh in progress, `motion.halt` missing from the graph
  (after one re-read) and `deadline_seconds <= 0` each end in
  `_halt_not_run`. The same holds for `adapter.safe_stop()` with the
  transport down. `_halt_not_run` starts a new halt epoch
  (`HALT_HOLD.halting()`), arms the hold with the running goals' tokens,
  publishes zero once and keeps holding it where the connection can
  (`_ObservationState.hold_unrun_halt`, which `_halt_raised` now uses too; a
  presence connection never publishes), and returns `failed`,
  `reason_code: effect_unknown`. A link-down halt also carries
  `transport_unavailable: true` and the transport status.
- A halt's arguments are no longer validated. A stop with arguments it does
  not take still runs, where it used to be refused.

### Hold, resume, deployment label: guards pinned

- Tests now pin the kept-hold `OSError` arm (a directory, and a
  `PermissionError` on read), the zero-hold restart in `_resume_hold` on a
  reconnect while held, and `_served_deployment_mode()` returning
  `unconfirmed` (the backend's own cargo read).
- `RclpyROS2Backend.__init__` calls `_resume_hold()`, so an rclpy host that
  starts held sends zero and holds it at once. Before this, that happened
  only on its first reconnect.
- `served_identity` answers `_served_deployment_mode()` (one owner) instead of
  re-deriving it.

### Cargo

- `CargoBridge` samples odometry through the backend's new `latest_odometry()`
  (`pose`, `sequence`, `age_s`). It records each message once, at
  `now - age_s` (its receive time on the bridge's wall clock), so odometry
  silence is a gap however often the bridge ticks. A read that fails or
  finds no odometry records an unknown pose at `now`. `PoseHistory` keeps
  its samples in time order. A place's decision time is fixed when the place
  is first ready to decide, and the decision waits for a message received at
  or after it. The bridge no longer narrows
  `FLYTO_ROS2_OBSERVATION_MAX_AGE_SECONDS` (it no longer reads
  `held_observation`).
- `cargo_sensor.served_topic` serves `/flyto/cargo_present` only on a marked
  simulation, also when `FLYTO_ROS2_CARGO_TOPIC` names it explicitly. A
  hardware deployment configured on it declares nothing, subscribes to
  nothing, and a read is refused with `cargo_topic_not_configured`.
- One resolver: `cargo_sensor.simulated_tray_topic()` (the `served_topic` of a
  marked simulation) is used by `cargo_bridge.bridge_topic()` and by
  `publish_simulated_state`. That publisher now refuses any topic but the tray
  topic (it is no longer a general Bool publisher), and still refuses command
  topics first. The rosbridge cargo subscription goes through `served_topic`
  as well.
- Pinned: a retract to any pose but `ready` never loads;
  `SETTLE_MIN_READINGS`; a failed odometry read never becomes a pose;
  the backend's `latest_odometry()` (sequence only grows with a message,
  `age_s` grows while silent, None after a drop).
- `run.py`'s report trail and the unload at home are tested under
  `make verify` with a stubbed arm (`tests/test_fetch_return_report_trail.py`):
  the `unload` evidence step, and the `unload` / `loss` / `mission_end`
  report order.
- `sim/fetch_return/README.md`: step 3 of the live sequence now runs
  `run.py --navigation gazebo --station-distance-m 0.4`, and the README says
  how the bridge's odometry-frame station relates to the parked pose. It also
  says the sequence has not been run.

### Site profile path

- `safety_profile.configured_profile_path()` is the one resolver.
  `configured_profile()` and `MissionGateway.safety_profile_path` both use it.
  The function-local `os` / `Path` imports in `safety_profile.py` moved to
  the module imports.

## What changed in the sixth round

The review of 623a8d6 (two reviewers, neither accepted) found a held goal
reported ended while its server still ran it, a reconnect race that lost a
goal, a halt that could not run without cancelling the running goals, no
operator release, and an rclpy reconnect that could wedge the hold. This
round fixes those.

### A held goal's end is confirmed only from its goal status

- `_ObservationState._held_goal_end` is now one implementation for both
  backends. It reads `<action>/_action/status` (action_msgs
  `GoalStatusArray`, reliable, transient-local) until no goal on that
  action server is ACCEPTED, EXECUTING or CANCELING. Only statuses 4, 5
  and 6 (`GOAL_ENDED_STATUSES`) count as ended. A malformed array, an
  entry with no status, a text or bool status reads as UNKNOWN (0), which
  is live.
- A cancel response never counts as an end. Per rcl_action a CANCELING
  goal is not cancelable, so a cancel-all answers "no error, nothing
  canceling" while the server still drives that goal; round 4's check
  released the hold on that answer. `CANCEL_GOAL_ENDED_CODES` and
  `CANCEL_ERROR_NONE` are removed.
- The backends only supply `_goal_statuses_until`: rosbridge subscribes
  over its socket, rclpy creates a temporary subscription on its current
  node.
- The rclpy `cancel()` confirms a cancel only when the goal's result future
  completes within `CANCEL_CONFIRM_WAIT_S` (3 s, shared with rosbridge). A
  non-empty `goals_canceling` is no longer trusted.

### Cancel tokens are (goal id, action name) on both backends

- rclpy records `_goal_actions` when a goal is sent, so its token survives a
  node rebuild and a host restart (through the kept hold file). A held goal
  is re-cancelled through its action server's own cancel service with the
  zero goal id (`_ObservationState._recancel` -> `_cancel_all_goals`:
  an unawaited `call_service` on rosbridge, a cached `CancelGoal` client on
  rclpy). `_recancel_held` lists only the cancels actually sent.
- rclpy reconnect liveness: `disconnect()` destroys the node, and the old
  code kept goal handles and pending futures that belonged to it, so no
  later halt could confirm their end. Held goals are now confirmed by
  action name on the current node, and a test shows a halt releasing the
  hold after a disconnect followed by a reconnect.

### Reconnect race closed

- Both reconnects hold the running goals and clear their goal tables
  inside `HALT_HOLD.forgetting()`, which takes the same lock as
  `sending()`. A goal is either sent and recorded before the hold reads
  the goals (and is held) or sent afterwards on the new connection.

### A halt that cannot run cancels the running goals

- `hold_unrun_halt` is one module-level function, used by `_halt_not_run`
  and `_halt_raised`. It arms the hold with the running goals' tokens.
  Where the connection commands, it also sends the cancel of every running
  and held goal, then publishes zero and keeps it held: zero alone is not
  a stop while a goal's controller publishes faster than the hold re-sends
  zero. The weaker fallback that armed the hold with no goals is removed.
  It never raises.

### Operator release

- `GenericROS2Adapter.release_hold(operator=..., reason=...)`
  (`_ObservationState.release_hold`). It needs a non-empty operator and
  reason, is refused on a presence connection, publishes zero, re-sends the
  held goals' cancels, and needs fresh odometry showing the robot still.
  `_HaltHold.operator_release` then appends a JSON line (who, why, what was
  held, the stillness evidence) to `<hold file stem>.releases.jsonl` and
  logs it before it clears the hold. If that record cannot be written,
  nothing is released.
- It is a Python entry point only. No capability, Cloud route or console
  button calls it yet.
- Deleting the kept hold file while the host runs releases nothing: the
  hold is kept in memory and the file is written again on the next arm.
  The `_HaltHold` docstring and `.env.example` say so. Without the operator
  release, the only way out is: stop the host, delete the file, start the
  host.

### Residual risk, stated plainly

- A halt cancels only goals this process knows: the ones it sent, and the
  ones in the hold. It does not cancel a goal it never knew: one sent to
  the same action server by another client, or by a host process that
  crashed before the hold was armed (running goals reach the kept file
  only when something arms the hold). If such a goal pauses the robot for
  at least `STILL_MIN_SECONDS` (0.5 s), as Nav2's Wait recovery does
  (5 s), a halt completes and releases nothing it does not know about; the
  goal can then drive again. The fifth round's "a long pause is covered by
  the goal cancels" was true only for goals this connection knows. The
  comment at `STILL_MIN_SECONDS` now says so. Not done: a cancel-all plus a
  status check on every known navigation action server in every halt.

### Design choices the owner should know

- A held goal is confirmed ended by "no live goal on that action server".
  Another client's goal on the same server therefore also keeps the hold.
- A held goal is re-cancelled through cancel-all on its action server,
  which also cancels other clients' goals there.
- An action server that has never published a status (restarted, no goal
  since) keeps the hold until an operator releases it.

### Cargo marker rule

- `cargo_sensor.served_topic(..., marker_check=False)` returns None for
  every deployment. The adapter passes
  `bool(FLYTO_ROS2_SIM_MARKER_TOPIC)`, so with the simulator marker check
  disabled no deployment reads any cargo topic, a custom one included.
  Before, a hardware adapter with the marker check off, pointed at the
  twin's endpoint with the same custom topic, would read the simulated
  tray labelled `real`. The refusal detail and `.env.example` say so.

### Naming

- `motion_outcome`'s stall thresholds are renamed `STALL_LINEAR_MPS` /
  `STALL_ANGULAR_RADPS`, with a comment on why stall evidence (linear x,
  angular z, looser) differs from halt stillness (every axis and the pose).

## Verified

All in `.claude/worktrees/flyto-robotics-fetch-live`, nothing pushed.

Sixth round:

- Tests first. The changed test files run against 623a8d6 (a scratch copy
  of that tip with the new test files) give 21 failed and 28 errors of
  299. Every `tests/test_halt_hold.py` error is its fixture patching the
  new `CANCEL_CONFIRM_WAIT_S`; with only that patch made tolerant (in the
  scratch copy), 14 of its then 28 tests failed. On behaviour: a held goal
  its server is still CANCELING released (`completed`); held goals
  released on a cancel answer, or never re-cancelled through
  `<action>/_action/cancel_goal` (rosbridge reconnect, rclpy statuses
  1-5); a goal recorded during a rosbridge or an rclpy reconnect not held;
  an rclpy pending send held as a future, not a token; a restarted rclpy
  host unable to confirm a kept goal (`failed`); a simulation and a
  hardware robot with the marker check off read cargo. Only on a missing
  API: `goal_statuses` / `live_goal_statuses`, `served_topic(marker_check=)`,
  `_goal_actions`, `release_hold`, and the unrun halt's `reissued_cancels`
  key (that test fails on the key before it reaches its cancel checks).
  The two tests added after the mutation run: a held goal with no action
  name reported among the re-sent cancels fails on behaviour; the release
  whose zero cannot be published fails on the missing `release_hold`.
  Passed before (they
  pin existing guards the reviewers found unpinned): deleting the kept
  file releases nothing, a halt that raised and a halt with the link gone
  keep the running goals, a bool twist axis is not still, a NaN or inf
  odometry age is an unknown pose.
- Mutation checks: 22 mutants of this round's guards (runner
  `scratchpad/r6/mutate6.py`, each in a fresh copy of the worktree,
  targeted suites of 7 test files). 19 killed, 3 survived: the operator
  and reason check (the test asked without odometry, so stillness refused
  first), the release's zero-publish check, and `_recancel` treating a
  token with no action name as sent. Tests were added for each; all three
  re-run with the full suite: killed. Final: 22/22 killed. The round-5
  review's survivors (CANCELING counted as ended, the unrun halt's goal
  tokens, the bool axis, the NaN cargo age) are among the 22.
- `PYTHONPATH=$PWD make verify PYTHON=<repo>/.venv/bin/python` on the final
  tree, twice in a row: rc 0 both times, ruff clean, 2204 passed, 20
  skipped each. (A run before the last three tests were added: rc 0,
  2202 passed, 20 skipped.)
- `flyto-index verify --strict` in the worktree: rc 0, Status PASS, 20
  pass, 0 warn, 0 fail.
- Indexer: CLI `context` on the held-goal end / unrun-halt path before
  this round's last edits (the index answers from committed code; the
  worktree changes were uncommitted). MCP `task(plan)` / `task(validate)`
  not used (the MCP server is pinned to another root).
- This repo has no project-memory lint target; none was run.
- Not run: anything against Gazebo, the live twin, rosbridge on 19090 or
  the robot; the MuJoCo-backed suites. The rcl semantics (a CANCELING goal
  is not cancelable, cancel-all answers) come from the ros2/rcl humble
  sources via the reviewers, not from a live server. The rosbridge status
  subscription sends a `qos` field; whether rosbridge honours it, or picks
  transient-local itself from the publisher, is not checked live. Whether
  Nav2 publishes an empty or all-terminal status array once its goals end
  or expire is likewise taken from the sources, not observed.

Fifth round (7babfd7):

- Tests first: `tests/test_halt_fail_closed.py` run against the code at
  b2da84e (a scratch `git archive` copy) fails 31 tests on behaviour: the
  three moving-pose halts (zero twist, `linear.y`, empty vectors) and the
  per-axis twist cases `completed`; the duration and creep cases reported
  still; reconnect goals not held; the no-time halt `timeout`, the
  link-down, presence and refresh halts `refused` and the link-down
  `safe_stop` `transport_unavailable`; the rclpy backend that starts held
  sent no zero. The kept-hold `OSError` tests, the `_resume_hold` zero
  restart and `tests/test_fetch_return_report_trail.py` passed before:
  they pin existing guards the reviewers found unpinned.
- The reviewers' probes (`scratchpad/probes/test_probe_still.py`,
  `test_probe_orphan.py`, `test_probe_halt_refused.py`) pass against
  7babfd7: the three moving-pose halts end `failed` (their twists lack
  axes, so `unknown_twist_messages` equals `odometry_messages`; the
  full-twist zero case with the pose moving is in the new tests); the
  orphaned goal is held at the reconnect, its cancel re-sent and a
  cancel-all sent before the halt completes; the three early-return halts
  end `failed` `effect_unknown` with the hold armed.
- Mutation checks, 41 mutants of this round's guards (runner and logs in
  the session scratchpad `fetchlive_r5_mut_k3v9/`, a unique directory;
  each in a fresh copy, targeted suites of 3-5 test files, harness checked
  first with the unmutated copy passing). First run: 37 killed, 4
  survived: angular x/y ignored, `stationary` not requiring the duration
  when the cap ends the wait, a twist with neither vector read as zeros,
  and a read error turned into a made-up pose (that mutant's pose was off
  the station, so it could never load; redone with a pose at the station).
  Tests were added for the first three. Rerun of the four: all killed.
  Final: 41/41 killed, including every survivor in the review (hold-file
  `OSError`, `_resume_hold`'s `_hold_zero`, `unconfirmed` label, retract
  pose, `SETTLE_MIN_READINGS`, read error as unknown, `run.py`'s unload
  evidence and its unload / mission_end report appends) and the gateway's
  profile path, `bridge_topic`, `served_identity`, `served_topic`'s
  explicit-topic check and `publish_simulated_state`'s topic restriction.
  Earlier rounds' guards were not re-mutated.
- `PYTHONPATH=$PWD make verify PYTHON=<repo>/.venv/bin/python`: rc 0, ruff
  clean, 2170 passed, 20 skipped (one run). Note (round 6): the round-5
  reviewers ran it twice more at 623a8d6 with a clean worktree, rc 0 both
  times, ruff clean, 2170 passed, 20 skipped; the two green runs the
  verification rule asks for are theirs, not this lane's.
- `flyto-index verify --strict` in the worktree: rc 0, Status PASS, 20
  pass, 0 warn, 0 fail.
- Indexer: CLI `impact` on `_served_deployment_mode` and `context` on the
  stillness/hold/reconnect path before the last edits. MCP `task(plan)` /
  `task(validate)` were not used (the MCP server is pinned to another root).
- Not run this round: the MuJoCo-backed `tests/test_fetch_return_sim.py`
  and the skipped MuJoCo test in `tests/test_sim_cargo_bridge.py`. The
  `run.py` report trail is now pinned under `make verify` by the stubbed
  arm instead.

Fourth round (a279e61):

- Tests first, failing on behaviour before the fix: a rosbridge cancel send
  raising `OSError` (raised out of `safe_stop`, no hold); a `/cmd_vel` zero
  publish raising `OSError` (raised out); a halt whose stillness wait
  raises (raised out); the zero hold after a failed publish (thread died);
  a halt seen still after a reconnect released a held goal with no
  confirmed end (completed); rclpy held goals with return codes 0/1 and no
  kept handle released, and 2/3 had no `held_goals_ended`; a simulation
  with the marker check disabled read cargo `completed` labelled `real`
  and passed the motion preflight's deployment check; mismatch and
  disconnected refusals had no `cargo_reading`. The rosbridge cancel test
  with `RuntimeError` and the settle-span test passed before (they pin
  existing guards the reviewers found unpinned).
- The reviewers' probes (`scratchpad/probe/test_probe_halt.py`,
  `test_probe_cargo.py`) now give `failed effect_unknown ... cancel not
  confirmed for nav-1`, `failed effect_unknown ... standard cmd_vel stop
  unavailable`, and a refused cargo read with `deployment_mode:
  unconfirmed`.
- Mutation checks, this round's guards (17, each changed in turn, 12 test
  files run, file restored): all 17 killed. They are the rosbridge cancel
  guard removed, narrowed to `RuntimeError`, and not catching
  `RuntimeError`; the `if not stopped` refusal; the `_halt` wrapper; the
  arm in `_halt_raised`; `_zero_published` narrowed to `RuntimeError`; the
  held-goal end check; a held-goal end error taken as ended; rosbridge
  ignoring `goals_canceling`; rclpy taking any code as ended; rclpy with no
  handle taken as ended; `_deployment_mismatch` not refusing an unmarked
  simulation; unmarked simulation labelled `real`; the mismatch and
  disconnected cargo refusals without evidence; the `SETTLE_S` span check.
  The round-3 guards were not re-mutated this round.
  Correction (review round 5): "all 17 killed" was true of that set only.
  The reviewers found 6 more survivors among this change's guards (the
  kept-hold `OSError` arm, `_resume_hold`'s zero hold, the `unconfirmed`
  label, and three cargo-bridge guards). They are pinned in round 5
  (above).
- `PYTHONPATH=$PWD make verify PYTHON=<repo>/.venv/bin/python`: run twice,
  rc 0 both times, ruff clean, 2109 passed, 20 skipped.
- `flyto-index verify --strict` in the worktree: rc 0, 20 pass, 0 warn,
  0 fail.
- Indexer: CLI `context` on the halt guards and the cargo/deployment path
  before editing (the MCP server is pinned to the flyto-indexer root, so
  MCP `task(plan)`/`task(validate)` were not used as evidence).
- Not re-run this round: the MuJoCo-backed bridge/fetch-return tests with
  the e2e host interpreter (no `arm_reports` code changed; the new settle
  test runs in the repo venv).

Third round (tip after 9015cd3):

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
  error and was redone. Final: all 37 of the guards in that set killed,
  including the four review survivors (`retract.at >= place.at`,
  `object_held is False`, `_resume_hold`'s presence-only skip, the
  hardware no-subscribe guard) and `cargo_topic`'s discover-on-demand.
  Correction (review round 4): that set was not complete. Three guards of
  this change outside it survived the reviewers' mutation: the rosbridge
  `_cancel_for_halt` except, `_halt_in_epoch`'s `if not stopped`, and
  `settled_run`'s `SETTLE_S` span. They are pinned in round 4 (above).
  Round 5's review found more of this set's guards unpinned (the kept-hold
  `OSError` arm among them), so "all 37 killed" held only for the 37
  mutated; see round 5.
- `PYTHONPATH=$PWD make verify PYTHON=<repo>/.venv/bin/python`: rc 0, ruff
  clean, 2094 passed, 20 skipped (one run; see the flake below).
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

- Round 5 liveness costs, not tried on a robot: a driver whose odometry
  carries no twist (or omits an axis) can never complete a halt and stays
  held; every completed halt now takes at least 0.5 s
  (`STILL_MIN_SECONDS`) of still odometry; any reconnect while a goal runs
  now needs a confirmed halt (cancel-all answered) before motion again.
  `STILL_MIN_SECONDS` does not cover a pause longer than 0.5 s such as
  Nav2's Wait recovery; that case relies on the cancels and the held-goal
  end check.
  (Round 6: a goal this process never knew is not covered at all.)
- The cargo bridge's receive-time sampling assumes the bridge and the
  backend share one host clock (`time.monotonic` age converted at the
  bridge's wall clock); tested with fakes only.

- Nothing ran against the live twin, Gazebo, rosbridge on 19090 or the
  robot. The halt's stillness wait, the zero hold, re-sent cancels and the
  halt epoch are tested with fake transports only.
- The confirmed end of held goals (round 4) is tested with fakes only. Not
  checked on a real rosbridge: that `call_service` reaches the hidden
  `/navigate_to_pose/_action/cancel_goal` with `type` given and the zero
  `uuid` as a list of 16 ints, and that Nav2 answers a cancel-all with no
  goals as `return_code` 0 and an empty `goals_canceling` (if it answers
  otherwise the hold is never released by a halt). The cancel-all also
  cancels goals another client sent to that action server; that is
  intended for a halt, but not discussed with the owner.
- Liveness cost (round 4), corrected in round 6: a held goal with no
  confirmable end (a token with no action name, an action server that
  never publishes its goal status) keeps the robot held until a halt can
  confirm it or an operator releases it (`release_hold`, round 6). The
  round-4 text said an operator could remove the kept hold file. That was
  wrong: deleting the file while the host runs releases nothing (the hold
  is kept in memory and the file is written again). Without the operator
  release, the procedure is: stop the host, delete the file, start the
  host. A restarted rclpy host no longer has this problem (round 6: its
  held goals carry their action name).
- `make verify` flake (reported by the reviewers, not reproduced here):
  `tests/test_managed_transport_adapter.py::test_status_reports_state_since_attempts_and_last_error`
  timed out once ("condition not reached" after 3 s) in one of the
  reviewers' two full runs and passed five times alone. This branch does
  not touch that file or the managed transport. Both round-4 full runs
  passed. Not checked on `main`; not fixed.
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

- Operator release: added in round 6 (`release_hold`). Not yet wired to
  a Cloud or console action; the owner decides who may call it and where.
- Check the cancel-all confirmation against the live twin's rosbridge and
  Nav2 before relying on a release after a reconnect.
- Look at the managed-transport timing flake on `main`.
- Re-run the 2.0 m twin advance and read `sim_time` before changing any
  default speed.
