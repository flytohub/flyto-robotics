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
  below. This file was rewritten in the commit after them.

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

## Verified

All in `.claude/worktrees/flyto-robotics-fetch-live`, nothing pushed.

- Tests first. Against the code before each change:
  - halt: 8 of the 9 new `tests/test_halt_hold.py` tests failed, 6 on
    behaviour (`completed` where `failed` was expected, `KeyError:
    'stillness'`, the rclpy cancel exception escaping `safe_stop`) and 2 on
    `halt_hold` missing; the ninth (a reconnect without a hold sends
    nothing) passed.
  - speed: 7 tests failed on behaviour (bound not tightened by the profile,
    no refusal for an unreadable profile, the retired variable still
    tightening).
  - cargo: 9 tests failed (hardware and unmarked simulation still declared
    the default topic; `served_topic` / `MIN_PUBLISH_HZ` missing).
  - bridge: against the cherry-picked rejected bridge (with a shim adding
    `started_at` and the three new report builders so collection works),
    22 tests failed, including "robot arrived during the place", "robot
    left before the handoff was decided", "creeping inside the tolerance",
    every unload path, and every simulation-gate test. The two rate tests
    hung there (the old `run` accepted 0.5 Hz and looped).
  - RTF: the new evidence test passed at once (characterisation; the
    evidence already existed).
- Mutation checks (guard removed, test suite run, file restored):
  halt 11/11 killed (stillness only with goals, hold not armed, preflight
  ignoring the hold, rosbridge and rclpy reconnect not resuming, reconnect
  clearing the hold, rclpy cancel unguarded, no-odometry not distinguished,
  zero hold not sending, held cancels not re-sent, release before
  stillness); speed 6/6 (preflight check removed, ceiling-below-minimum
  allowed, maximum not folded, default not folded, fold widening, unreadable
  profile ignored); cargo 4/5, the survivor was a duplicate resolver in
  `discover`, now the same `_served_cargo_topic`; bridge 29/31 at first
  (survivors: sample-gap check with no test, and two overlapping mission-end
  guards), then a silent-odometry test was added and the redundant guard
  removed, 2/2; `run.py` hooks 4/4 with the MuJoCo interpreter (no unload at
  home, no unload report, no mission_end report, `started_at` taken after
  the place).
- `PYTHONPATH=$PWD make verify PYTHON=<repo>/.venv/bin/python`: rc 0, ruff
  clean, 2063 passed, 20 skipped (the robotics venv has no `mujoco`).
- MuJoCo-backed tests with an interpreter that has `mujoco` 3.17.0 (the
  e2e host venv, read only, with this repo, `tests/` and
  flyto-modules-lerobot `src` on `PYTHONPATH`):
  `tests/test_sim_cargo_bridge.py tests/test_fetch_return_sim.py` 60 passed.
- `flyto-index verify --strict`: Status PASS, 20 pass, 0 warn, 0 fail.
- Indexer exploration via the CLI (`context`, `impact` on `_halt`,
  `declared_arguments`, `configured_topic`, `reconnect`, `held_goals`). The
  MCP server is pinned to another root, so MCP `task(plan)` /
  `task(validate)` were not run; `make verify` ran ruff and pytest.

## Not verified

- Nothing ran against the live twin, Gazebo, rosbridge on 19090 or the
  robot. The halt's stillness wait, the zero hold and the re-sent cancels
  are untested against Nav2; whether rosbridge honours a
  `cancel_action_goal` for a goal sent on an earlier socket is unknown (the
  re-send is best effort; stillness decides).
- The hold is per process. A host process restart forgets it; nothing
  persists it. A presence-only watch never publishes zero for it.
- Every halt now waits up to 2 s for odometry; a robot without odometry
  (or a stopped odometry stream) can never complete a halt and stays held.
  Callers that halt an idle robot (e.g. `GenericROS2Adapter.reconnect` with
  an actuating call in flight) now take that wait.
- The 52 s advance: its evidence was not available here; read
  `motion_outcome.sim_time.rtf_*` and `braking.speed_changes` of that call.
- Cloud: schema hash change of `motion.advance` / `motion.retreat`
  (default in schema; maximum under a site ceiling) and whether it needs a
  new approval; whether `effect_unknown` holds the task end to end;
  whether Cloud reads `cargo_reading.reason`
  `cargo_topic_not_configured`.
- The live bridge sequence in `sim/fetch_return/README.md` is unexercised;
  the station is given in the odometry frame.

## Follow-ups

- Decide whether the halt hold must survive a host restart (persist it with
  the host's state) and whether an operator release is wanted; today only a
  halt seen still releases it.
- Re-run the 2.0 m twin advance and read `sim_time` before changing any
  default speed.
