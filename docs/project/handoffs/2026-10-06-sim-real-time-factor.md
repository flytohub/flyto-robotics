# A motion result says how fast the simulator ran

Owner: claude
Branch: claude/sim-rtf (off origin/main 5bb4ca1, 0.6.4)
Date: 2026-10-06

## What changed (0.6.5, read-only, no motion change)

- `flyto_robotics/sim_time.py` (new, no ROS import):
  - `ClockRecord` keeps `/clock` samples as (simulated s, wall monotonic s).
    It drops samples whose wall time goes backwards, restarts on a simulator
    reset and flags `clock_reset_seen`, and halves itself past 2048 samples.
  - `summary()` gives `rtf_mean` over the motion, `rtf_min` over windows of at
    least `WINDOW_S` (1 s), `wall_s`, `sim_s`, `samples`, `windows`, and
    `sim_slow` when `rtf_min < SIM_SLOW_RTF` (0.5, or `FLYTO_ROS2_SIM_SLOW_RTF`
    in (0, 1]), with `sim_slow_threshold` and `basis`.
  - `combine()` merges several legs: clocks summed, the slowest window kept.
    `not_applicable()` gives the record for a real deployment.
- `flyto_robotics/motion_outcome.py`: `MotionTrack.clock` / `sim_time_note`;
  `summarize()` always adds `sim_time`.
- `flyto_robotics/generic_ros2_adapter.py`:
  - `_served_deployment_mode()` returns `simulation` only when the configured
    mode is simulation and the marker check is on, i.e. what `served_identity`
    answers once the graph confirms it. Motion is refused on a mismatch
    before a track starts.
  - `_begin_track` gives a simulated motion a `ClockRecord`. On a transport
    with `READS_SIM_CLOCK = False` (rclpy) it records a note instead.
  - Rosbridge subscribes the marker topic as `rosgraph_msgs/msg/Clock`,
    best effort, throttled to `CLOCK_THROTTLE_MS` (100 ms), only in a
    simulation deployment. `_track_clock` feeds every running track.
  - The escape's `legs[]` carry each leg's `sim_time`, and
    `navigation_escape.sim_time` covers the call.
- `tests/test_sim_time.py` (new, 10 tests, fakes only).

## Why

Twin 2026-10-06 22:01 local (t-cbcef8d39a3afb70, Desktop execution 18bcc321):
the detour took 68.1 s against ~33 s. BackUp took 9.1 s (usually 4.9-5.5),
the waypoint leg 23.9 s (9.2-9.9) and the goal leg 33.7 s (16.9-18.3). There
was no stall of 2 s or more, no recovery and no progress failure. The
controller logged its loop rate as 29.4 / 13.9 Hz only in that call, which
points at sim time and wall time drifting apart. Telling "the simulator was
slow" from "the robot was slow" took Nav2 logs; the result now says it.

Threshold basis: at 0.5 every wall duration is at least doubled. The twin
usually runs near 0.7 (CPU-only LiDAR rendering, `twin_nav2_params.py`), and
the 22:01 stretch of 1.8-2.4x on those runs is about 0.3-0.4.

## Verified

- `make PYTHON=<repo .venv>/bin/python verify`: ruff clean, 1606 passed, 1 skipped; `flyto-index verify
  --strict`: all PASS, exit 0.
- Fakes: steady 0.7 is not slow; a 3 s spell at 0.35 sets `sim_slow` on the
  minimum window; fewer than 2 samples is unmeasured; a reset is flagged; a
  wall clock going backwards is ignored; the env threshold is validated; legs
  combine; real is not applicable; rclpy simulation says why. Rosbridge
  subscribes `/clock` (typed, throttled) only in simulation and feeds a running
  track. Hardware neither subscribes nor measures, even when a `/clock`
  message arrives. An escape reports per-leg and call-level records.

## Not verified

- Not run against the twin or the robot. The `rtf_*` values of the 22:01 run
  are reconstructed from Nav2 phase durations, not measured with this code.
- rclpy transport: no clock subscription (it would mean a Python callback per
  simulator step). Simulated motions there say so.
- The 0.6.2 stationary stall is still unexplained and has not recurred on
  0.6.3+ (see `2026-10-06-goal-stall-instrumentation.md`).

## Follow-ups

- On the next twin series, read `motion_outcome.sim_time` before treating a
  slow motion as a robotics defect.
