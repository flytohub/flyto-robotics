# The first cloud-dispatched mission a real robot ever completed and reported

Date: 2026-08-08 (overnight from 2026-08-07)
Spans: `flyto-robotics` (merged to `main`), `flyto-cloud` (deployed to production),
live hardware on `flyto-robot.local`

## What this is

The robot half of the Space task closed loop, taken from "every layer written"
to "every layer proven against the real thing". Six defects sat between those
two states. Every one of them was invisible to the test suite, because every
one of them was a place where a fixture spoke a shape the real service never
sends.

The loop now runs end to end: a task dispatched from the cloud reaches this
robot, the robot turns, and the evidence it measured comes back and is
assessed. Verified 00:05 on 2026-08-08 — see *The run* below.

## What was wrong, in the order the ladder found it

Each of these was found by testing one rung against the real dependency
before wiring the next. Building the whole chain first is what hid them.

1. **The runner spoke a completion contract nothing accepts.** It posted
   `{"status": "succeeded", "result": {...}}`. `JobCompleteRequest` requires
   `status` matching `^(success|failed)$` and carries workflow output under
   `variables`. It had only ever been tested against a fake cloud that
   accepted any shape, so no mission this runner ran could ever have been
   reported — on any day, to any server.
2. **It waited for a mission state no gateway sends.** `TERMINAL_STATES` held
   `"succeeded"`; `MissionState.COMPLETED` is `"completed"`. A real mission
   would have been watched for the full five-minute window and then reported
   as "outcome unknown".
3. **It read the final pose from a field the payload does not have.**
   `final_pose` is the fixtures' name; `DeliveryGateway._session_payload`
   emits `pose`. A real mission produced no arrival evidence at all.
4. **The gateway on this robot carried a simulated rover's identity.** The
   unit ran with `--job examples/jobs/ai-space-delivery.json`, whose
   `robot_id` is `flyto-rover-sim-001`. Lab plans are for
   `flyto-tb3-lab-001`, so the gateway refused them — correctly. Worse, the
   sim job's safety envelope (0.35 m/s, 0.45 m obstacle stop) is looser than
   this TurtleBot3 warrants; the lab job's is 0.2 m/s and 0.25 m. Fixed with
   a drop-in at `/etc/systemd/system/flyto-delivery.service.d/lab-identity.conf`.
   Note the lab job bounds a mission at 30 s, so `--confirmation-timeout`
   must fit inside it — the gateway refuses to start otherwise, which is how
   that constraint was found.
5. **The session never recorded the range it measured.** The ROS backend
   computed the closest lidar return every scan, handed it to the controller,
   and dropped it: `session.minimum_range` was never assigned, so every
   ROS-backed mission reported `minimum_range: null`. Fixed in
   `ros2_delivery_runner.py`, with the infinity guard in `mission.closest_range`
   so "nothing measured" cannot leave as a number that reads like a wide open
   corridor.
6. **systemd ran a stale copy of the runner.** The unit executed
   `/home/ubuntu/flyto_job_runner.py` while `rsync` deployed the tree at
   `/home/ubuntu/flyto-robotics/`. The robot kept running yesterday's runner
   after every deploy. The unit now points at the deployed tree.
7. **The lease went out under a header the API does not read.** The runner
   sent `X-Flyto-Lease`; `routes_jobs.py` reads `x-flyto2-job-lease`. Every
   mission ran to completion and every report came back `409 Job lease is
   missing or invalid` — the robot moved, and the schedule went on believing
   the step was still running.

## Why the tests were green through all of it

The fake cloud accepted a completion carrying no lease at all, and the fake
gateway answered with `state`/`final_pose`/`succeeded` — three names the real
gateway never uses. A fixture that is more permissive than the service it
stands in for does not test an integration; it certifies a fiction. The
fixtures now speak exactly what `_session_payload` emits and enforce the
lease, so this class of drift fails in `pytest` rather than on hardware.

## The silence that made it expensive

The runner logged non-401/403 HTTP failures as `retrying in 6s` and nothing
else. A mission that ran, moved the robot, and could not be reported looked
identical to a flaky poll. It now logs the status and body, and a report
refused after a mission ran says exactly that:

```
job aba20525 ran (succeeded) but the report was refused: 409 {"error":"Job lease is missing or invalid"}
```

That one line is what found defect 7.

## The run

```
00:05:05  job aba20525-77f2-485e-a54f-f8dd41195f92 -> plan shortcut.turn.left.90deg.v1
00:05:13  job aba20525 reported succeeded: completed | evidence: ['arrival.pose', 'clearance.measurement']
```

And on the cloud side, the task's own timeline:

```
operator/dispatched : s1: <robot command> queued on 198501b5… as aba20525…
resource/completed  : s1: reported by the execution job
validator/assessed  : missing zone.overview
scheduler/escalated : zone.overview missing: added <camera command> on 49f5517f… (vision.observe)
```

Collected evidence: `arrival.pose {"x": -0.0, "y": 0.0, "yaw": 0.022}` and
`clearance.measurement (nearest obstacle 0.50 m)` — a real lidar reading from
a real turn. The escalation is the whole design working: the mission was short
of `zone.overview`, and the registry's own matching scheduled the camera for
it without anyone asking.

## What is still open

- **The camera step cannot run yet**, for two independent reasons. The
  capture command uses `shell.exec`, which `flyto-core`'s module policy denies
  by default (`shell.*` — arbitrary host execution from a dispatched job is
  exactly the threat it exists for). And macOS has not granted camera access
  to the process that would run it. The right answer is a narrow capture
  module rather than relaxing the shell denylist; that module does not exist.
- **`flyto-delivery.service` is not in version control.** It lives only on the
  robot, which is how it kept a simulated rover's identity across a hardware
  deployment. It belongs in `deploy/systemd/` beside the other two units.
- The 90° turn is what the robot actually performs for a "clearance" reading.
  Turning rather than driving was chosen because front clearance measured
  0.72 m and a turn cannot collide; the lidar reading is the same kind either
  way.
