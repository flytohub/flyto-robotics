# Failures that report success

Owner: claude
Branch: main
Date: 2026-08-08

Follows [the robot turns but will not drive](2026-08-08-move-fails-while-turn-succeeds.md).
That day produced three defects of one shape, which was enough to suspect there
were more. A read-only adversarial sweep looked for the rest: 51 agents, five
independent lenses, every candidate judged by two skeptics that both defaulted
to refuting. Eight survived. All eight are fixed here.

## The shape

**A condition the system cannot evaluate, reported as the reassuring answer.**

Not a crash and not a wrong number that surfaces — those get noticed. This one
is quiet: the gate cannot tell, so it says yes; the diagnostic cannot read, so
it says healthy; the snapshot cannot be dated, so it renders as current.

The tell is almost always a sentinel that is also a valid comparison operand.
`math.inf` compares as "far". `"unknown"` sits in a set beside `"active"`.
`99` reads as metres. None of them announce that they are placeholders.

## What was wrong, and what it would have cost

### The obstacle guard drove blind (`8203581`)

`sector_field` drops every beam outside `[range_min, range_max]` and leaves the
sector at infinity. Every layer of `_obstacle_guard` — emergency,
omnidirectional, directional, lateral — is written `math.isfinite(x) and
x < limit`, so an all-infinite field skipped **all four** and returned clear.

Four physically different sweeps were indistinguishable from an open corridor:

| | |
|---|---|
| stalled rotor | every beam `0.0` |
| covered sensor | every beam under `range_min` |
| wall at 0.10 m | under this LDS's 0.10 m floor, so invisible |
| dead driver | every beam `NaN` |

The third is the one to sit with: **an obstacle close enough to be unmeasurable
was safer to the guard than one at 0.30 m.** The run then recorded
`succeeded, safety_stop_count: 0`, byte-identical to a clean corridor.

Fixed by classifying beams rather than filtering them. Past `range_max` —
including `+inf`, which is how most drivers spell "no echo" — is a definite
answer. NaN, non-positive, or under `range_min` is not an answer. A sector
where nothing was definite lands in `RangeField.blind`, and `blind_for()`
consults exactly the sectors `blocking()` does.

Simulation is unaffected: Gazebo's `+inf` sweeps classify as definite. Partial
degradation is not over-refused — one usable beam redeems its sector.

### The doctor called an uninspectable robot healthy (`23135ff`)

`_service_state` returns `"unknown"` when systemctl is missing, hangs past its
three second timeout, or answers something unrecognised. Both the classifier
and the payload builder filtered it out alongside `"active"`:

```
primary_reason_code: healthy      quality: good
services.healthy:    true         action_codes: []
```

This is the tool an operator reads to decide whether a robot is fit to run.
`"unknown"` now has its own reason code at degraded quality. A known failure
still outranks an unread one — the first names a fix, the second an absence.

### The recovery portal aged into a lie (`af00237`)

`observed_at` was passed through untouched, so a frozen `latest.json` rendered
exactly like a live one: green GOOD, the reason, "No action required", with the
age in a timestamp at the foot that nobody reads once the headline says the
robot is fine.

A frozen snapshot is the normal consequence of the diagnostic writer dying, and
a dead writer is precisely when someone opens this page. The doctor timer runs
every 60s, so past five missed runs the view now reports stale, says how old,
offers an action, and carries a banner. An unreadable timestamp counts as stale
— not knowing the age is not evidence of freshness.

### Seven tests verified nothing (`9e62888`)

The lab driver's safety behaviour was asserted by searching its source for
strings, one requiring a specific line break inside a subscribe call.

The proof is in the commit: the logic moved to another file **byte-for-byte
unchanged in behaviour**, and the test went red. A test that fails when nothing
changed will pass when everything breaks.

The state machine now lives in `lab_safety_observation` with 17 tests that run
it. What is still structural is labelled as such at the top of the file.

## What is still not covered, said plainly

`tests/test_gazebo_lab_driver.py` and `tests/test_lima_gazebo_contract.py`
still contain source-text assertions for world path accumulation, the replay
delay, and the delivery-gate ordering. **They are not evidence that anything
works.** The way to fix one is to move the logic into a module that imports no
ROS, as was done here — not to add another string to search for.

## Verified on the robot

Every fix was deployed and run on the lab TurtleBot3 at `9e62888`:

```
live lidar        blind=none across 5 sweeps, would_refuse=False
                  (the new refusal does not fire on a healthy sensor)
mission           cmd_vel Twist (provisional) -> TwistStamped at +0.3s
                  odometry at +0.005s, safety stop for a real obstacle
forward           1.49 m clearance passed the pre-check, so it is not
                  over-refusing either
doctor            services.healthy true, unknown_service_ids []
                  — every service genuinely read
```

`make verify`: ruff clean, 499 passed, 3 skipped.

The robot is still boxed in and every motion ends `failed` with
`safety_stops=1`. **That is the safety system working.** Move it before reading
a failure there as a defect.

## Two traps in this workspace

**Bytecode cached outside the tree.** This Mac's Xcode Python writes `.pyc`
files to `~/Library/Caches/com.apple.python/<abs path>/`, not to
`__pycache__/`. Deleting `__pycache__` does nothing. Worse, invalidation is by
size and mtime-second, and `= 15.0` is the same length as `= 30.0` — so a quick
edit that preserves file size is silently ignored, and **the suite runs against
code that is not on disk**. This cost a confusing half hour: the source read
15.0 while the import returned 30.0. Clear that directory when a change appears
to have no effect.

**A chained verify hides everything after the first failure.** `make verify` is
`lint test assets dry-run ...`. `ruff` was installed here without its binary,
so `lint` failed and the other nine targets had never run on this machine. A
green-looking gate that exits early is worse than a red one.

## Method note

The sweep was read-only and every finding was reproduced independently before
being fixed — one of the agents' claims contained a wrong contrast (it cited a
0.10 m wall as the working case when 0.10 m is itself below `range_min` and
equally invisible), which only surfaced by re-running it. Treat agent findings
as leads.

Every fix here has a test that was confirmed to **fail against the previous
behaviour** before being kept.
