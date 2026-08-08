# The site decides the safety limits, not the job

Owner: claude
Branch: main
Date: 2026-08-09

## What was actually wrong

`SafetyLimits` was never hardcoded. It has always been a job field with hard
clamps at parse time (`obstacle_stop_distance` 0.15–2.0 m, `max_linear_speed`
0.02–0.5 m/s). The complaint that the numbers were fixed was aimed at the right
problem through the wrong window.

The problem was **scope and authority**. Limits lived on the job, so "this
robot, in this building, may not exceed X" had to be written into every job and
could be forgotten in any one of them. Changing one thing meant changing
everything, which is indistinguishable from not being able to change it.

## What it is now

An installation profile folds over the job:

```
job 0.40 m/s, site 0.25   ->  0.25, logged
job 0.20 m/s, site 0.25   ->  0.20, the job is stricter and is left alone
```

**The direction is data, not five hand-written comparisons.** A lower speed is
safer; a *higher* stop distance is safer. `CONSTRAINABLE` declares which way
each field runs and everything derives from it. An implementation that treated
them alike would let a job drive closer to things than the site permits, and
would still pass a test suite that only checked speeds — so the tests check
both, and a deliberately backwards version was confirmed to fail them.

Enforced in `contracts._safety()`, the single place a `SafetyLimits` is built.
Not at call sites: a site limit applied at call sites is a site limit missing
from whichever path someone adds in a hurry. There is no disable switch — one
that could be flipped from the job side would not be a site limit.

A profile that exists but cannot be parsed is an **error**, never treated as
absent. A site that wrote a limit and had it silently ignored would believe it
was in force.

## The interface

On the mission gateway: loopback, token-authorised — what an offline
installation has instead of a cloud console.

- `GET /v1/safety-profile` returns the limits, **which way safer runs for each
  field**, and the recent history. A limit without its history invites "who set
  this to 0.2?" with no way to answer.
- `POST /v1/safety-profile` requires `changed_by` and `reason`, and **refuses
  outright if the audit cannot be written**. A limit that can be moved without
  leaving a trace is not governed, and the moment it matters is exactly the
  moment someone would rather it left none.
- Whether a change relaxed safety is **computed from the numbers**, not taken
  from the operator's description. A relaxation labelled as a tightening is the
  entry an audit exists to catch.

No lock, and none needed: a change takes effect at the next job load, and a
mission already running resolved its limits when it started.

## Verified on the robot

`0018d77`, live on the lab TurtleBot3:

```
job asks for : 0.2 m/s, stop at 0.25 m
site set to  : {'max_linear_speed': 0.12, 'obstacle_stop_distance': 0.8}
same job now : 0.12 m/s, stop at 0.8 m

site safety profile: max_linear_speed: job asked 0.2, which is above the
  site ceiling of 0.12; using 0.12
site safety profile: obstacle_stop_distance: job asked 0.25, which is
  inside the site floor of 0.8; using 0.8
```

Note the two messages use opposite language for the two directions. That is the
type speaking, not the copy.

The whole suite also runs on the robot: 49/49 for this module there.

**Nothing was left set.** The demonstration pointed `FLYTO_SAFETY_PROFILE` at a
temporary directory; the robot has no site profile, which is the correct state
for a machine that has not been commissioned into one.

## Not built

The characterisation routine — measuring real stopping distance, sensor latency
and control jitter, and *proposing* limits from them — is the other half and is
not started. The profile is where its output would land.

Two positions worth keeping when it is:

- **Measure, do not collide.** The number wanted is how far the robot travels
  after a stop command, and that is measured precisely in free space. A
  collision-derived margin needs collisions in its data forever, including with
  people, and the same contract runs on a hospital delivery robot where that is
  not cheap.
- **It proposes; a human accepts.** Auto-applying a machine-derived safety
  limit is the single thing that should never be automatic. And it may only
  ever propose the more conservative direction: measurement error and mechanical
  wear both run one way.
