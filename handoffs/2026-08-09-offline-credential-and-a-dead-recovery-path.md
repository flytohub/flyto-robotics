# The offline credential path, and a recovery portal that had been dead for hours

Owner: claude
Branch: main
Date: 2026-08-09

## The enterprise credential path is complete and verified on hardware

`LoadCredentialEncrypted=` now works end to end. The runner already preferred
`$CREDENTIALS_DIRECTORY`; what was added is everything around it.

- `scripts/provision-device-credential.sh` — credential on **stdin**, never
  argv (argv is readable through `ps`). Validates the document before
  encrypting, because a malformed credential encrypts fine and fails at boot.
  Decrypts what it wrote and compares through a hash before replacing anything.
- `deploy/systemd/flyto-job-runner.service.d/enterprise-credential.conf` — a
  drop-in, not a fork. A test refuses `ExecStart`/`User`/`Restart` in it: this
  repo has already paid for a second copy that drifted.
- Three files must agree on one string. A mismatch does **not** fail — the
  runner silently falls back to the on-disk file. That agreement is asserted.

Verified on the robot with a synthetic credential:

```
CREDENTIALS_DIRECTORY = /run/credentials/run-u89.service
SOURCE: systemd tmpfs
secret on disk: False
```

and a negative control with no credential loaded correctly found nothing, so
the positive result was not passing for the wrong reason.

**What it does not buy on this hardware.** The Pi has no TPM, so systemd falls
back to a root-only host key on the same disk. The script says that in those
words rather than letting an operator assume TPM sealing. On a host that has
one, the ciphertext is sealed to it and a copied disk decrypts to nothing.

## The recovery portal had failed 760 times and nothing said so

Found while verifying the above. It binds `10.77.0.1`, the USB gadget address,
which exists only with a cable. With no cable the bind failed EADDRNOTAVAIL,
the service died, systemd restarted it — and the unit reported **`activating`**,
which reads as "coming up", not "dead since 22:53".

Two things made it invisible:

1. A permanently-restarting service does not report `failed`. It reports
   `activating`, forever.
2. **`robot_doctor` was not watching it.** `SERVICE_NAMES` had the three
   services that move the robot and not the one that explains why it stopped,
   so `services.healthy` stayed `true` throughout.

A health check that does not watch the recovery path will always look best
exactly when it is wrong.

Fixed with `IP_FREEBIND`: hold the port for an address the host does not have
yet, answer the moment it appears. Binding `0.0.0.0` would have fixed the crash
by publishing diagnostics on Wi-Fi, which is worse than the disease, and a test
refuses that shortcut. The portal is now in `SERVICE_NAMES`.

Verified on the robot: listening on `10.77.0.1:8770` with no cable; with the
address temporarily added to `lo`, `/health`, `/` and `/v1/diagnostics` all
answered 200 and returned live JSON; after removing it the service stayed up
with no restart. **Still not verified with an actual cable** — that remains
open from the 2026-08-08 recovery handoff.

## A mistake worth not repeating

`provision-device-credential.sh` originally ran
`install -d -m 0700 "$(dirname "$OUTPUT")"` unconditionally. Testing it with
`--output /tmp/test-device.cred` therefore set **`/tmp` itself to 0700 root**
on the live robot, locking every other user out. Restored to 1777 by hand; the
script now only creates a directory that is absent and never re-permissions one
it did not make.

The lesson is narrow and worth keeping: a script that hardens a path must own
that path. `dirname` of an argument is not owned.

## Where the safety limits actually stand

Recorded here because it keeps being asked. `SafetyLimits` are **not**
hardcoded: they are per-job fields with hard clamps at parse time
(`obstacle_stop_distance` 0.15–2.0 m, `max_linear_speed` 0.02–0.5 m/s). What is
missing is not configurability but *scope* — there is no installation-level
profile, so changing "this robot in this building" means editing every job, and
nothing records who changed a limit or when. See the reply thread of
2026-08-09 for the proposed installation profile and characterisation routine;
neither is built.
