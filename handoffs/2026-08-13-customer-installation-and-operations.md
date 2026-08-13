# Customer installation and operations closure

Owner: codex
Branch: main
Date: 2026-08-13

## What was missing

The accepted lifecycle implementation and package metadata had no single
customer-facing path from a supplied wheel through rehearsal, install, daily
status, update, rollback, support collection, and bounded device-event export.
The built-wheel test proved `flyto-robot` and the presence of the lifecycle
registry, but not the second customer command or the registry's profile data.

## What changed

`docs/INSTALLATION.md` is the wheel-first operator runbook and README links to
it. It says explicitly that `generic` is middleware- and vendor-neutral and
that `ros2` extends it additively with one adapter service. Examples use only
synthetic versions, local paths, and placeholder notes.

`tests/test_packaging.py` now reads the built wheel's entry-point metadata for
both `flyto-robot` and `flyto-device-events`, parses its profile JSON, installs
that wheel offline with `--no-index --no-deps` into a fresh temporary virtual
environment, removes checkout injection through `PYTHONPATH`, and executes
both installed commands with `--help`. The profile assertions preserve proof
of the neutral generic base and additive ROS 2 relationship.

## Evidence boundary

This closure owns documentation plus contract, packaging, and lifecycle-test
proof. Existing Gazebo results remain inherited repository evidence referenced
by the runbook and STATE; no Gazebo run was performed for this documentation
and packaging closure. It does not claim physical cold-boot repetition,
distance calibration, hardware E-stop, real sensor-loss response, camera
operation, device deployment, artifact publication, or customer-site
acceptance; none of those activities was performed in this round. No robot
login, motion, staging, commit, push, publication, or Codex audit acceptance is
claimed.

Verification results belong to the exact post-change commands recorded by the
implementation worker and the host's subsequent strict Indexer validation;
historical results from the failed `unplanned_diff` round are not acceptance
evidence for this revision.

Worker evidence for this revision: the focused packaging run completed with
`2 passed`; `make verify` passed Ruff, then pytest completed with `980 passed,
4 skipped, 40 failed, 17 errors`. Every displayed failure/error was the worker
sandbox refusing loopback or Unix socket creation with `PermissionError:
[Errno 1] Operation not permitted`; `make verify` exited 2 at `test`, so its
later targets did not run. This is recorded as a failed full verify, not a pass;
the sandbox limitation is retained as provenance rather than erased.

Successful pre-rework evidence is separately bound to governed job
`job_5c2afa927da54eaa920f0128` and exact revision
`e8d3db0b1863382409558cfbe195f09e45c37baec0acc00de8695347974a1b0d`.
The governed route check passed for that revision, and the Codex auditor then
independently ran `make verify` in the repository: exit 0, Ruff clean, `1037
passed, 4 skipped`, with all asset, contract, and dry-run gates completed. This
is durable verification evidence for the pre-rework revision; it is not a
claim that the present rework revision has received Codex audit acceptance.
