# Cloud handoff and robot execution receipt

Owner: codex
Branch: main
Date: 2026-08-22

## What changed

`deploy/flyto_job_runner.py` validates the exact versioned Cloud handoff before
starting a trace-bearing Space job. `flyto_robotics/delivery_gateway.py` emits
one cached, bounded terminal execution receipt; the runner validates its plan
binding, status, fields, and canonical digest before forwarding it. Focused
tests cover success, missing receipts, tampering, retargeting, and the
trace-without-handoff refusal.

## Why

The queue, local gateway, and Cloud evidence evaluator previously worked, but
their joins were implicit. The edge now proves which scheduled workflow it
accepted and returns one reconcilable terminal record without turning action
success into mission success.

## Verified

Focused delivery-gateway, plan-endpoint, and job-runner tests passed. Final
`make verify` passed Ruff, 1467 tests with 1 skipped, asset validation, all
required dry-runs, lab/facility contracts, ROS 2 pairing, and execution-grant
checks. Strict Flyto Indexer verification passed 18/18. A cross-repository
smoke accepted a Cloud-generated handoff and the same terminal receipt through
both the edge and Cloud validators.

## Not verified

No ROS/Gazebo deployment, physical motion, TPM-backed attestation, package
publication, or protected-branch release was performed.

## Follow-ups

Run a calibrated physical Cloud-to-robot mission only under the normal site
safety process, then retain the returned receipt together with independent
mission evidence.
