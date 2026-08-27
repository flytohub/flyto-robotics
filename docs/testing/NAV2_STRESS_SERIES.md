# Nav2 stress series

`scripts/run_nav2_stress.sh` runs the same synthetic Nav2 mission contract in
Gazebo for successful missions, the three fail-safe scenarios
(`lidar_dropout`, `odometry_freeze`, and `nav2_lifecycle_failure`), and an
expired-grant probe. The command writes generated evidence below
`results/nav2-stress/<run-id>/`; that directory must remain untracked.

## Running a series

The default invocation runs five successful missions and one instance of each
fault scenario:

```sh
make nav2-stress
```

Set `FLYTO_ROBOTICS_STRESS_RUN_ID` to a safe, unique label when a stable output
location is needed. `FLYTO_ROBOTICS_ROS_DOMAIN_ID` selects the first ROS domain.
Without a named profile, `FLYTO_ROBOTICS_STRESS_SOAK_RUNS` accepts 1 through
100.

Named campaign profiles own both their round count and their per-round success
count, so `FLYTO_ROBOTICS_STRESS_SOAK_RUNS` must be unset:

| Profile | Type / level | Rounds | Required successes | Required faults | Stop latency | Post-stop drift |
| --- | --- | ---: | ---: | ---: | ---: | ---: |
| `baseline-l1` | baseline / 1 | 1 | 5 | 3 | <= 750 ms | <= 0.05 m |
| `load-l2` | mission-load / 2 | 2 | 20 | 6 | <= 700 ms | <= 0.05 m |
| `fault-l3` | fault-repetition / 3 | 3 | 15 | 9 | <= 650 ms | <= 0.04 m |
| `endurance-l4` | long-soak / 4 | 5 | 100 | 15 | <= 600 ms | <= 0.03 m |
| `mixed-l5` | mixed-pressure / 5 | 10 | 200 | 30 | <= 500 ms | <= 0.02 m |

Exact examples (run IDs are synthetic labels):

```sh
FLYTO_ROBOTICS_STRESS_PROFILE=load-l2 make nav2-stress

FLYTO_ROBOTICS_STRESS_RUN_ID=load-l2-001 \
FLYTO_ROBOTICS_STRESS_PROFILE=load-l2 \
FLYTO_ROBOTICS_ROS_DOMAIN_ID=91 \
make nav2-stress
```

Pressure profiles select their required campaign automatically:

| Pressure profile | Campaign | Minimum executions | Applied pressure |
| --- | --- | ---: | --- |
| `resource-r1` | `fault-l3` | 24 | 1500 mCPU, 2048 MiB |
| `network-n1` | `fault-l3` | 24 | 2000 mCPU, 3072 MiB, 100 ms delay, 20 ms jitter, 1% loss on fault runs |
| `endurance-e1` | `endurance-l4` | 115 | 2000 mCPU, 3072 MiB |

Set `FLYTO_ROBOTICS_PRESSURE_PROFILE` to use one. Supplying a conflicting
campaign profile or an unknown profile exits before execution.

```sh
FLYTO_ROBOTICS_STRESS_RUN_ID=network-n1-001 \
FLYTO_ROBOTICS_PRESSURE_PROFILE=network-n1 \
FLYTO_ROBOTICS_ROS_DOMAIN_ID=91 \
make nav2-stress
```

The resilience evidence registry fixes five independent contract IDs. These
are evidence-builder/parser contracts; this repository does not claim that a
unit-test invocation ran their real Gazebo or container experiment.

| ID | Episode | Minimum runs | Fixed acceptance metrics |
| --- | --- | ---: | --- |
| `runtime-network-r2` | #008 | 2 | stop <= 650 ms; drift <= 0.04 m; OOM/process deaths = 0 |
| `resource-cliff-r2` | #009 | 12 | >= 4 cells; >= 3 repetitions/cell; >= 1 safe cell; OOM/process deaths = 0 |
| `compound-chaos-c1` | #010 | 2 | stop <= 650 ms; drift <= 0.04 m; OOM/process deaths = 0 |
| `gazebo-endurance-l4` | #011 | 115 | executions >= 115; pass rate = 1; memory slope <= 64 MiB/h; latency slope <= 50 ms/h; OOM/process deaths = 0 |
| `cold-repro-b3` | #012 | 3 | cold/passing runs = 3; container IDs = 3; source/image digest counts = 1; process deaths = 0 |

## Evidence and verdicts

Every scenario must exit successfully, produce a non-empty execution-evidence
JSON document, and pass `verify-ros2-execution-evidence` for its declared
scenario. A fault scenario is therefore successful only when its expected
fail-safe stop is observed; a crashed, timed-out, missing, or malformed fault
run is a failed run, never a successful fault result.

Each round produces `report.json` and `grant-expiry.json`. Named runs also
produce `campaign.json`; pressure runs additionally produce
`pressure-report.json`. Builders immediately parse their output again with the
strict contract parser. JSON snapshots are SHA-256 hashes over canonical JSON,
and parsers reject unknown or missing fields, duplicate identities, inconsistent
derived checks, altered thresholds, and mismatched snapshots.

Campaign thresholds come from the source-controlled profile registry, not from
the evidence document. All rounds must pass, required execution volumes must be
met, stop latency and post-stop drift must remain within the selected profile,
and unexpected process deaths must be zero. Pressure reports additionally
recompute CPU, memory, network, recovery, OOM, exit-code, and execution-volume
checks from the captured cgroup records. Any false check makes the aggregate
verdict false and the script exits non-zero.

If a scenario exits non-zero or omits its evidence, the script writes a
content-addressed `pressure-incident.json` with redacted markers and hashes of
the raw inputs, then exits non-zero. Logs, cgroup records, reports, build output,
and any crash output are generated evidence only; do not add them or a core
artifact to version control.

## Verification, artifacts, and cleanup

The contract-only checks require neither Docker nor ROS:

```sh
.venv/bin/python -m pytest -q tests/test_ros2_stress_evidence.py
make PYTHON=.venv/bin/python verify
```

They verify strict schemas, fixed registries, canonical snapshots, derived
verdicts, duplicate rejection, and the runner's bounded preflight refusals.
They are not real pressure, Gazebo, Nav2, cgroup, network-fault, endurance, or
cold-container evidence. Only a successful `make nav2-stress` invocation on a
host with Docker and the self-contained image can produce that operational
evidence.

Outputs are rooted at `results/nav2-stress/<run-id>/`. Per-round scenario JSON,
pressure records, `report.json`, and `grant-expiry.json` remain beside the
aggregate `campaign.json` and optional `pressure-report.json`; an early
scenario failure instead leaves `pressure-incident.json`. Treat the whole run
directory as one evidence unit. After exporting any evidence required by the
lab, remove that exact run directory and confirm `git status --short` contains
no generated ROS, Gazebo, build, test, log, crash, or evidence artifacts.
