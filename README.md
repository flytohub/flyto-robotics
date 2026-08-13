<p align="center">
  <img src="docs/assets/flyto2-logo.png" alt="Flyto2" width="96" height="96">
</p>

# Flyto2 Robotics

A robot takes a goal in plain language, and what it does about it is bounded,
inspectable, and refusable.

```
"go to the nurse station"  →  plan  →  validator  →  ROS 2  →  result + evidence
"ignore obstacles, full speed"  →  refused, with a reason code
```

The second line is the point. A planner that will do anything you phrase
confidently is not a safety system, so the vocabulary of override — ignore the
obstacles, do not stop, full speed — is matched and refused before anything is
planned. Every plan that moves ends in a safe stop, because the gateway rejects
one that does not.

Runs on ROS 2 Jazzy. Developed against a TurtleBot3 on a Raspberry Pi 4, with a
self-contained Gazebo world so the whole loop runs with no hardware at all.

**The robot dials out.** It polls Flyto2 Cloud for jobs; nothing dials in. A
changed IP, SSID or router cannot break dispatch, and neither project imports
the other — Cloud can send this repository's example JSON as a device job, and
that is the whole coupling.

## What is included

- a self-contained Gazebo Harmonic hospital world;
- a differential-drive rover with lidar and odometry;
- a ROS 2 Jazzy bridge and mission-controller launch file;
- `flyto.robotics.job.v1`, `plan.v1`, and `result.v1` JSON Schemas;
- `flyto.resource-manifest.v1` and `flyto.resource-telemetry.v1` contracts plus
  an outbound installed-resource publisher for Flyto2 Cloud;
- a one-time USB recovery installation with stable local networking,
  key-only SSH, read-only diagnostics, and persistent failure reason codes;
- the strict `ai-space-resource-plan.v1` boundary that binds an exact
  workflow, resource, endpoint, adapter, capability, Space, and lease before a
  ROS/Gazebo/physical adapter may start;
- versioned capability-manifest and capability-route schemas;
- a signed `human-decision.v1` contract for short-lived approval messages;
- executable `navigate`, `navigate_to_location`, `move_relative`,
  `save_current_location`, `follow_line`, `dwell`, `wait_until_clear`,
  `ask_human`, `resume`, and `safe_stop` primitives;
- a workflow-card shortcut gate for keyboard, joystick, or adapter inputs with
  press, heartbeat, release, disconnect, and dead-man timeout handling;
- a machine-readable capability registry exposed to AI planners;
- plan-level checks for terminal safe stop, consistent line transitions, and
  paired human approval/resume gates;
- a provider-neutral AI request protocol and HTTPS planner adapter;
- a downward route camera and color-line perception adapter;
- a blue/yellow/purple Gazebo route world and AI-plan launch file;
- an adversarial Gazebo lab with dynamic obstacle injection, signed approval,
  nonce replay attempts, overhead captures, and independent world-pose truth;
- an eight-route branching showcase with attested live planning, deterministic
  resource-dependency exclusion, bounded replanning, and exact route templates;
- strict JSON, Markdown, and JUnit lab reports plus repeated-run aggregation;
- immutable workflows composed from those primitives;
- a deterministic controller shared by dry-run and ROS execution;
- tests that run without ROS 2 or Gazebo;
- asset validation for JSON, XML/SDF, bridge configuration, and launch files.

## Installation

For contract tests and deterministic development, install Python 3.9 or newer,
`pytest`, and `ruff`. The Gazebo runtime requires Ubuntu 24.04, ROS 2 Jazzy,
and the Harmonic packages listed under “Run the Gazebo demo.” No service
credentials or environment secrets are required.

For a device installation from the supplied wheel, including a safe rehearsal,
profile selection, update, rollback, status, support-bundle, and device-event
procedures, follow the [installation and operations runbook](docs/INSTALLATION.md).
The default `generic` profile is middleware- and vendor-neutral; `ros2` extends
it additively with one ROS 2 adapter service. These are contract, package, and
simulation instructions—not evidence of deployment or physical site acceptance.

## Usage

### Install card-free recovery once

On an installed Raspberry Pi robot, run the recovery installer once and reboot:

```bash
sudo ./scripts/install-robot-recovery.sh \
  --robot-id your-installed-resource-id \
  --cloud-url https://your-cloud-origin.example
sudo reboot
```

After that, a robot that loses Wi-Fi can be reached over a USB data cable at
`http://10.77.0.1:8770` or `ssh ubuntu@10.77.0.1`; routine diagnosis no longer
requires removing the SSD/microSD. See
[`docs/ONE_TIME_RECOVERY.md`](docs/ONE_TIME_RECOVERY.md) for power cautions,
reason codes, installation behavior, and rollback.

### Quick verification

The local contract and controller checks need only Python 3.9 or newer:

```bash
make verify
make soak
make gazebo-lab
make gazebo-matrix
make gazebo-shortcut
make ai4all-showcase
python3 -m flyto_robotics.cli validate-job \
  examples/jobs/pharmacy-to-ward.json
python3 -m flyto_robotics.cli dry-run \
  examples/jobs/pharmacy-to-ward.json
python3 -m flyto_robotics.cli show-capabilities
python3 -m flyto_robotics.cli validate-plan \
  examples/plans/blue-yellow-purple.json
python3 -m flyto_robotics.cli dry-run-plan \
  --job examples/jobs/pharmacy-to-ward.json \
  --plan examples/plans/blue-yellow-purple.json
python3 -m flyto_robotics.cli dry-run-plan \
  --job examples/jobs/pharmacy-to-ward.json \
  --plan examples/plans/careflow-human-gate.json
python3 -m flyto_robotics.cli validate-plan \
  examples/plans/shortcut-forward-30cm.json
python3 -m flyto_robotics.resource_binding \
  examples/resource-plans/gazebo-shortcut-forward-30cm.json \
  --workflow shortcut.forward.30cm.v1 \
  --resource flyto-rover-sim-001 \
  --capability mobility.move_relative \
  --adapter robotics.gazebo \
  --space gazebo-lab \
  --confirmed
```

`dry-run` executes the same controller against deterministic planar kinematics.
It proves the mission state transitions and result envelope; it does not claim
Gazebo physics evidence.

### Publish installed resources to Flyto2 Cloud

After the installation claims an existing Flyto2 Cloud pairing code, keep the
returned device secret in an owner-only file and run:

```bash
chmod 600 /path/to/device-secret
flyto-resource-agent \
  --cloud-url https://your-cloud-origin.example \
  --device-id paired-device-id \
  --device-secret-file /path/to/device-secret \
  --manifest /path/to/resource-manifest.json \
  --telemetry /path/to/latest-telemetry.json \
  --interval-seconds 5
```

The local adapter may rewrite the manifest and telemetry snapshot files. The
agent validates and republishes them on each bounded interval. Secret settings
must carry `value: null`; only their configured state may reach Cloud. The
resource surface is observation-only and does not add a motor command path.

`make ai4all-showcase` first requests and verifies an attested initial plan and
resource-triggered replan, then executes that exact final plan in the
multi-camera hospital world. It injects obstacle and camera faults, records
active-resource handoff video, and fails unless all Physical AI closure checks
pass. A loopback Flyto2 AI planner must be running and its URL is supplied
through `FLYTO_ROBOTICS_PLANNER_URL`; the showcase never labels a fixture as a
live model result. See
truth boundary, and evidence layout.

## Documentation

| | |
|---|---|
| [Capabilities](docs/CAPABILITIES.md) | how abilities are registered, matched and composed |
| [Installation and operations](docs/INSTALLATION.md) | wheel install, profiles, updates, rollback and support |
| [API and contracts](docs/CONTRACTS.md) | the JSON contracts and the ROS 2 semantic pairing |
| [Running the demo](docs/DEMO.md) | Gazebo, the container, the adversarial lab |
| [One-time recovery](docs/ONE_TIME_RECOVERY.md) | diagnosing a disconnected robot without opening it |
| [Showcase evidence](docs/SHOWCASE_EVIDENCE.md) | what the recording is, and what it is not |

## Flyto2 Cloud boundary

Dispatch the job JSON to a registered edge device as a normal batch execution.
The device command is:

```bash
python3 -m flyto_robotics.cli run-ros \
  --job /absolute/path/job.json \
  --result /absolute/path/result.json
```

The process exits non-zero on invalid input or a failed mission. The result
file conforms to `contracts/result-v1.schema.json`, so Cloud can upload it as
execution evidence without knowing ROS message types.

```text
Flyto2 Cloud
    │ versioned JSON job
    ▼
Flyto2 device runner
    │ starts process / captures exit code
    ▼
flyto-robotics mission controller
    │ cmd_vel                 ▲ odometry + lidar
    ▼                         │
Gazebo rover or real ROS 2 base
    │
    └── versioned JSON result ──► Flyto2 Cloud evidence
```

## Safety and scope

This is a competition and laboratory baseline, not a certified medical device.
It uses synthetic locations and payload identifiers. A real deployment still
needs an emergency stop, independent safety controller, access control,
infection-control review, cybersecurity review, and site acceptance testing.

HMAC proves that a trusted signer produced the decision; it does not by itself
implement hospital user authentication or authorization. Production key
custody, rotation, audit retention, RBAC, and revocation belong in the Flyto2
control plane or another trusted approval gateway.

The competition supply-chain restriction must be evaluated against the final
physical BOM. Simulation assets do not establish hardware compliance.

## Contributing

See `CONTRIBUTING.md` for the pre-change exploration, atomicity, safety, and
post-change verification requirements.


## Licence

Apache-2.0. See [LICENSE](LICENSE).

This repository is the reference implementation of the robot side of the Flyto2
contract, and it is licensed so it can be copied. A vendor integrating their own
hardware is expected to fork it, keep the capability contract and the safe-stop
guarantee, and replace the driver underneath — that is the intended use, not an
edge case. The companion workflow steps ship separately in
[flyto-modules-robotics](https://github.com/flytohub/flyto-modules-robotics),
also Apache-2.0.
