# Device executors

Operations flyto-cloud can dispatch that are not delivery steps.

`flyto_job_runner` routes any authored step whose module id starts with
`robotics.` through `flyto-modules-robotics` to the delivery gateway, and
everything else to the installed device-executor registry. That second path is
where commissioning work belongs: it happens once per venue, before any
delivery, and it must not be in the vocabulary the AI planner composes delivery
plans out of. A plan that could emit "begin remapping the building" halfway
through carrying something is a plan shape nobody wants to have to refuse.

## flyto-mapping

Records a venue map as a dispatched job rather than an ssh session. Three
module ids:

| module id | what it does |
|---|---|
| `mapping.start` | Checks the configured preconditions, then starts the configured SLAM unit. |
| `mapping.save` | Saves what SLAM has built to the configured map directory, then stops SLAM. |
| `mapping.abort` | Stops the configured SLAM unit and keeps nothing. |

The driving in between is not this executor's job and deliberately so — the
robot already exposes `robotics.motion.move_relative` and
`robotics.motion.turn_relative` as approved capabilities with the sensor gates
and obstacle stops already applied to them. Mapping does not need a second,
unreviewed way to move a robot.

`mapping.save` reads the written `.yaml` and `.pgm` back and reports the
resolution and cell count as evidence. `map_saver_cli` exits zero having
written a map of nothing, and a one-cell map reaching Nav2 as a working venue
is the failure this is guarding.

### Installing

```bash
# 1. the executor, its settings loader, and the unit it drives
install -D -m 0755 deploy/executors/flyto_mapping_executor.py \
  /home/ubuntu/executors/flyto_mapping_executor.py
install -D -m 0644 deploy/executors/mapping_settings.py \
  /home/ubuntu/executors/mapping_settings.py
sudo install -D -m 0644 deploy/systemd/flyto-slam.service \
  /etc/systemd/system/flyto-slam.service
sudo systemctl daemon-reload

# 2. this machine's settings and the manifest that names them
sudo install -D -m 0644 deploy/executors/turtlebot3-mapping.json \
  /etc/flyto/mapping.json
sudo install -D -m 0644 deploy/executors/flyto-mapping.json \
  /etc/flyto/device-executors/flyto-mapping.json

# 3. privileges — read this before running it
sudo install -m 0440 deploy/executors/flyto-mapping.sudoers \
  /etc/sudoers.d/flyto-mapping
sudo visudo -c
```

`turtlebot3-mapping.json` is an installable example, not a universal robot
profile. Before installing it on another machine, change the systemd units,
map directory, ROS setup path, ROS domain, battery topic, and measured voltage
floor to match that machine. The executor refuses unknown keys, invalid unit
names, relative paths, out-of-range domains, and a battery topic or voltage
floor supplied without the other.

`readiness_unit`, `navigation_unit`, `ros_setup`, `ros_domain_id`,
`battery_topic`, and `min_mapping_volts` are optional. Omitting a unit states
that the machine has no corresponding systemd precondition. Omitting both
battery fields explicitly disables the pre-run pack check and is recorded in
the successful start evidence; do not copy that omission onto a battery robot
merely because its telemetry has not been wired yet.

The manifest passes `/etc/flyto/mapping.json` with `--config`. Changing the
settings path therefore means changing the manifest command too. Environment
variables cannot carry these values: the device-executor registry deliberately
starts the process with an empty environment.

### About step 3

Run `visudo -c` after installing. A malformed file in `/etc/sudoers.d` can lock
sudo out of the machine, and this robot has no console attached.

The file grants exactly two commands by full path and nothing else, because
`systemctl *` would hand a general privilege-escalation primitive to anything
that can reach the job runner, and the reason the executor boundary exists is
that a dispatched job must not become arbitrary host execution.

**On the current TurtleBot3 that enumeration buys nothing, and it is worth
knowing why.** Ubuntu's cloud-init ships `/etc/sudoers.d/90-cloud-init-users`
containing:

    ubuntu ALL=(ALL) NOPASSWD:ALL

flyto-job-runner.service runs as `ubuntu`. So the executor can already run any
command as root, this file is a no-op, and the boundary's central property —
that a dispatched job cannot become arbitrary host execution — is **not
enforced on that machine**. It is enforced by the contract (bounded JSON, a
fixed module-id set, a timeout) and by nothing below it.

That is a property of the image, not of this executor, and it is left alone
here on purpose: narrowing a running robot's own administrative access is a
change that can lock out a machine with no console, and it is not a decision to
make as a side effect of installing a mapping feature. It is the right decision
to make deliberately, before the robot is on a venue network.

This file is still worth installing: it is correct, it is what the executor
needs on any host that does *not* hand `ubuntu` blanket root, and it stops
being a no-op the moment the cloud-init grant is tightened.
