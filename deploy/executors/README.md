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
| `mapping.start` | Checks preconditions, then starts `flyto-slam.service`. |
| `mapping.save` | Saves what SLAM has built to `~/.flyto/maps/<name>`, then stops SLAM. |
| `mapping.abort` | Stops SLAM and keeps nothing. |

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
# 1. the executor and the unit it drives
install -D -m 0755 deploy/executors/flyto_mapping_executor.py \
  /home/ubuntu/executors/flyto_mapping_executor.py
sudo install -D -m 0644 deploy/systemd/flyto-slam.service \
  /etc/systemd/system/flyto-slam.service
sudo systemctl daemon-reload

# 2. the manifest the job runner discovers it through
sudo install -D -m 0644 deploy/executors/flyto-mapping.json \
  /etc/flyto/device-executors/flyto-mapping.json

# 3. privileges — read this before running it
sudo install -m 0440 deploy/executors/flyto-mapping.sudoers \
  /etc/sudoers.d/flyto-mapping
sudo visudo -c
```

### About step 3

The job runner runs as `ubuntu`, and starting a systemd unit needs root. The
sudoers file grants exactly two commands by full path — `systemctl start
flyto-slam.service` and `systemctl stop flyto-slam.service` — and nothing else.

That enumeration is the security property. `systemctl *` would hand a general
privilege-escalation primitive to anything that can reach the job runner, and
the entire reason the executor boundary exists is that a dispatched job must
not be able to become arbitrary host execution. Widening this file gives that
away quietly.

Run `visudo -c` after installing. A malformed file in `/etc/sudoers.d` can lock
sudo out of the machine, and this robot has no console attached.
