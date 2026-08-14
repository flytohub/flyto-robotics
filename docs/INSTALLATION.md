# Install and operate Flyto2 Robotics

This page is the customer-facing procedure for installing the supplied Python
wheel on a systemd-based robot, rehearsing the operation without touching the
host, and operating or recovering an installed release. It does not require a
source checkout after the wheel has been copied to the device.

The examples use synthetic version `1.4.0` and local paths. They contain no
credentials or production endpoints.

## Choose a profile

`generic` is the default and is middleware- and vendor-neutral. It installs the
resource agent and periodic doctor only. It does not require ROS, TurtleBot, or
another robot vendor's runtime.

`camera` is a provider-neutral additive profile. It extends `generic`, keeps
every generic unit byte-identical, and owns `flyto-camera-gateway.service`.
`ros2` extends `camera` and adds only `flyto-robot-ros2.service`; the inherited
generic and camera units remain byte-identical. Every owned unit is written,
enabled, restarted, and verified active by the same transactional activation.
Select `ros2` only on a device whose ROS 2 adapter and environment have already
been commissioned. A future site profile
may extend either profile through a separately reviewed registry; a job cannot
select or weaken the installation profile.

## Install the supplied artifact

Create a dedicated virtual environment, install the wheel you received, and
confirm that both customer commands came from that environment:

```bash
sudo python3 -m venv /opt/flyto-robot/venv
sudo /opt/flyto-robot/venv/bin/python -m pip install ./flyto_robotics-0.1.0-py3-none-any.whl
/opt/flyto-robot/venv/bin/flyto-robot --help
/opt/flyto-robot/venv/bin/flyto-device-events --help
/opt/flyto-robot/venv/bin/flyto-camera-gateway --check-settings
```

First rehearse against a temporary filesystem root. This records systemd
operations without reloading or starting host services:

```bash
rehearsal_root="$(mktemp -d)"
/opt/flyto-robot/venv/bin/flyto-robot \
  --root "$rehearsal_root" install \
  --from-package --version 1.4.0 --profile generic --dry-run
```

Inspect the single JSON result. A successful rehearsal has `"ok": true` and
`"dry_run": true`; retain its stable `reason` and `action_code` with the change
record. Remove the temporary directory when its output is no longer needed.

Install on the real device only after the rehearsal and site safety review:

```bash
sudo /opt/flyto-robot/venv/bin/flyto-robot install \
  --from-package --version 1.4.0 --profile generic \
  --python /opt/flyto-robot/venv/bin/python
```

If and only if this is the first Flyto activation and the selected profile's
unit names already belong to a reviewed legacy installation, create a bounded
takeover receipt without changing the machine:

```bash
sudo /opt/flyto-robot/venv/bin/flyto-robot plan-takeover \
  --profile generic --acknowledge-legacy-takeover \
  > /tmp/flyto-takeover-receipt.json
sudo /opt/flyto-robot/venv/bin/flyto-robot revalidate-takeover \
  --profile generic --receipt /tmp/flyto-takeover-receipt.json
```

After reviewing the receipt and the unchanged machine, attach that exact file
to the first landing install:

```bash
sudo /opt/flyto-robot/venv/bin/flyto-robot install \
  --from-package --version 1.4.0 --profile generic \
  --python /opt/flyto-robot/venv/bin/python \
  --takeover-receipt /tmp/flyto-takeover-receipt.json
```

The receipt contains only digests and generations, not unit bytes, credentials,
or endpoints. It is rejected if the host, commissioning prerequisites,
lifecycle state, unit bytes, or systemd state changed after planning. The
option is intentionally absent from `update`, and is refused with `--dry-run`
or after any activation has committed. A normal first install without a receipt
continues to refuse colliding unit names. Once sealed, any pre-commit failure
restores the captured legacy unit bytes and active/enabled state. Immediately
before the lifecycle state commit, a private durable intent binds the exact
incoming activation; after a response loss, recovery finalizes only that exact
committed activation and never restores legacy units beneath it. If either
outcome cannot be proved, the durable marker remains and later lifecycle
mutations fail closed.

Use `--profile camera` for the robot-local camera observation gateway without
the ROS bringup unit, or `--profile ros2` when both are intended. Configure the
gateway in `/etc/flyto-robot/camera.env`. `FLYTO_CAMERA_PROVIDER` is either
`ros_image` (the default) or `avfoundation`. For ROS, set
`FLYTO_CAMERA_TOPIC` (default `/camera/image_raw`). For macOS AVFoundation, set
an ffmpeg input in `FLYTO_CAMERA_DEVICE` and run the installed
`flyto-camera-gateway` console command because systemd is Linux-only. ffmpeg is
optional and is neither imported nor required for settings validation.

Camera capture defaults to `FLYTO_CAMERA_WIDTH=1280`,
`FLYTO_CAMERA_HEIGHT=720`, and `FLYTO_CAMERA_FRAMERATE=10`. Width and height
must each be an integer from 1 through 8192; framerate must be a finite decimal
greater than zero and no greater than 120. The gateway validates these bounded
operator settings for every provider. AVFoundation uses the same validated
values for both its one-shot availability check and its long-lived capture;
the ROS provider carries them as configuration but does not apply them to the
ROS topic.

Set the operator-assigned `FLYTO_CAMERA_SOURCE_ID` to a bounded label of at
most 64 letters, digits, dots, underscores, or hyphens. Rotation is restricted
to `FLYTO_CAMERA_ROTATION=0|90|180|270`, and flip to
`FLYTO_CAMERA_FLIP=none|horizontal|vertical|both`. Also set the bounded
`FLYTO_CAMERA_ZONE`; workflows cannot select the source, transforms, topic, or
HTTP listener. `FLYTO_CAMERA_BIND` accepts only a literal IPv4
loopback address and defaults to `127.0.0.1`. The only observation route is
the fixed `GET http://127.0.0.1:9000/api/spaces/zone-camera/observation`.
`FLYTO_CAMERA_PORT` is operator-only
configuration and defaults to `9000`, matching the canonical vision consumer;
workflow or job data cannot override it.

The camera unit permits only `AF_INET`, `AF_INET6`, `AF_NETLINK`, and `AF_UNIX`:
IPv4 is required for the loopback HTTP listener and DDS, IPv4/IPv6 cover ROS 2
DDS transports, netlink permits middleware interface discovery, and Unix
sockets support the local runtime. It does not use `PrivateNetwork`, `IPAddressDeny`,
or syscall filters that would isolate DDS. The remaining hardening keeps a
read-only system, private temporary directory, protected home/kernel/control
group state, and denies privilege elevation, SUID/SGID, personality changes,
and writable executable memory. Bind validation independently remains literal
IPv4 loopback-only; this permission does not make the HTTP API externally
reachable.

The gateway retains frame acceptance time only. It emits no pixels, raw device
identity, serial, device path, or content hash. The bounded host inventory is
only XC-TECH vendor `0x5843`, product `0x7884`, with `1280x720@10fps` metadata;
that inventory is not deployment or operational proof. A read-only macOS
check established only that direct AVFoundation null-output capture can use
`1280x720` at 10 fps with the configured orientation; automated validation
does not probe device capabilities. No Pi camera or ROS image topic has been
observed, and no deployment was performed.

This producer/lifecycle closure is not the full Cloud-device `vision.observe`
path. `deploy/flyto_job_runner.py` still rejects non-robotics work and is a
delivery-only executor; the generic installed executor protocol remains
required before Cloud-device vision dispatch exists.
The command is non-interactive, emits exactly one JSON document, and returns
non-zero unless activation and readiness verification succeed. On failure,
follow its stable `reason` and `action_code`; do not infer success from a unit
that is merely starting.

## Daily operations

Status reports the committed activation, releases, unit health, and next
action without changing the device:

```bash
sudo /opt/flyto-robot/venv/bin/flyto-robot status
```

Readiness is established only when the command exits zero and its one JSON
document has `"ok": true`, `"reason": "ok"`, and `"action_code": "none"`.
Record those fields as the stable operational evidence; do not substitute a
human reading of transient systemd output.

An update stages an immutable release, activates it, and verifies readiness.
If activation fails, the lifecycle restores the previous activation before
returning failure:

```bash
sudo /opt/flyto-robot/venv/bin/flyto-robot update \
  --from-package --version 1.5.0 --profile generic \
  --python /opt/flyto-robot/venv/bin/python
```

Rollback replays the exact prior activation snapshot, including its recorded
interpreter, profile, rendered units, unit policy, and readiness contract:

```bash
sudo /opt/flyto-robot/venv/bin/flyto-robot rollback
sudo /opt/flyto-robot/venv/bin/flyto-robot status
```

Do not edit the `current` symlink, activation snapshots, or rendered systemd
units by hand. The lifecycle owns them as one transaction. An install also
writes `/etc/flyto-robot/README-runbook.txt`, a durable local copy of the core
operator commands.

## Support and device events

Create a redacted, bounded support bundle before changing a failed device:

```bash
sudo /opt/flyto-robot/venv/bin/flyto-robot support-bundle \
  --output /tmp/flyto-support-bundle.json \
  --note "synthetic lab device failed its readiness check"
```

The bundle describes lifecycle state and log file metadata; it does not copy
credentials or raw logs. Review it before transferring it outside the device.

Export a bounded page of the validated device-event journal without granting
another process direct access to that file:

```bash
sudo /opt/flyto-robot/venv/bin/flyto-device-events export \
  --journal /var/lib/flyto-robot --limit 100
```

Use the returned cursor for the next page as described by
`flyto-device-events export --help`. Keep each requested `--limit` bounded;
optionally add the documented `--max-bytes` bound. The command performs no
network access and prints fixed, content-free errors when a journal cannot be
exported safely.

## Acceptance boundary

This runbook and its automated checks establish contracts, installed-wheel
command availability, transactional lifecycle behavior, and simulation proof.
They do not establish physical cold-boot repeatability, distance calibration,
hardware E-stop behavior, real sensor-loss response, camera operation, device
deployment, artifact publication, or customer-site acceptance. Each requires a
separately authorized physical or release procedure and its own evidence.

For a disconnected Raspberry Pi recovery path, see
[One-time recovery](ONE_TIME_RECOVERY.md). For Gazebo and development flows,
see [Running the demo](DEMO.md).
