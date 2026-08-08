# One-time robot recovery installation

The storage card is an installation medium, not an operating interface. After
this is installed once, a router, SSID, DHCP, DNS, Cloud, or robot-service
failure can be diagnosed without opening the robot or removing its SSD/microSD.

## What is installed

```text
normal path
robot → outbound HTTPS → Flyto Cloud generic resource telemetry

recovery path (works without Wi-Fi, router, or internet)
Mac/PC ── USB data cable ── usb0 10.77.0.1
                            ├── read-only diagnosis page :8770
                            └── existing key-only SSH :22
```

The USB link uses Raspberry Pi gadget Ethernet and systemd-networkd's local
DHCP server. It does not route the computer through the robot and does not open
a new actuator API. The portal is read-only and contains reason/action codes,
not SSIDs, IP addresses, credentials, or raw logs.

The doctor refreshes every minute and writes:

- `/var/lib/flyto-robot/diagnostics/latest.json` — current state;
- `/var/lib/flyto-robot/diagnostics/last-failure.json` — most recent failure,
  preserved after recovery.

Both are `flyto.resource-telemetry.v1` envelopes. A generic resource publisher
can upload them as `system.diagnostics` and `system.last_failure`; Cloud does
not need a TurtleBot-specific data model.

## Install once

Run on the robot from the deployed repository:

```bash
sudo ./scripts/install-robot-recovery.sh \
  --robot-id flyto-tb3-lab-001 \
  --cloud-url https://your-cloud-origin.example
sudo reboot
```

The installer is idempotent. It preserves a first-run
`cmdline.txt.flyto-backup` and `config.txt.flyto-backup`, derives stable local
USB MAC addresses from `/etc/machine-id`, enables the existing key-only SSH and
Avahi services, and installs the doctor timer and portal. It never accepts or
writes a Wi-Fi password or device secret.

The first reboot is required because the kernel must load `dwc2,g_ether`.
Future software updates and diagnostics use SSH or the normal outbound update
channel; they do not require another card edit.

## Diagnose a disconnected robot

1. Keep the robot powered normally.
2. Connect a USB **data** cable to the Pi 4 USB-C port. If another source is
   already feeding 5 V, use an appropriate USB power blocker/data-only adapter
   or follow the hardware power plan; do not unintentionally double-feed it.
3. Open <http://10.77.0.1:8770> or run:

```bash
curl http://10.77.0.1:8770/v1/diagnostics
ssh ubuntu@10.77.0.1
```

The same existing SSH public key is used. No console password is enabled.

Stable primary reasons include:

| Reason | Meaning |
|---|---|
| `provisioning_degraded` | cloud-init is degraded/error before networking closes |
| `wifi_not_associated` | supplicant has not joined a configured network |
| `wifi_no_address` | association succeeded but DHCP did not |
| `default_route_missing` | local address exists without an uplink route |
| `dns_unavailable` | route exists but name resolution failed |
| `cloud_unreachable` | local network works but the configured Cloud origin does not |
| `robot_service_unhealthy` | network is healthy; a required robot service is not active |

The page reports action codes. An operator can then use SSH to inspect the
specific journal, validate cloud-init, or apply an approved network change.
The portal intentionally does not accept Wi-Fi credentials or service-control
requests.

## Why Cloud alone cannot solve Wi-Fi loss

The robot's control and telemetry paths dial out. If the robot has no network,
Cloud cannot ask it why it has no network. A permanent out-of-band path is
therefore part of installation, not a Cloud retry policy. Once connectivity
returns, `system.last_failure` preserves the preceding reason for remote
review.

## Rollback

The recovery surface is additive. Restore the two `.flyto-backup` boot files,
remove the three `flyto-*recovery*`/doctor units plus
`80-flyto-usb0.network` and `flyto-usb-recovery.conf`, reload systemd, and
reboot. Robot motion, job dispatch, and mission contracts are independent.

