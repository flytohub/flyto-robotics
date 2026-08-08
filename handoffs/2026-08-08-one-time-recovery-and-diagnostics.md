# One-time recovery and persistent diagnostics

Owner: codex
Branch: codex/workflow-space-war-room-20260808
Date: 2026-08-08

## Outcome

The router-loss handoff was pulled from `main` and converted into a permanent
installation boundary. Routine network diagnosis no longer depends on editing
the FAT boot partition or removing the robot's SSD/microSD.

One idempotent installer now prepares:

- Pi USB gadget Ethernet at `10.77.0.1/24` with local DHCP;
- stable locally administered gadget MAC addresses derived from machine-id;
- existing key-only SSH and Avahi, without enabling a console password;
- a one-minute observation-only doctor;
- a read-only USB-bound diagnostic portal;
- current and last-failure generic resource telemetry snapshots.

## Trust boundary

No Wi-Fi password, pairing secret, IP address, SSID, raw journal, service
control, job dispatch, or actuator command is exposed by the diagnostic
contract or portal. The only HTTP methods with behavior are read-only GETs.
The portal and SSH are an out-of-band local route; outbound Cloud behavior is
unchanged.

## Verification

- Pure reason ordering, telemetry hashing/privacy, recovery persistence,
  portal escaping, boot-file transforms, stable MAC derivation, idempotent
  sandbox installation, unit boundaries, and unsafe Cloud origins passed.
- Full `make verify`: 379 passed, 3 skipped; asset and deterministic mission
  checks passed.
- Local HTTP smoke: health and diagnostics returned 200, HTML returned 200,
  and POST was rejected with 405.
- Strict flyto-indexer verification: 18/18 passed with no secret or taint
  finding; unstaged impact found no high/moderate-risk symbol.
- Physical Pi reboot, USB enumeration, Mac DHCP, portal, and SSH remain the
  required hardware acceptance run; host tests do not claim that evidence.

## Operator path

See `docs/ONE_TIME_RECOVERY.md`. Install once, reboot once, then connect a USB
data cable and open `http://10.77.0.1:8770` whenever normal networking is lost.
