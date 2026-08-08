# The robot loses the network, and how to get back in

Owner: claude
Branch: main
Date: 2026-08-08

## What happened

The lab router was replaced (TP-Link → Hitron). The robot's stored networks no
longer existed, so it never rejoined, and with no network there was no SSH — the
usual way in was gone at the same moment as the thing it was needed for.

Not resolved in this session. What follows is the ground truth gathered while
trying, so the next attempt starts from measurement instead of guesswork.

## The first thing to know: the cloud half does not care

`deploy/flyto_job_runner.py` **dials out**. It polls
`GET /api/devices/jobs/poll` against `FLYTO_CLOUD_URL`, sends heartbeats, and
claims and completes jobs. Nothing dials in.

So a changed IP, a changed SSID, a changed router — none of it breaks
cloud-to-robot dispatch. The robot reappears on its own the moment it has
internet, with nothing to reconfigure on the cloud side. **Only the operator's
own SSH access breaks.** Do not go looking for a cloud-side fix.

## The hardware, measured rather than assumed

| | |
|---|---|
| Board | Raspberry Pi 4 Model B Rev 1.5 (`c03115`) |
| Wi-Fi | Broadcom `BCM4345/6` (CYW43455), **dual band** |
| Firmware | `7.45.234`, loads cleanly via `brcmfmac` |
| Hostname | `flyto-robot`, `avahi-daemon` installed → `flyto-robot.local` |

**The band is not the problem, and two rounds were wasted assuming it might
be.** A Pi 4 does 5 GHz. Confirm the board before theorising about radios: the
image on the card supports rpi-2 through rpi-4 and Zero 2, so the presence of a
`.dtb` proves nothing.

## The card, and what a Mac can do with it

Two partitions. `system-boot` is FAT32 and mounts on macOS read-write;
the rest is ext4 and macOS cannot read it at all (no ext4fuse, fuse-ext2 or
lklfuse installed, and none is needed if you use the boot partition properly).

So **everything must go through the FAT partition**: `network-config`,
`user-data`, `meta-data`, `config.txt`, `cmdline.txt`. Editing
`/etc/netplan/*.yaml` directly is not an option from a Mac.

### The one trick that makes card edits work at all

cloud-init applies `network-config` **once per instance**. On an
already-provisioned system it ignores your edit entirely unless the
`instance-id` in `meta-data` changes. Bump it every time:

```
instance-id: flyto-robot-<what-changed>-<date>
```

Verified working: `applied instance-id` in the diagnostic came back as the new
value, and all four configured SSIDs appeared in `/etc/netplan/50-cloud-init.yaml`.

Re-running is safe on this image: `ssh_deletekeys: false` (host keys survive, so
no host-key warnings), the `users` block only re-asserts the same authorized
keys, and there is no `runcmd`/`bootcmd` doing anything destructive.

## Ways in that do not need Wi-Fi

Checked on the card, all present already:

- **Ethernet.** `eth0: dhcp4: true` is in `network-config` and predates any of
  this, so a cable works regardless of whether cloud-init re-ran. The simplest
  guaranteed route.
- **Serial console.** `config.txt` already has `enable_uart=1` and
  `cmdline.txt` starts with `console=serial0,115200`. A USB-TTL adapter on
  GPIO 6/8/10 gives a login prompt with no configuration change at all —
  **except** that `user-data` sets `lock_passwd: true`, so the `ubuntu` account
  has no usable console password. Unlocking it is a card edit.
- **USB gadget.** `dtoverlay=dwc2` is in the `[all]` section, so OTG is enabled
  on every board (`dr_mode=host` at the end is `[cm4]`-only and does not apply).
  Adding `modules-load=dwc2,g_ether` to `cmdline.txt` gives SSH over a USB cable
  using the existing key — no password, no LAN. Caveat: the same port carries
  power, and a Pi 4 may draw more than a Mac port supplies.
- **Phone hotspot with an old SSID.** The stored networks are still in
  `network-config`; recreating one on a phone makes the robot join by itself.
  Needs no disassembly and was the cheapest option not taken.

## Where it actually stands

After the config was confirmed applied — all four SSIDs present in netplan,
correct `instance-id`, radio firmware loaded:

```
eth0    DOWN
wlan0   DORMANT
```

and `wpa_supplicant` logged exactly three lines: starting, "Successfully
initialized", started. **No scan, no association attempt, no authentication
failure.** The supplicant is running and has not been asked to connect to
anything.

That points away from the passphrase and away from the radio, and towards
netplan's generated per-interface supplicant service not being started —
`wpa_supplicant.service` is up, which on a netplan system does nothing by
itself; the interface is driven by `netplan-wpa-wlan0.service` off
`/run/netplan/wpa-wlan0.conf`. Next attempt should check exactly that, and try
an explicit `netplan apply`.

Also unexplained and worth reading first:

```
WARNING: cloud-config failed schema validation!
```

with `extended_status: degraded running`. A schema failure can make cloud-init
skip modules, so it may be upstream of everything above. `cloud-init schema
--system` prints the detail.

## What the diagnostic got wrong

A one-shot script was written to `/boot/firmware/flyto-diag.txt` so the answers
could be read off the FAT partition with no network. The idea works — the file
came back — but two mistakes made half of it useless, and both are easy to
repeat:

**It used tools that are not installed.** `iw` and `rfkill` are absent on this
image, so the radio and scan sections were empty. Worse, the channel-count
section printed `2.4GHz: 0` and `5GHz: 0`, which reads as a measurement and was
nothing of the kind. Use `wpa_cli`, `systemctl`, `journalctl` and
`/sys/class/net/`, which are always there.

**It ran too early.** `runcmd` fired ~25 s into boot while `cloud-init status`
still said `running`, so the network state captured was mid-provisioning. Wait
for `cloud-init status --wait` before measuring anything.

## Do not repeat

- Guessing SSIDs. Two rounds went into `65N` vs `65N-5G` vs `65N-2.4G`, each
  costing a card pull, a boot and a wait. Ask for the exact name, add every
  candidate at once, or measure from inside.
- Reading "the config is present" as "the config took effect". Netplan had all
  four networks while `wlan0` sat dormant.
- Reading "other devices connect fine" as evidence about the robot. It proves
  the network exists and nothing about the robot's side.
- Assuming a band problem before checking `/proc/device-tree/model`.
