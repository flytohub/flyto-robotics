"""Idempotent one-time installer for the card-free USB recovery channel."""

from __future__ import annotations

import argparse
import hashlib
import os
import re
import shutil
import subprocess
from collections.abc import Callable, Sequence
from pathlib import Path
from urllib.parse import urlparse

_IDENTIFIER = re.compile(r"^[A-Za-z0-9][A-Za-z0-9._:@/-]{0,127}$")
USB_ADDRESS = "10.77.0.1/24"
NETWORK_CONFIG = f"""[Match]
Name=usb0

[Network]
Address={USB_ADDRESS}
DHCPServer=yes
LinkLocalAddressing=yes
MulticastDNS=yes

[DHCPServer]
PoolOffset=10
PoolSize=20
EmitDNS=no
"""


def update_cmdline(value: str) -> str:
    lines = value.splitlines()
    if len(lines) != 1 or not lines[0].strip():
        raise ValueError("cmdline.txt must contain exactly one non-empty line")
    words = lines[0].split()
    module_indexes = [index for index, word in enumerate(words) if word.startswith("modules-load=")]
    if len(module_indexes) > 1:
        raise ValueError("cmdline.txt contains multiple modules-load entries")
    required = ["dwc2", "g_ether"]
    if module_indexes:
        index = module_indexes[0]
        modules = [item for item in words[index].split("=", 1)[1].split(",") if item]
        incompatible = sorted(
            item for item in modules if item.startswith("g_") and item != "g_ether"
        )
        if incompatible:
            raise ValueError(
                "cmdline.txt already loads an incompatible USB gadget: " + ", ".join(incompatible)
            )
        for item in required:
            if item not in modules:
                modules.append(item)
        words[index] = "modules-load=" + ",".join(modules)
    else:
        words.append("modules-load=" + ",".join(required))
    return " ".join(words) + "\n"


def update_config(value: str) -> str:
    section = ""
    for line in value.splitlines():
        stripped = line.strip()
        if stripped.startswith("[") and stripped.endswith("]"):
            section = stripped[1:-1].strip().lower()
        elif stripped == "dtoverlay=dwc2" and section in {"", "all"}:
            return value if value.endswith("\n") else value + "\n"
    suffix = "" if not value or value.endswith("\n") else "\n"
    return value + suffix + "\n[all]\ndtoverlay=dwc2\n"


def gadget_macs(machine_id: str) -> tuple[str, str]:
    if not machine_id.strip():
        raise ValueError("machine-id is empty")

    def derive(label: str) -> str:
        value = bytearray(hashlib.sha256(f"{machine_id}:{label}".encode()).digest()[:6])
        value[0] = (value[0] | 0x02) & 0xFE
        return ":".join(f"{item:02x}" for item in value)

    return derive("host"), derive("device")


def install_recovery(
    *,
    source_root: Path,
    robot_id: str,
    cloud_url: str,
    cmdline_path: Path,
    config_path: Path,
    machine_id_path: Path,
    systemd_dir: Path,
    network_dir: Path,
    modprobe_dir: Path,
    environment_path: Path,
    run: Callable[[Sequence[str]], None] | None = None,
) -> None:
    if not _IDENTIFIER.fullmatch(robot_id):
        raise ValueError("robot ID must be a safe namespaced identifier")
    if cloud_url:
        parsed = urlparse(cloud_url)
        if parsed.scheme not in {"http", "https"} or not parsed.hostname:
            raise ValueError("Cloud URL must be an HTTP(S) origin")
        if parsed.username or parsed.password or parsed.path not in {"", "/"} or parsed.query:
            raise ValueError("Cloud URL must not include credentials, path, or query")

    _rewrite_boot_file(cmdline_path, update_cmdline)
    _rewrite_boot_file(config_path, update_config)
    host_mac, device_mac = gadget_macs(machine_id_path.read_text(encoding="utf-8").strip())

    network_dir.mkdir(parents=True, exist_ok=True)
    modprobe_dir.mkdir(parents=True, exist_ok=True)
    systemd_dir.mkdir(parents=True, exist_ok=True)
    environment_path.parent.mkdir(parents=True, exist_ok=True)
    _write_if_changed(network_dir / "80-flyto-usb0.network", NETWORK_CONFIG, 0o644)
    _write_if_changed(
        modprobe_dir / "flyto-usb-recovery.conf",
        f"options g_ether host_addr={host_mac} dev_addr={device_mac}\n",
        0o644,
    )
    _write_if_changed(
        environment_path,
        f"FLYTO_ROBOT_RESOURCE_ID={robot_id}\nFLYTO_CLOUD_URL={cloud_url}\n",
        0o644,
    )
    for name in (
        "flyto-robot-doctor.service",
        "flyto-robot-doctor.timer",
        "flyto-recovery-portal.service",
    ):
        shutil.copy2(source_root / "deploy" / "systemd" / name, systemd_dir / name)

    if run is not None:
        run(["systemctl", "daemon-reload"])
        run(["systemctl", "enable", "ssh.service", "avahi-daemon.service"])
        run(
            [
                "systemctl",
                "enable",
                "flyto-robot-doctor.timer",
                "flyto-recovery-portal.service",
            ]
        )


def main(argv: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="Install permanent Flyto USB recovery")
    parser.add_argument("--robot-id", required=True)
    parser.add_argument("--cloud-url", default=os.getenv("FLYTO_CLOUD_URL", ""))
    parser.add_argument(
        "--source-root",
        type=Path,
        default=Path(__file__).resolve().parents[1],
    )
    parser.add_argument("--cmdline", type=Path, default=Path("/boot/firmware/cmdline.txt"))
    parser.add_argument("--config", type=Path, default=Path("/boot/firmware/config.txt"))
    args = parser.parse_args(argv)
    if os.geteuid() != 0:
        parser.error("run this one-time installer with sudo")

    def run(command: Sequence[str]) -> None:
        subprocess.run(list(command), check=True)

    install_recovery(
        source_root=args.source_root.resolve(),
        robot_id=args.robot_id,
        cloud_url=args.cloud_url,
        cmdline_path=args.cmdline,
        config_path=args.config,
        machine_id_path=Path("/etc/machine-id"),
        systemd_dir=Path("/etc/systemd/system"),
        network_dir=Path("/etc/systemd/network"),
        modprobe_dir=Path("/etc/modprobe.d"),
        environment_path=Path("/etc/flyto/robot-recovery.env"),
        run=run,
    )
    print("Flyto USB recovery installed. Reboot once to create usb0 at 10.77.0.1.")
    return 0


def _rewrite_boot_file(path: Path, transform: Callable[[str], str]) -> None:
    current = path.read_text(encoding="utf-8")
    updated = transform(current)
    backup = path.with_name(f"{path.name}.flyto-backup")
    if not backup.exists():
        shutil.copy2(path, backup)
    _write_if_changed(path, updated, path.stat().st_mode & 0o777)


def _write_if_changed(path: Path, value: str, mode: int) -> None:
    if path.exists() and path.read_text(encoding="utf-8") == value:
        return
    temporary = path.with_name(f".{path.name}.tmp")
    temporary.write_text(value, encoding="utf-8")
    temporary.chmod(mode)
    temporary.replace(path)


if __name__ == "__main__":
    raise SystemExit(main())
