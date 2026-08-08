from __future__ import annotations

from pathlib import Path

import pytest

from flyto_robotics.recovery_install import (
    NETWORK_CONFIG,
    gadget_macs,
    install_recovery,
    update_cmdline,
    update_config,
)


def test_cmdline_update_is_idempotent_and_preserves_existing_modules():
    original = "console=serial0,115200 root=LABEL=writable modules-load=foo quiet\n"
    once = update_cmdline(original)

    assert "modules-load=foo,dwc2,g_ether" in once
    assert update_cmdline(once) == once
    assert len(once.splitlines()) == 1


def test_config_update_is_idempotent_and_scoped_to_all_boards():
    once = update_config("[pi4]\nmax_framebuffers=2\n")

    assert once.endswith("[all]\ndtoverlay=dwc2\n")
    assert update_config(once) == once

    scoped_elsewhere = update_config("[cm4]\ndtoverlay=dwc2\n")
    assert scoped_elsewhere.count("dtoverlay=dwc2") == 2
    assert scoped_elsewhere.endswith("[all]\ndtoverlay=dwc2\n")


def test_cmdline_refuses_a_competing_gadget_driver():
    with pytest.raises(ValueError, match="incompatible USB gadget"):
        update_cmdline("root=LABEL=writable modules-load=dwc2,g_serial\n")


def test_gadget_macs_are_stable_local_and_distinct():
    first = gadget_macs("machine-a")
    second = gadget_macs("machine-a")

    assert first == second
    assert first[0] != first[1]
    assert int(first[0].split(":")[0], 16) & 0x02


def test_install_writes_only_non_secret_recovery_state_and_is_repeatable(tmp_path):
    root = Path(__file__).resolve().parents[1]
    boot = tmp_path / "boot"
    boot.mkdir()
    cmdline = boot / "cmdline.txt"
    config = boot / "config.txt"
    cmdline.write_text("console=serial0,115200 root=LABEL=writable\n")
    config.write_text("[all]\nenable_uart=1\n")
    machine_id = tmp_path / "machine-id"
    machine_id.write_text("machine-a\n")
    calls = []

    kwargs = {
        "source_root": root,
        "robot_id": "flyto-tb3-lab-001",
        "cloud_url": "https://cloud.example.test",
        "cmdline_path": cmdline,
        "config_path": config,
        "machine_id_path": machine_id,
        "systemd_dir": tmp_path / "systemd",
        "network_dir": tmp_path / "network",
        "modprobe_dir": tmp_path / "modprobe",
        "environment_path": tmp_path / "flyto" / "robot-recovery.env",
        "run": calls.append,
    }
    install_recovery(**kwargs)
    first = {path: path.read_bytes() for path in tmp_path.rglob("*") if path.is_file()}
    install_recovery(**kwargs)
    second = {path: path.read_bytes() for path in tmp_path.rglob("*") if path.is_file()}

    assert first == second
    assert (tmp_path / "network" / "80-flyto-usb0.network").read_text() == NETWORK_CONFIG
    environment = (tmp_path / "flyto" / "robot-recovery.env").read_text().lower()
    assert "flyto-tb3-lab-001" in environment
    assert "password" not in environment
    assert "secret" not in environment
    assert cmdline.with_name("cmdline.txt.flyto-backup").exists()
    assert calls.count(["systemctl", "daemon-reload"]) == 2


def test_install_refuses_cloud_urls_with_credentials_or_paths(tmp_path):
    root = Path(__file__).resolve().parents[1]
    cmdline = tmp_path / "cmdline.txt"
    config = tmp_path / "config.txt"
    machine_id = tmp_path / "machine-id"
    cmdline.write_text("root=LABEL=writable\n")
    config.write_text("[all]\n")
    machine_id.write_text("machine-a\n")

    with pytest.raises(ValueError, match="must not include"):
        install_recovery(
            source_root=root,
            robot_id="robot-1",
            cloud_url="https://user:pass@example.test/private",
            cmdline_path=cmdline,
            config_path=config,
            machine_id_path=machine_id,
            systemd_dir=tmp_path / "systemd",
            network_dir=tmp_path / "network",
            modprobe_dir=tmp_path / "modprobe",
            environment_path=tmp_path / "flyto" / "robot-recovery.env",
        )


def test_deployment_units_keep_recovery_read_only_and_out_of_band():
    root = Path(__file__).resolve().parents[1]
    units = {
        path.name: path.read_text()
        for path in (root / "deploy" / "systemd").glob("flyto-*doctor*.*")
    }
    portal = (root / "deploy" / "systemd" / "flyto-recovery-portal.service").read_text()
    all_text = "\n".join([*units.values(), portal]).lower()

    assert "10.77.0.1" in portal
    assert "recovery_portal" in portal
    assert "robot_doctor" in all_text
    assert "password" not in all_text
    assert "device_secret" not in all_text
    assert "execstartpost" not in portal
    assert "ipmasquerade" not in NETWORK_CONFIG.lower()
    assert "ipforward" not in NETWORK_CONFIG.lower()
