from __future__ import annotations

import ast
import subprocess
import xml.etree.ElementTree as ET
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]


def test_lima_lab_is_arm64_low_memory_and_docker_free() -> None:
    config = (ROOT / "lima/flyto-robot-gazebo.yaml").read_text(
        encoding="utf-8"
    )
    assert "template:ubuntu-24.04" in config
    assert "vmType: vz" in config
    assert "arch: aarch64" in config
    assert "memory: 4GiB" in config
    assert "system: false" in config
    assert "user: false" in config
    assert "writable: false" in config
    assert "TURTLEBOT3_MODEL: burger" in config


def test_world_has_high_fidelity_physics_and_no_online_models() -> None:
    world_file = ROOT / "worlds/turtlebot3-fidelity.sdf"
    tree = ET.parse(world_file)
    world = tree.getroot().find("world")
    assert world is not None
    assert world.findtext("physics/max_step_size") == "0.001"
    assert world.findtext("physics/real_time_factor") == "1.0"
    assert world.findtext("physics/ode/solver/iters") == "150"
    assert world.findtext("gravity") == "0 0 -9.80665"
    assert world.find("plugin[@filename='gz-sim-sensors-system']") is not None
    source = world_file.read_text(encoding="utf-8")
    assert "http://" not in source
    assert "https://" not in source


def test_launch_uses_official_burger_and_flyto_fail_safe_topics() -> None:
    launch_file = ROOT / "launch/turtlebot3_fidelity.launch.py"
    source = launch_file.read_text(encoding="utf-8")
    ast.parse(source, filename=str(launch_file))
    for required in (
        "turtlebot3_gazebo",
        "turtlebot3_burger/model.sdf",
        '"/flyto/cmd_vel"',
        '"/flyto/raw_odom"',
        '"/flyto/odom"',
        '"/flyto/raw_scan"',
        '"/flyto/scan"',
        '"/flyto/imu"',
        '"/flyto/actuator_cmd_vel"',
        "ros2_sensor_guard",
        "ros2_safety_supervisor",
        "lidar_dropout",
        "odometry_freeze",
    ):
        assert required in source


def test_lima_scripts_are_valid_and_do_not_start_removed_runtimes() -> None:
    scripts = [
        ROOT / "scripts/provision-lima-gazebo.sh",
        ROOT / "scripts/run-lima-gazebo.sh",
        ROOT / "scripts/stop-lima-gazebo.sh",
        ROOT / "scripts/verify-lima-gazebo.sh",
    ]
    for script in scripts:
        subprocess.run(["bash", "-n", str(script)], check=True)
        source = script.read_text(encoding="utf-8").lower()
        assert "docker " not in source
        assert "colima" not in source
        assert "paperclip" not in source
    verify_source = scripts[-1].read_text(encoding="utf-8")
    assert "lidar_dropout" in verify_source
    assert "post_latch_zero_commands" in verify_source
    assert "stop_latency" in verify_source
    run_source = scripts[1].read_text(encoding="utf-8")
    assert "setsid --fork --wait" in run_source
    assert 'kill -TERM -- "-${managed_pgid}"' in run_source
    assert '"${runtime_root}/${name}.pgid"' in run_source
