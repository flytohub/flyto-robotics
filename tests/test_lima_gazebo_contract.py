from __future__ import annotations

import ast
import re
import subprocess
import xml.etree.ElementTree as ET
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
VERIFY_SCRIPT = ROOT / "scripts/verify-lima-gazebo.sh"
RESTORE_CALL = '"${repository_root}/scripts/run-lima-gazebo.sh"'
FIRST_START_CALL = f"{RESTORE_CALL} --no-gateway"
CLEANUP_CONTRACT_VERSION = "flyto.robotics.runtime-cleanup.v1"
PHYSICS_SETTLE_SOURCE = "gz_transport_world_pose_info_sim_time"


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


# ---------------------------------------------------------------------------
# Safety ordering / invariant coverage for verify-lima-gazebo.sh.
#
# These are static contract assertions over the script text. They do not start
# Lima, ROS 2, or Gazebo and prove nothing about a real simulation run; real
# acceptance is proven only by an actual Lima/Gazebo run of the script.
# ---------------------------------------------------------------------------


def _verify_source() -> str:
    return VERIFY_SCRIPT.read_text(encoding="utf-8")


def _normal_probe_source(source: str) -> str:
    """Return the in-guest normal-probe Python block."""
    marker = "python3 - <<'PY'\n"
    body = source[source.index(marker) + len(marker) :]
    return body[: body.index("\nPY\n")]


def test_normal_probe_waits_for_command_graph_before_sampling_or_motion() -> None:
    source = _verify_source()
    probe = _normal_probe_source(source)
    ast.parse(probe)

    # Fail closed on at least two /flyto/cmd_vel subscribers, under a bounded wait.
    assert re.search(r"COMMAND_GRAPH_MIN_SUBSCRIPTIONS\s*=\s*2", probe)
    assert re.search(r"COMMAND_GRAPH_READY_TIMEOUT_S\s*=\s*\d", probe)
    assert "get_subscription_count()" in probe
    assert re.search(r"observed\s*<\s*COMMAND_GRAPH_MIN_SUBSCRIPTIONS", probe)
    assert re.search(r"deadline\s*=\s*time\.monotonic\(\)\s*\+\s*timeout", probe)
    assert "time.monotonic() < deadline" in probe
    assert "raise CommandGraphError" in probe

    # Ordering: the gate precedes the ground-truth start sample, the odometry
    # start sample, and any commanded motion.
    gate = probe.index("command_subscriptions = wait_for_command_graph")
    world_start = probe.index("world_start_stamp, world_start = sample_ground_truth")
    odom_start = probe.index("start = probe.odom[-1].pose.pose.position")
    motion = probe.index("probe.publisher.publish(command)")
    assert gate < world_start
    assert gate < odom_start
    assert gate < motion

    # Readiness is preserved as evidence and re-checked by the host aggregator.
    assert '"command_subscription_count": command_subscriptions' in probe
    assert re.search(r"EXPECTED_MIN_COMMAND_SUBSCRIPTIONS\s*=\s*2", source)
    assert '"command_subscription_count",' in source


def test_normal_probe_settles_gazebo_physics_before_sampling_or_motion() -> None:
    source = _verify_source()
    probe = _normal_probe_source(source)

    # A bounded settle window expressed in Gazebo simulation time, with a small
    # chassis stability window. Fail closed on either.
    assert re.search(r"PHYSICS_SETTLE_MIN_SIM_S\s*=\s*10\.0", probe)
    assert re.search(r"PHYSICS_SETTLE_MAX_DRIFT_M\s*=\s*0\.01", probe)
    assert re.search(r"PHYSICS_SETTLE_TIMEOUT_S\s*=\s*\d", probe)  # bounded
    assert "class PhysicsSettleError" in probe
    assert probe.count("raise PhysicsSettleError") >= 3
    assert "time.monotonic() + PHYSICS_SETTLE_TIMEOUT_S" in probe

    # Settle is measured from world pose stamps, i.e. simulation time and world
    # ground truth -- never odometry.
    assert "start_stamp, start_position = sample_ground_truth(topic)" in probe
    assert "advance = stamp - start_stamp" in probe
    assert re.search(r"advance\s*>=\s*PHYSICS_SETTLE_MIN_SIM_S", probe)
    assert re.search(r"max_drift\s*>\s*PHYSICS_SETTLE_MAX_DRIFT_M", probe)
    assert "settle_physics(probe, ground_truth_topic)" in probe
    settle_body = probe[probe.index("def settle_physics") : probe.index("rclpy.init()")]
    assert "probe.odom" not in settle_body

    # Ordering: graph readiness, then physics settle, and only then the world
    # start sample, the odometry start sample, and the commanded motion.
    gate = probe.index("command_subscriptions = wait_for_command_graph")
    settle = probe.index("settle_sim_seconds, settle_max_drift = settle_physics")
    world_start = probe.index("world_start_stamp, world_start = sample_ground_truth")
    odom_start = probe.index("start = probe.odom[-1].pose.pose.position")
    motion = probe.index("probe.publisher.publish(command)")
    assert gate < settle < world_start
    assert settle < odom_start
    assert settle < motion

    # Settle duration and drift are preserved as evidence and validated host side.
    assert '"gazebo_physics_settle_sim_seconds": settle_sim_seconds' in probe
    assert '"gazebo_physics_settle_max_drift_m": settle_max_drift' in probe
    for key in (
        '"gazebo_physics_settle_sim_seconds",',
        '"gazebo_physics_settle_max_drift_m",',
    ):
        assert key in source
    # The settle evidence names its exact provenance: Gazebo world-pose
    # simulation time, checked for equality host side.
    assert re.search(
        rf'PHYSICS_SETTLE_SOURCE\s*=\s*"{re.escape(PHYSICS_SETTLE_SOURCE)}"', probe
    )
    assert '"gazebo_physics_settle_source": PHYSICS_SETTLE_SOURCE' in probe
    assert '"gazebo_physics_settle_source",' in source
    assert re.search(
        rf'EXPECTED_PHYSICS_SETTLE_SOURCE\s*=\s*"{re.escape(PHYSICS_SETTLE_SOURCE)}"',
        source,
    )
    assert (
        '("gazebo_physics_settle_source", EXPECTED_PHYSICS_SETTLE_SOURCE),' in source
    )
    assert re.search(r"settle_seconds\s*<\s*PHYSICS_SETTLE_MIN_SIM_S", source)
    assert re.search(
        r"0\.0\s*<=\s*settle_drift\s*<=\s*PHYSICS_SETTLE_MAX_DRIFT_M", source
    )

    # The settle gate must not widen the motion window or replace ground truth.
    assert "motion_deadline = time.monotonic() + 1.25" in probe
    assert "assert 0.025 <= odometry_displacement <= 0.16" in probe
    assert re.search(r"GROUND_TRUTH_MIN_DISPLACEMENT_M\s*=\s*0\.02", probe)
    assert re.search(r"GROUND_TRUTH_MAX_DISPLACEMENT_M\s*=\s*0\.30", probe)
    assert "ground_truth_displacement = math.hypot(" in probe


def test_verify_restores_normal_gateway_runtime_exactly_once_via_exit_trap() -> None:
    source = _verify_source()
    lines = source.splitlines()

    # A single EXIT trap owns restoration, with signal paths routed through it.
    assert "trap restore_runtime EXIT" in source
    assert re.search(r"trap\s+'exit 130'\s+INT", source)
    assert re.search(r"trap\s+'exit 143'\s+TERM", source)
    assert "cleanup_entered" in source  # re-entrancy guard

    # Exactly one bare (normal, non-fault, gateway) restoration call exists...
    restore_lines = [i for i, line in enumerate(lines) if line.strip() == RESTORE_CALL]
    assert len(restore_lines) == 1, restore_lines

    # ...and it lives inside the handler, i.e. before the trap is installed,
    # so it cannot also run inline on the success path.
    trap_line = next(
        i for i, line in enumerate(lines) if line.strip() == "trap restore_runtime EXIT"
    )
    assert restore_lines[0] < trap_line

    # The EXIT trap is armed before the first --no-gateway runtime start, so no
    # window exists where the runtime is degraded without a restoration owner.
    first_start = next(
        i for i, line in enumerate(lines) if line.strip() == FIRST_START_CALL
    )
    assert trap_line < first_start

    # No duplicate restoration after report aggregation.
    summary_line = next(
        i for i, line in enumerate(lines) if "Virtual robot verification" in line
    )
    assert all(
        "run-lima-gazebo.sh" not in line for line in lines[summary_line:]
    ), "restoration must not be repeated after aggregation"
    assert restore_lines[0] < summary_line


def test_cleanup_preserves_failure_status_and_records_bounded_evidence() -> None:
    source = _verify_source()
    start = source.index("restore_runtime() {")
    handler = source[start : source.index("\n}\n", start)]

    # Original verification status is captured first and re-raised at the end.
    assert re.search(r"local status=\$\?", handler)
    assert 'exit "${status}"' in handler
    # Restoration failure fails an otherwise successful run...
    assert 'status="${restore_status}"' in handler
    # ...but never overwrites an existing nonzero verification status.
    assert "preserving" in handler

    # Zero command is best effort and must not abort cleanup.
    assert "ros2 topic pub --once /flyto/cmd_vel" in handler
    assert "set +e" in handler

    # Cleanup evidence is written, and bounded to this run's result directory.
    assert '"${result_directory}/cleanup.json"' in handler
    for key in (
        "contract_version",
        "passed",
        "restored_normal_gateway_runtime",
        "normal_runtime_restoration_exit_code",
        "zero_command_published",
        "verification_status",
    ):
        assert key in handler

    # The numeric restoration exit code is recorded verbatim, and is exactly the
    # value that drives the boolean restore verdict: 0 on success, nonzero on
    # failure. Nonzero also fails an otherwise-successful run.
    assert '"normal_runtime_restoration_exit_code": %s' in handler
    assert '"${restore_status}"' in handler
    # The 0-on-success / nonzero-on-failure meaning is documented in place.
    assert "normal_runtime_restoration_exit_code is 0 when" in handler
    assert "nonzero when restoration failed" in handler
    assert "local restore_status=$?" in handler
    # 0 => restored true; nonzero => restored false...
    assert re.search(
        r'\[\[ "\$\{restore_status\}" -eq 0 \]\]\s*\|\|\s*restored=false', handler
    )
    assert re.search(r'\[\[ "\$\{restore_status\}" -ne 0 \]\]', handler)
    # ...and a nonzero code fails an otherwise-successful verification run.
    assert 'status="${restore_status}"' in handler

    # The cleanup evidence carries an exact, versioned contract identity, and
    # that identity is immutable for the lifetime of the run.
    assert re.search(
        rf'readonly\s+cleanup_contract_version='
        rf'"{re.escape(CLEANUP_CONTRACT_VERSION)}"',
        source,
    )
    assert '"${cleanup_contract_version}"' in handler
