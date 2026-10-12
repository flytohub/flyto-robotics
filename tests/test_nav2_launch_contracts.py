from __future__ import annotations

import ast
import xml.etree.ElementTree as ET
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
LAUNCH_PATH = ROOT / "launch" / "nav2_closed_loop.launch.py"


def _tree() -> ast.Module:
    return ast.parse(LAUNCH_PATH.read_text(encoding="utf-8"), filename=str(LAUNCH_PATH))


def _calls(tree: ast.AST, function_name: str) -> list[ast.Call]:
    return [
        node
        for node in ast.walk(tree)
        if isinstance(node, ast.Call)
        and isinstance(node.func, ast.Name)
        and node.func.id == function_name
    ]


def _keyword(call: ast.Call, name: str) -> ast.expr | None:
    return next((item.value for item in call.keywords if item.arg == name), None)


def _literal_keyword(call: ast.Call, name: str) -> object | None:
    value = _keyword(call, name)
    try:
        return ast.literal_eval(value) if value is not None else None
    except (ValueError, TypeError):
        return None


def _assignment(tree: ast.AST, name: str) -> ast.expr:
    for node in ast.walk(tree):
        if not isinstance(node, (ast.Assign, ast.AnnAssign)):
            continue
        targets = node.targets if isinstance(node, ast.Assign) else [node.target]
        if any(isinstance(target, ast.Name) and target.id == name for target in targets):
            assert node.value is not None
            return node.value
    raise AssertionError(f"missing assignment for {name}")


def _assigned_call(tree: ast.AST, name: str, function_name: str) -> ast.Call:
    value = _assignment(tree, name)
    assert isinstance(value, ast.Call)
    assert isinstance(value.func, ast.Name) and value.func.id == function_name
    return value


def _name(value: ast.expr | None) -> str | None:
    return value.id if isinstance(value, ast.Name) else None


def _name_list(value: ast.expr | None) -> list[str]:
    assert isinstance(value, ast.List)
    names = [_name(item) for item in value.elts]
    assert all(name is not None for name in names)
    return [name for name in names if name is not None]


def test_closed_loop_physics_keeps_controller_and_lidar_margin() -> None:
    world = ET.parse(ROOT / "worlds" / "nav2-closed-loop.sdf").getroot()
    model = ET.parse(ROOT / "models" / "flyto_rover" / "model.sdf").getroot()
    max_step = float(world.findtext(".//physics/max_step_size", default="nan"))
    real_time_factor = float(world.findtext(".//physics/real_time_factor", default="nan"))
    lidar_rate = float(
        model.findtext(".//sensor[@name='front_lidar']/update_rate", default="nan")
    )
    params = (ROOT / "config" / "nav2_params.yaml").read_text(encoding="utf-8")

    assert max_step == 0.01
    assert real_time_factor == 1.0
    assert lidar_rate == 10.0
    assert "controller_frequency: 20.0" in params
    assert 1.0 / max_step >= 5 * 20.0
    assert 1.0 / max_step >= 10 * lidar_rate


def test_optional_route_camera_is_not_bridged_during_nav2_endurance() -> None:
    model = ET.parse(ROOT / "models" / "flyto_rover" / "model.sdf").getroot()
    camera = model.find(".//sensor[@name='route_camera']")
    assert camera is not None
    assert camera.findtext("topic") == "/flyto/camera/image"
    assert camera.findtext("update_rate") == "15"
    assert camera.findtext("always_on") == "false"
    assert camera.findtext("visualize") == "false"

    arguments = _keyword(_assigned_call(_tree(), "bridge", "Node"), "arguments")
    assert isinstance(arguments, ast.List)
    bridge_topics = [
        item.value
        for item in arguments.elts
        if isinstance(item, ast.Constant) and isinstance(item.value, str)
    ]
    assert all("/flyto/camera/image" not in topic for topic in bridge_topics)


def test_showcase_bridge_guard_preserves_the_full_bridge_configuration() -> None:
    launch_path = ROOT / "launch" / "gazebo_lab.launch.py"
    tree = ast.parse(launch_path.read_text(encoding="utf-8"), filename=str(launch_path))
    guards = [
        call
        for call in _calls(tree, "Node")
        if _literal_keyword(call, "executable") == "parameter_bridge_guard"
    ]
    assert len(guards) == 1
    guard = guards[0]

    assert _literal_keyword(guard, "package") == "flyto_robotics"
    assert _literal_keyword(guard, "executable") == "parameter_bridge_guard"
    assert _literal_keyword(guard, "ros_arguments") == ["--disable-rosout-logs"]
    assert _literal_keyword(guard, "output") == "screen"
    parameters = _keyword(guard, "parameters")
    assert parameters is not None
    assert "config/bridge.yaml" in ast.unparse(parameters)


def test_lifecycle_managers_start_after_their_managed_nodes() -> None:
    tree = _tree()
    map_delay = _assigned_call(tree, "delayed_map_manager", "TimerAction")
    navigation_delay = _assigned_call(tree, "delayed_navigation_manager", "TimerAction")
    lab_delay = _assigned_call(tree, "delayed_lab", "TimerAction")

    assert _literal_keyword(map_delay, "period") == 2.0
    assert _literal_keyword(navigation_delay, "period") == 10.0
    assert _literal_keyword(lab_delay, "period") == 15.0
    assert _name_list(_keyword(map_delay, "actions")) == ["map_manager"]
    assert _name_list(_keyword(navigation_delay, "actions")) == ["navigation_manager"]
    assert _name_list(_keyword(lab_delay, "actions")) == ["lab"]

    runtime = next(
        node
        for node in tree.body
        if isinstance(node, ast.FunctionDef) and node.name == "_launch_runtime"
    )
    returned = next(node.value for node in runtime.body if isinstance(node, ast.Return))
    assert isinstance(returned, ast.List)
    order = [_name(item) for item in returned.elts]
    assert order.index("map_server") < order.index("delayed_map_manager")
    assert order.index("delayed_map_manager") < order.index("delayed_navigation_manager")
    assert order.index("delayed_navigation_manager") < order.index("delayed_lab")


def test_navigation_manager_has_bounded_reconnection_and_exact_node_set() -> None:
    manager = _assigned_call(_tree(), "navigation_manager", "Node")
    parameters = _keyword(manager, "parameters")
    assert isinstance(parameters, ast.List) and len(parameters.elts) == 1
    # No autostart: the lab configures the nodes and then sends RESUME, so the
    # manager must not run its own bringup concurrently (one lifecycle owner).
    assert ast.literal_eval(parameters.elts[0]) == {
        "use_sim_time": True,
        "autostart": False,
        "attempt_respawn_reconnection": False,
        "node_names": [
            "controller_server",
            "smoother_server",
            "planner_server",
            "behavior_server",
            "velocity_smoother",
            "bt_navigator",
            "waypoint_follower",
        ],
    }


def test_shutdown_orders_managers_before_their_managed_nodes() -> None:
    tree = _tree()
    lab_handler = _assigned_call(tree, "stop_managers_after_lab", "RegisterEventHandler")
    navigation_handler = _assigned_call(
        tree, "stop_navigation_after_manager", "RegisterEventHandler"
    )
    map_handler = _assigned_call(tree, "stop_map_after_manager", "RegisterEventHandler")

    lab_exit = _calls(lab_handler, "OnProcessExit")[0]
    assert _name(_keyword(lab_exit, "target_action")) == "lab"
    assert {
        _name(call.args[0]) for call in _calls(lab_exit, "matches_action") if call.args
    } == {"navigation_manager", "map_manager"}
    shutdown_timers = _calls(lab_exit, "TimerAction")
    assert len(shutdown_timers) == 1
    assert _literal_keyword(shutdown_timers[0], "period") == 30.0
    assert len(_calls(shutdown_timers[0], "Shutdown")) == 1

    navigation_exit = _calls(navigation_handler, "OnProcessExit")[0]
    assert _name(_keyword(navigation_exit, "target_action")) == "navigation_manager"
    assert any(
        isinstance(call.args[0], ast.Name) and call.args[0].id == "node"
        for call in _calls(navigation_exit, "matches_action")
        if call.args
    )
    assert any(isinstance(node, ast.comprehension) for node in ast.walk(navigation_exit))

    map_exit = _calls(map_handler, "OnProcessExit")[0]
    assert _name(_keyword(map_exit, "target_action")) == "map_manager"
    assert [
        _name(call.args[0]) for call in _calls(map_exit, "matches_action") if call.args
    ] == ["map_server"]


def test_self_contained_lab_declares_transport_and_resource_environment_first() -> None:
    tree = _tree()
    description = next(
        node
        for node in tree.body
        if isinstance(node, ast.FunctionDef) and node.name == "generate_launch_description"
    )
    actions = _calls(description, "LaunchDescription")[0].args[0]
    assert isinstance(actions, ast.List)
    environment_calls = [
        action
        for action in actions.elts
        if isinstance(action, ast.Call)
        and isinstance(action.func, ast.Name)
        and action.func.id == "SetEnvironmentVariable"
    ]
    environment_names = [ast.literal_eval(call.args[0]) for call in environment_calls]

    assert [ast.literal_eval(arg) for arg in environment_calls[0].args] == [
        "ROS_AUTOMATIC_DISCOVERY_RANGE",
        "LOCALHOST",
    ]
    assert [ast.literal_eval(arg) for arg in environment_calls[1].args] == [
        "FASTDDS_BUILTIN_TRANSPORTS",
        "UDPv4",
    ]
    assert environment_names[2:] == ["SDF_PATH", "GZ_FILE_PATH", "GZ_SIM_RESOURCE_PATH"]
    assert actions.elts[: len(environment_calls)] == environment_calls


def test_launch_process_timeouts_are_explicit_and_signals_are_graceful() -> None:
    tree = _tree()
    settings = {
        (ast.literal_eval(call.args[0]), ast.literal_eval(call.args[1]))
        for call in _calls(tree, "SetLaunchConfiguration")
    }
    assert settings == {("sigterm_timeout", "10"), ("sigkill_timeout", "5")}

    signals = _calls(tree, "SignalProcess")
    assert signals
    for call in signals:
        signal_number = _keyword(call, "signal_number")
        assert isinstance(signal_number, ast.Attribute)
        assert isinstance(signal_number.value, ast.Name)
        assert (signal_number.value.id, signal_number.attr) == ("signal", "SIGINT")


# The Nav2 lab runs the canonical standard profile, so it must start every
# node that profile pairs against, and the velocity smoother it adds must sit
# inside the command chain, never beside it.
SAFETY_LATENCY_BOUND_SECONDS = 0.75  # bounded_stop_latency in ros2_execution_evidence


def _navigation_node_calls() -> dict[str, ast.Call]:
    nodes = _assignment(_tree(), "navigation_nodes")
    assert isinstance(nodes, ast.List)
    calls: dict[str, ast.Call] = {}
    for item in nodes.elts:
        assert isinstance(item, ast.Call) and _name(item.func) == "Node"
        name = _literal_keyword(item, "name")
        assert isinstance(name, str) and name not in calls
        calls[name] = item
    return calls


def _explicit_remappings(call: ast.Call) -> dict[str, str]:
    """The literal remappings a node adds on top of the shared tf remappings."""

    value = _keyword(call, "remappings")
    if value is None:
        return {}
    if isinstance(value, ast.BinOp):
        assert isinstance(value.op, ast.Add)
        assert _name(value.left) == "tf_remappings"
        value = value.right
    elif _name(value) == "tf_remappings":
        return {}
    pairs = ast.literal_eval(value)
    return dict(pairs)


def _navigation_manager_parameters() -> dict[str, object]:
    parameters = _keyword(_assigned_call(_tree(), "navigation_manager", "Node"), "parameters")
    assert isinstance(parameters, ast.List) and len(parameters.elts) == 1
    value = ast.literal_eval(parameters.elts[0])
    assert isinstance(value, dict)
    return value


def _safety_parameters() -> dict[str, object]:
    parameters = _keyword(_assigned_call(_tree(), "safety", "Node"), "parameters")
    assert isinstance(parameters, ast.List) and len(parameters.elts) == 1
    value = ast.literal_eval(parameters.elts[0])
    assert isinstance(value, dict)
    return value


def _nav2_section(name: str) -> dict[str, object]:
    """Scalar and flow-list parameters of one top-level node in nav2_params.yaml.

    PyYAML is not a dependency, and the smoother block is flat, so read it
    line by line rather than pull in a parser for one test.
    """

    lines = (ROOT / "config" / "nav2_params.yaml").read_text(encoding="utf-8").splitlines()
    start = lines.index(f"{name}:")
    assert lines[start + 1].strip() == "ros__parameters:"
    values: dict[str, object] = {}
    for line in lines[start + 2 :]:
        if line and not line.startswith(" "):
            break
        if not line.strip() or line.strip().startswith("#"):
            continue
        assert line.startswith("    ") and not line.startswith("     "), line
        key, _, raw = line.strip().partition(":")
        raw = raw.strip()
        if raw in {"true", "false"}:
            values[key] = raw == "true"
        elif raw.startswith("["):
            values[key] = ast.literal_eval(raw)
        else:
            try:
                values[key] = float(raw)
            except ValueError:
                values[key] = raw
    return values


def test_lab_starts_every_node_the_standard_profile_pairs_against() -> None:
    import json

    from flyto_robotics import ros2_closed_loop_lab as lab

    manifest = json.loads(
        (ROOT / "examples" / "ros2-adapters" / "flyto2-standard.json").read_text(encoding="utf-8")
    )
    (adapter,) = manifest["adapters"]
    required = set(adapter["managed_nodes"])
    started = {f"/{name}" for name in _navigation_node_calls()}
    started.add(f"/{_literal_keyword(_assigned_call(_tree(), 'map_server', 'Node'), 'name')}")

    assert started == required
    assert "/velocity_smoother" in started and "/waypoint_follower" in started
    # The manager activates exactly the navigation nodes, and the lab
    # configures exactly the same set before it sends RESUME.
    manager_nodes = _navigation_manager_parameters()["node_names"]
    assert isinstance(manager_nodes, list)
    assert set(manager_nodes) == set(_navigation_node_calls())
    assert [f"/{name}" for name in manager_nodes] == list(lab.NAVIGATION_LIFECYCLE_NODES)
    assert _navigation_manager_parameters()["autostart"] is False


def test_navigation_manager_brings_up_the_smoother_after_the_controller() -> None:
    order = _navigation_manager_parameters()["node_names"]
    assert isinstance(order, list)
    assert order == [
        "controller_server",
        "smoother_server",
        "planner_server",
        "behavior_server",
        "velocity_smoother",
        "bt_navigator",
        "waypoint_follower",
    ]


def test_velocity_chain_is_controller_then_smoother_then_safety_supervisor() -> None:
    calls = _navigation_node_calls()
    smoother = calls["velocity_smoother"]
    assert _literal_keyword(smoother, "package") == "nav2_velocity_smoother"
    assert _literal_keyword(smoother, "executable") == "velocity_smoother"

    raw = "/nav2/cmd_vel_nav"
    smoothed = "/nav2/cmd_vel"
    # Every Nav2 velocity producer feeds the smoother, never the supervisor.
    assert _explicit_remappings(calls["controller_server"]) == {"cmd_vel": raw}
    assert _explicit_remappings(calls["behavior_server"]) == {"cmd_vel": raw}
    # The smoother is the one stage between Nav2 and the supervisor.
    assert _explicit_remappings(smoother) == {"cmd_vel": raw, "cmd_vel_smoothed": smoothed}
    publishers_into_supervisor = [
        name
        for name, call in calls.items()
        if smoothed in _explicit_remappings(call).values()
    ]
    assert publishers_into_supervisor == ["velocity_smoother"]
    for name, call in calls.items():
        if name not in {"controller_server", "behavior_server", "velocity_smoother"}:
            assert _explicit_remappings(call) == {}, name

    # The safety supervisor stays the last stage: it alone reads the smoothed
    # command and alone writes the base's /cmd_vel.
    safety = _safety_parameters()
    assert safety["cmd_vel_input_topic"] == smoothed
    assert safety["cmd_vel_output_topic"] == "/cmd_vel"
    bridge_remaps = dict(
        ast.literal_eval(_keyword(_assigned_call(_tree(), "bridge", "Node"), "remappings"))
    )
    assert bridge_remaps["/flyto/cmd_vel"] == "/cmd_vel"
    for call in calls.values():
        assert "/cmd_vel" not in _explicit_remappings(call).values()


def test_smoother_deadman_ends_motion_inside_the_supervisor_command_timeout() -> None:
    import re

    smoother = _nav2_section("velocity_smoother")
    safety = _safety_parameters()
    command_timeout = float(safety["command_timeout_seconds"])
    assert command_timeout == 0.30

    velocity_timeout = float(smoother["velocity_timeout"])
    tick = 1.0 / float(smoother["smoothing_frequency"])
    supervisor_source = (ROOT / "flyto_robotics" / "ros2_safety_node.py").read_text(
        encoding="utf-8"
    )
    match = re.search(
        r'declare_parameter\("max_abs_linear_speed_mps", ([0-9.]+)\)', supervisor_source
    )
    assert match is not None
    assert "max_abs_linear_speed_mps" not in safety
    base_speed_cap = float(match.group(1))
    decel = abs(float(smoother["max_decel"][0]))

    # When the controller goes silent the smoother keeps sending its last
    # command until velocity_timeout (checked once per tick), then ramps to
    # zero. The base must be stopped by the time the supervisor alone would
    # have stopped it.
    motion_after_silence = velocity_timeout + tick + base_speed_cap / decel
    assert motion_after_silence <= command_timeout + 1e-9
    # The smoother then falls silent, and the supervisor's own command_stale
    # must still land inside the evidence's bounded stop latency.
    assert velocity_timeout + tick + command_timeout < SAFETY_LATENCY_BOUND_SECONDS
    assert smoother["feedback"] == "OPEN_LOOP"
    assert float(smoother["max_velocity"][0]) <= 0.35
    assert smoother["enable_stamped_cmd_vel"] is False


def test_bounded_stop_latency_constant_matches_the_evidence_verdict() -> None:
    source = (ROOT / "flyto_robotics" / "ros2_execution_evidence.py").read_text(encoding="utf-8")
    assert f'["safety_stop_latency_ms"] <= {SAFETY_LATENCY_BOUND_SECONDS * 1000:.1f}' in source
