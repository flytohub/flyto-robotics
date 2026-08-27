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
    assert ast.literal_eval(parameters.elts[0]) == {
        "use_sim_time": True,
        "autostart": True,
        "attempt_respawn_reconnection": False,
        "node_names": [
            "controller_server",
            "smoother_server",
            "planner_server",
            "behavior_server",
            "bt_navigator",
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
