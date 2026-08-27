from __future__ import annotations

import ast
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
LAUNCH_DIR = ROOT / "launch"


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


def _launch_tree(path: Path) -> ast.Module:
    return ast.parse(path.read_text(encoding="utf-8"), filename=str(path))


GUARDED_GAZEBO_LAUNCHES = {
    "atomic_ai_demo.launch.py",
    "gazebo_lab.launch.py",
    "hospital_demo.launch.py",
    "shortcut_gazebo_demo.launch.py",
}


def _guarded_bridge(launch_file: Path) -> ast.Call:
    bridges = [
        call
        for call in _calls(_launch_tree(launch_file), "Node")
        if _literal_keyword(call, "executable") == "parameter_bridge_guard"
    ]
    assert len(bridges) == 1, f"{launch_file} must launch exactly one bridge guard"
    return bridges[0]


def test_gazebo_bridges_use_the_supervised_guard_contract() -> None:
    for filename in sorted(GUARDED_GAZEBO_LAUNCHES):
        launch_file = LAUNCH_DIR / filename
        bridge = _guarded_bridge(launch_file)
        assert _literal_keyword(bridge, "package") == "flyto_robotics"
        assert _literal_keyword(bridge, "executable") == "parameter_bridge_guard"
        assert _literal_keyword(bridge, "ros_arguments") == [
            "--disable-rosout-logs"
        ]
        assert _literal_keyword(bridge, "output") == "screen", (
            f"{launch_file} must retain bridge diagnostics on the console"
        )

        assert not any(
            _literal_keyword(call, "package") == "ros_gz_bridge"
            and _literal_keyword(call, "executable") == "parameter_bridge"
            for call in _calls(_launch_tree(launch_file), "Node")
        ), f"{launch_file} bypasses the supervised bridge guard"


def test_gazebo_launches_use_only_local_resource_paths() -> None:
    for filename in sorted(GUARDED_GAZEBO_LAUNCHES):
        launch_file = LAUNCH_DIR / filename
        tree = _launch_tree(launch_file)
        bridge = _guarded_bridge(launch_file)
        assert _literal_keyword(bridge, "package") == "flyto_robotics"
        assert _literal_keyword(bridge, "ros_arguments") == [
            "--disable-rosout-logs"
        ]
        environment_names = {
            ast.literal_eval(call.args[0])
            for call in _calls(tree, "SetEnvironmentVariable")
            if call.args and isinstance(call.args[0], ast.Constant)
        }
        assert "GZ_SIM_RESOURCE_PATH" in environment_names, (
            f"{launch_file} does not declare its local Gazebo resource path"
        )
        source = launch_file.read_text(encoding="utf-8")
        assert "Fuel" not in source
        assert "http://" not in source
        assert "https://" not in source


def test_mission_launches_shut_down_when_their_bounded_worker_exits() -> None:
    expected_targets = {
        "atomic_ai_demo.launch.py": "executor",
        "gazebo_lab.launch.py": "executor",
        "hospital_demo.launch.py": "mission",
        "shortcut_gazebo_demo.launch.py": "driver",
    }

    for filename, target in expected_targets.items():
        tree = _launch_tree(LAUNCH_DIR / filename)
        matching_handlers = [
            call
            for call in _calls(tree, "OnProcessExit")
            if isinstance(_keyword(call, "target_action"), ast.Name)
            and _keyword(call, "target_action").id == target
        ]
        assert matching_handlers, f"{filename} does not supervise {target}"
        assert any(
            _calls(value, "Shutdown")
            for call in matching_handlers
            if (value := _keyword(call, "on_exit")) is not None
        ), f"{filename} does not stop after {target} exits"
