"""Launch the AI-composed blue/yellow/purple capability-plan example."""

from __future__ import annotations

import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    RegisterEventHandler,
    SetEnvironmentVariable,
    Shutdown,
)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from launch import LaunchDescription


def _launch_runtime(context: object) -> list[object]:
    share = Path(get_package_share_directory("flyto_robotics"))
    ros_gz_share = Path(get_package_share_directory("ros_gz_sim"))
    world = share / "worlds/atomic-color-route.sdf"
    gz_arguments = ["-r", "-v", "3"]
    if LaunchConfiguration("headless").perform(context).lower() in {
        "1",
        "true",
        "yes",
        "on",
    }:
        gz_arguments.extend(["-s", "--headless-rendering"])
    gz_arguments.append(str(world))

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(ros_gz_share / "launch/gz_sim.launch.py")),
        launch_arguments={"gz_args": " ".join(gz_arguments)}.items(),
    )
    bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="flyto_atomic_gazebo_bridge",
        parameters=[{"config_file": str(share / "config/bridge.yaml")}],
        output="screen",
    )
    executor = Node(
        package="flyto_robotics",
        executable="mission_controller",
        name="flyto_ai_capability_executor",
        parameters=[
            {
                "job_file": LaunchConfiguration("job_file"),
                "plan_file": LaunchConfiguration("plan_file"),
                "result_file": LaunchConfiguration("result_file"),
                "semantic_map_file": LaunchConfiguration("semantic_map_file"),
                "semantic_map_id": LaunchConfiguration("semantic_map_id"),
                "use_sim_time": True,
                "gazebo_physics": True,
            }
        ],
        output="screen",
    )
    shutdown_after_execution = RegisterEventHandler(
        OnProcessExit(target_action=executor, on_exit=[Shutdown(reason="plan finished")])
    )
    return [gazebo, bridge, executor, shutdown_after_execution]


def generate_launch_description() -> LaunchDescription:
    share = Path(get_package_share_directory("flyto_robotics"))
    existing_resources = os.environ.get("GZ_SIM_RESOURCE_PATH", "")
    resource_path = os.pathsep.join(
        part for part in (str(share / "models"), existing_resources) if part
    )
    default_result = str(Path.cwd() / "results/atomic-ai-result.json")
    return LaunchDescription(
        [
            SetEnvironmentVariable("GZ_SIM_RESOURCE_PATH", resource_path),
            DeclareLaunchArgument(
                "job_file",
                default_value=str(share / "examples/jobs/pharmacy-to-ward.json"),
                description="Safety limits and target robot identity",
            ),
            DeclareLaunchArgument(
                "plan_file",
                default_value=str(
                    share / "examples/plans/blue-yellow-purple-waypoints.json"
                ),
                description="Validated AI-composed capability plan",
            ),
            DeclareLaunchArgument(
                "result_file",
                default_value=default_result,
                description="Atomic execution-result JSON output path",
            ),
            DeclareLaunchArgument(
                "semantic_map_file",
                default_value=str(
                    share / "examples/maps/atomic-color-route.json"
                ),
                description="Trusted semantic-location map",
            ),
            DeclareLaunchArgument(
                "semantic_map_id",
                default_value="gazebo.atomic-color-route.v1",
                description="Expected physical map identity",
            ),
            DeclareLaunchArgument(
                "headless",
                default_value="false",
                description="Run Gazebo server without the GUI",
            ),
            OpaqueFunction(function=_launch_runtime),
        ]
    )
