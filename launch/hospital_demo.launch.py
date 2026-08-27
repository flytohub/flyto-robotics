"""Launch the self-contained hospital world and composed delivery workflow."""

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
    headless = LaunchConfiguration("headless").perform(context).lower()
    world = share / "worlds/hospital-logistics.sdf"
    gz_arguments = ["-r", "-v", "3"]
    if headless in {"1", "true", "yes", "on"}:
        gz_arguments.extend(["-s", "--headless-rendering"])
    gz_arguments.append(str(world))

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(ros_gz_share / "launch/gz_sim.launch.py")),
        launch_arguments={"gz_args": " ".join(gz_arguments)}.items(),
    )
    bridge = Node(
        package="flyto_robotics",
        executable="parameter_bridge_guard",
        name="flyto_gazebo_bridge",
        ros_arguments=["--disable-rosout-logs"],
        parameters=[{"config_file": str(share / "config/bridge.yaml")}],
        output="screen",
    )
    mission = Node(
        package="flyto_robotics",
        executable="mission_controller",
        name="flyto_hospital_delivery",
        parameters=[
            {
                "job_file": LaunchConfiguration("job_file"),
                "result_file": LaunchConfiguration("result_file"),
                "use_sim_time": True,
                "gazebo_physics": True,
            }
        ],
        output="screen",
    )
    shutdown_after_mission = RegisterEventHandler(
        OnProcessExit(target_action=mission, on_exit=[Shutdown(reason="mission finished")])
    )
    return [gazebo, bridge, mission, shutdown_after_mission]


def generate_launch_description() -> LaunchDescription:
    share = Path(get_package_share_directory("flyto_robotics"))
    models = str(share / "models")
    existing_resources = os.environ.get("GZ_SIM_RESOURCE_PATH", "")
    resource_path = os.pathsep.join(part for part in (models, existing_resources) if part)
    default_result = str(Path.cwd() / "results/mission-result.json")

    return LaunchDescription(
        [
            SetEnvironmentVariable("GZ_SIM_RESOURCE_PATH", resource_path),
            DeclareLaunchArgument(
                "job_file",
                default_value=str(share / "examples/jobs/pharmacy-to-ward.json"),
                description="Validated Flyto Robotics job contract",
            ),
            DeclareLaunchArgument(
                "result_file",
                default_value=default_result,
                description="Atomic mission-result JSON output path",
            ),
            DeclareLaunchArgument(
                "headless",
                default_value="false",
                description="Run Gazebo server without the GUI",
            ),
            OpaqueFunction(function=_launch_runtime),
        ]
    )
