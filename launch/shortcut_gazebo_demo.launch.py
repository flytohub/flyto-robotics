"""Launch the Flyto2 workflow-card input closed loop in Gazebo."""

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
    gz_arguments = ["-r", "-v", "3"]
    if LaunchConfiguration("headless").perform(context).lower() in {
        "1",
        "true",
        "yes",
        "on",
    }:
        gz_arguments.extend(["-s", "--headless-rendering"])
    gz_arguments.append(str(share / "worlds/atomic-color-route-lab.sdf"))

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(ros_gz_share / "launch/gz_sim.launch.py")),
        launch_arguments={"gz_args": " ".join(gz_arguments)}.items(),
    )
    bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="flyto_shortcut_gazebo_bridge",
        parameters=[{"config_file": str(share / "config/bridge.yaml")}],
        output="screen",
    )
    driver = Node(
        package="flyto_robotics",
        executable="shortcut_gazebo_driver",
        name="flyto_shortcut_demo_driver",
        parameters=[
            {
                "job_file": LaunchConfiguration("job_file"),
                "evidence_dir": LaunchConfiguration("evidence_dir"),
                "video_frames_dir": LaunchConfiguration("video_frames_dir"),
                "video_max_frames": LaunchConfiguration("video_max_frames"),
                "minimum_video_frames": LaunchConfiguration("minimum_video_frames"),
                "use_sim_time": True,
            }
        ],
        output="screen",
    )
    executor = Node(
        package="flyto_robotics",
        executable="shortcut_controller",
        name="flyto_shortcut_executor",
        parameters=[
            {
                "job_file": LaunchConfiguration("job_file"),
                "plan_file": LaunchConfiguration("plan_file"),
                "result_file": LaunchConfiguration("result_file"),
                "deadman_timeout_seconds": 0.8,
                "gateway_enabled": False,
                "exit_after_completed_workflows": 0,
                "gazebo_physics": True,
                "obstacle_injected": True,
                "use_sim_time": True,
            }
        ],
        output="screen",
    )
    shutdown = RegisterEventHandler(
        OnProcessExit(
            target_action=driver,
            on_exit=[Shutdown(reason="shortcut closed loop finished")],
        )
    )
    return [gazebo, bridge, driver, executor, shutdown]


def generate_launch_description() -> LaunchDescription:
    share = Path(get_package_share_directory("flyto_robotics"))
    existing_resources = os.environ.get("GZ_SIM_RESOURCE_PATH", "")
    resource_path = os.pathsep.join(
        part for part in (str(share / "models"), existing_resources) if part
    )
    default_output = Path.cwd() / "results/shortcut-gazebo"
    return LaunchDescription(
        [
            SetEnvironmentVariable("GZ_SIM_RESOURCE_PATH", resource_path),
            DeclareLaunchArgument(
                "job_file",
                default_value=str(share / "examples/jobs/pharmacy-to-ward.json"),
            ),
            DeclareLaunchArgument(
                "plan_file",
                default_value=str(
                    share / "examples/plans/shortcut-forward-30cm.json"
                ),
            ),
            DeclareLaunchArgument(
                "result_file",
                default_value=str(default_output / "shortcut-result.json"),
            ),
            DeclareLaunchArgument(
                "evidence_dir",
                default_value=str(default_output / "images"),
            ),
            DeclareLaunchArgument("video_frames_dir", default_value=""),
            DeclareLaunchArgument("video_max_frames", default_value="600"),
            DeclareLaunchArgument("minimum_video_frames", default_value="8"),
            DeclareLaunchArgument("headless", default_value="true"),
            OpaqueFunction(function=_launch_runtime),
        ]
    )
