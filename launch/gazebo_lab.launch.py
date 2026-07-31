"""Launch the adversarial Gazebo evidence laboratory."""

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
    world = Path(LaunchConfiguration("world_file").perform(context))
    if world.suffix != ".sdf" or not world.is_file():
        raise ValueError("world_file must reference a readable local SDF file")
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
        name="flyto_lab_gazebo_bridge",
        parameters=[{"config_file": str(share / "config/bridge.yaml")}],
        output="screen",
    )
    driver = Node(
        package="flyto_robotics",
        executable="gazebo_lab_driver",
        name="flyto_adversarial_lab_driver",
        parameters=[
            {
                "job_file": LaunchConfiguration("job_file"),
                "evidence_dir": LaunchConfiguration("evidence_dir"),
                "scenario_id": "gazebo.careflow.adversarial.v1",
                "qr_recipient_ref": LaunchConfiguration("qr_recipient_ref"),
                "guarded_handoff_policy_file": LaunchConfiguration(
                    "guarded_handoff_policy_file"
                ),
                "guarded_handoff_script_file": LaunchConfiguration(
                    "guarded_handoff_script_file"
                ),
                "guarded_handoff_step_delay_seconds": LaunchConfiguration(
                    "guarded_handoff_step_delay_seconds"
                ),
                "video_frames_dir": LaunchConfiguration("video_frames_dir"),
                "video_max_frames": LaunchConfiguration("video_max_frames"),
                "use_sim_time": True,
            }
        ],
        output="screen",
    )
    executor = Node(
        package="flyto_robotics",
        executable="mission_controller",
        name="flyto_lab_capability_executor",
        parameters=[
            {
                "job_file": LaunchConfiguration("job_file"),
                "plan_file": LaunchConfiguration("plan_file"),
                "result_file": LaunchConfiguration("result_file"),
                "semantic_map_file": LaunchConfiguration("semantic_map_file"),
                "semantic_map_id": LaunchConfiguration("semantic_map_id"),
                "use_sim_time": True,
                "gazebo_physics": True,
                "obstacle_injected": True,
                "human_approval_injected": True,
            }
        ],
        output="screen",
    )
    shutdown_after_execution = RegisterEventHandler(
        OnProcessExit(target_action=executor, on_exit=[Shutdown(reason="lab finished")])
    )
    return [gazebo, bridge, driver, executor, shutdown_after_execution]


def generate_launch_description() -> LaunchDescription:
    share = Path(get_package_share_directory("flyto_robotics"))
    existing_resources = os.environ.get("GZ_SIM_RESOURCE_PATH", "")
    resource_path = os.pathsep.join(
        part for part in (str(share / "models"), existing_resources) if part
    )
    default_output = Path.cwd() / "results/gazebo-lab"
    return LaunchDescription(
        [
            SetEnvironmentVariable("GZ_SIM_RESOURCE_PATH", resource_path),
            DeclareLaunchArgument(
                "job_file",
                default_value=str(share / "examples/jobs/pharmacy-to-ward.json"),
            ),
            DeclareLaunchArgument(
                "world_file",
                default_value=str(share / "worlds/atomic-color-route-lab.sdf"),
            ),
            DeclareLaunchArgument(
                "plan_file",
                default_value=str(
                    share / "examples/plans/careflow-waypoints-human-gate.json"
                ),
            ),
            DeclareLaunchArgument(
                "result_file",
                default_value=str(default_output / "mission-result.json"),
            ),
            DeclareLaunchArgument(
                "evidence_dir",
                default_value=str(default_output / "images"),
            ),
            DeclareLaunchArgument("video_frames_dir", default_value=""),
            DeclareLaunchArgument("video_max_frames", default_value="600"),
            DeclareLaunchArgument(
                "semantic_map_file",
                default_value=str(
                    share / "examples/maps/atomic-color-route.json"
                ),
            ),
            DeclareLaunchArgument(
                "semantic_map_id",
                default_value="gazebo.atomic-color-route.v1",
            ),
            DeclareLaunchArgument(
                "qr_recipient_ref",
                default_value="ward-b.receiver",
            ),
            DeclareLaunchArgument(
                "guarded_handoff_policy_file",
                default_value="",
            ),
            DeclareLaunchArgument(
                "guarded_handoff_script_file",
                default_value="",
            ),
            DeclareLaunchArgument(
                "guarded_handoff_step_delay_seconds",
                default_value="0.65",
            ),
            DeclareLaunchArgument("headless", default_value="true"),
            OpaqueFunction(function=_launch_runtime),
        ]
    )
