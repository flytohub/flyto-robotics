"""Launch the single-command AI4ALL physical-AI showcase."""

from __future__ import annotations

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from launch import LaunchDescription


def generate_launch_description() -> LaunchDescription:
    """Reuse the verified adversarial lab as the showcase execution boundary."""
    share = Path(get_package_share_directory("flyto_robotics"))
    lab_launch = share / "launch/gazebo_lab.launch.py"
    default_output = Path.cwd() / "results/ai4all-showcase"
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "output_dir",
                default_value=str(default_output),
                description="Generated evidence root; must remain outside source control",
            ),
            DeclareLaunchArgument(
                "headless",
                default_value="false",
                description="Show Gazebo while recording the same deterministic mission",
            ),
            DeclareLaunchArgument("robot_id", default_value="flyto-rover-sim-001"),
            DeclareLaunchArgument(
                "plan_file",
                description="Strict plan generated and validated by planning_session",
            ),
            DeclareLaunchArgument(
                "planning_session_file",
                description="Attested live planning and replan evidence",
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
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(str(lab_launch)),
                launch_arguments={
                    "plan_file": LaunchConfiguration("plan_file"),
                    "world_file": str(
                        share / "worlds/ai4all-branching-route.sdf"
                    ),
                    "semantic_map_file": str(
                        share / "examples/maps/ai4all-branching-route.json"
                    ),
                    "semantic_map_id": "gazebo.ai4all-branching-route.v1",
                    "result_file": [
                        LaunchConfiguration("output_dir"),
                        "/mission-result.json",
                    ],
                    "evidence_dir": [
                        LaunchConfiguration("output_dir"),
                        "/images",
                    ],
                    "video_frames_dir": [
                        LaunchConfiguration("output_dir"),
                        "/frames/overhead",
                    ],
                    "video_max_frames": "900",
                    "qr_recipient_ref": LaunchConfiguration(
                        "qr_recipient_ref"
                    ),
                    "guarded_handoff_policy_file": LaunchConfiguration(
                        "guarded_handoff_policy_file"
                    ),
                    "guarded_handoff_script_file": LaunchConfiguration(
                        "guarded_handoff_script_file"
                    ),
                    "guarded_handoff_step_delay_seconds": LaunchConfiguration(
                        "guarded_handoff_step_delay_seconds"
                    ),
                    "headless": LaunchConfiguration("headless"),
                }.items(),
            ),
            Node(
                package="flyto_robotics",
                executable="showcase_gazebo_observer",
                name="flyto_ai4all_multidevice_observer",
                parameters=[
                    {
                        "resource_file": str(
                            share
                            / "examples/facility-resources/ai4all-showcase-facility.json"
                        ),
                        "goal_frame_file": str(
                            share
                            / "examples/goal-frames/ai4all-branching-careflow.json"
                        ),
                        "plan_file": LaunchConfiguration("plan_file"),
                        "planning_session_file": LaunchConfiguration(
                            "planning_session_file"
                        ),
                        "robot_id": LaunchConfiguration("robot_id"),
                        "evidence_dir": [
                            LaunchConfiguration("output_dir"),
                            "/facility",
                        ],
                        "video_frames_dir": [
                            LaunchConfiguration("output_dir"),
                            "/frames/active-camera",
                        ],
                        "video_max_frames": 900,
                        "use_sim_time": True,
                    }
                ],
                output="screen",
            ),
        ]
    )
