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
    default_plan = share / "examples/plans/careflow-waypoints-human-gate.json"
    default_goal = (
        "依序經過藍色與黃色區域，確認走道淨空並取得護理站收件核准後，"
        "前往紫色區域安全停止。"
    )
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
            DeclareLaunchArgument("goal", default_value=default_goal),
            DeclareLaunchArgument("robot_id", default_value="flyto-rover-sim-001"),
            DeclareLaunchArgument("plan_file", default_value=str(default_plan)),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(str(lab_launch)),
                launch_arguments={
                    "plan_file": LaunchConfiguration("plan_file"),
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
                            / "examples/goal-frames/ai4all-careflow-showcase.json"
                        ),
                        "plan_file": LaunchConfiguration("plan_file"),
                        "goal": LaunchConfiguration("goal"),
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
