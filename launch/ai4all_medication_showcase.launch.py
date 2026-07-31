"""Launch the advanced synthetic medication handoff showcase."""

from __future__ import annotations

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from launch import LaunchDescription


def generate_launch_description() -> LaunchDescription:
    """Reuse the branching mission and add deterministic high-risk gates."""
    share = Path(get_package_share_directory("flyto_robotics"))
    base_launch = share / "launch/ai4all_showcase.launch.py"
    default_output = Path.cwd() / "results/ai4all-medication-showcase"
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "output_dir",
                default_value=str(default_output),
            ),
            DeclareLaunchArgument("headless", default_value="false"),
            DeclareLaunchArgument("robot_id", default_value="flyto-rover-sim-001"),
            DeclareLaunchArgument("plan_file"),
            DeclareLaunchArgument("planning_session_file"),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(str(base_launch)),
                launch_arguments={
                    "output_dir": LaunchConfiguration("output_dir"),
                    "headless": LaunchConfiguration("headless"),
                    "robot_id": LaunchConfiguration("robot_id"),
                    "plan_file": LaunchConfiguration("plan_file"),
                    "planning_session_file": LaunchConfiguration(
                        "planning_session_file"
                    ),
                    "qr_recipient_ref": "patient-12",
                    "guarded_handoff_policy_file": str(
                        share
                        / "examples/guarded-handoff/medication-policy.json"
                    ),
                    "guarded_handoff_script_file": str(
                        share
                        / "examples/guarded-handoff/medication-script.json"
                    ),
                    "guarded_handoff_step_delay_seconds": "0.65",
                }.items(),
            ),
        ]
    )
