"""Gazebo world + the twin burger + robot_state_publisher.

Replaces turtlebot3_gazebo's world launch for the twin. Differences from the
upstream launch, each to match flyto-robot.local:

- the model is models/flyto_burger (robot's sensor rates, ranges, camera);
- no ros_gz_bridge: twin_hardware.py reads Gazebo directly and publishes the
  robot's driver topics under the robot's node names;
- robot_state_publisher gets the robot's own robot_description text and
  frame_prefix '' (upstream's launch turns an empty prefix into '/', which
  produced frames named '/base_link').
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch.actions import (
    AppendEnvironmentVariable,
    DeclareLaunchArgument,
    IncludeLaunchDescription,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from launch import LaunchDescription

TWIN = os.environ.get("TWIN_HOME", "/opt/flyto-twin")


def generate_launch_description():
    tb3_gazebo = get_package_share_directory("turtlebot3_gazebo")
    ros_gz_sim = get_package_share_directory("ros_gz_sim")
    world = LaunchConfiguration("world")
    x_pose = LaunchConfiguration("x_pose")
    y_pose = LaunchConfiguration("y_pose")
    yaw = LaunchConfiguration("yaw")

    with open(os.path.join(TWIN, "config", "real", "robot_description.urdf")) as handle:
        robot_description = handle.read()

    return LaunchDescription([
        DeclareLaunchArgument(
            "world",
            default_value=os.path.join(tb3_gazebo, "worlds", "turtlebot3_world.world"),
        ),
        DeclareLaunchArgument("x_pose", default_value="-2.0"),
        DeclareLaunchArgument("y_pose", default_value="-0.5"),
        DeclareLaunchArgument("yaw", default_value="0.0"),
        AppendEnvironmentVariable("GZ_SIM_RESOURCE_PATH", os.path.join(tb3_gazebo, "models")),
        AppendEnvironmentVariable("GZ_SIM_RESOURCE_PATH", os.path.join(TWIN, "models")),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(ros_gz_sim, "launch", "gz_sim.launch.py")),
            launch_arguments={"gz_args": ["-r -s -v2 ", world], "on_exit_shutdown": "true"}.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(ros_gz_sim, "launch", "gz_sim.launch.py")),
            # The window is only a viewer: if it dies, the simulation keeps running.
            launch_arguments={"gz_args": "-g -v2 ", "on_exit_shutdown": "false"}.items(),
        ),
        Node(
            package="ros_gz_sim",
            executable="create",
            arguments=[
                "-name", "burger",
                "-file", os.path.join(TWIN, "models", "flyto_burger", "model.sdf"),
                "-x", x_pose, "-y", y_pose, "-z", "0.01", "-Y", yaw,
            ],
            output="screen",
        ),
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            output="screen",
            parameters=[{
                "use_sim_time": True,
                "robot_description": robot_description,
                "frame_prefix": "",
                "publish_frequency": 20.0,
            }],
        ),
    ])
