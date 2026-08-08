"""Launch the physical-dimension TurtleBot3 Burger with Flyto safety boundaries."""

from __future__ import annotations

import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

from launch import LaunchDescription


def _launch_runtime(context: object) -> list[object]:
    turtlebot_share = Path(get_package_share_directory("turtlebot3_gazebo"))
    ros_gz_share = Path(get_package_share_directory("ros_gz_sim"))
    world = Path(LaunchConfiguration("world_file").perform(context))
    fault_scenario = LaunchConfiguration("fault_scenario").perform(context)
    if world.suffix != ".sdf" or not world.is_file():
        raise ValueError("world_file must reference a local SDF file")
    if fault_scenario not in {"none", "lidar_dropout", "odometry_freeze"}:
        raise ValueError("fault_scenario is unsupported for the Burger lab")

    model_file = turtlebot_share / "models/turtlebot3_burger/model.sdf"
    urdf_file = turtlebot_share / "urdf/turtlebot3_burger.urdf"
    if not model_file.is_file() or not urdf_file.is_file():
        raise ValueError("the installed ROBOTIS TurtleBot3 Burger assets are missing")

    gz_arguments = ["-r", "-v", "2"]
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
    spawn = Node(
        package="ros_gz_sim",
        executable="create",
        name="spawn_turtlebot3_burger",
        arguments=[
            "-name",
            "burger",
            "-file",
            str(model_file),
            "-x",
            LaunchConfiguration("x_pose"),
            "-y",
            LaunchConfiguration("y_pose"),
            "-z",
            "0.01",
        ],
        output="screen",
    )
    bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="flyto_turtlebot3_bridge",
        arguments=[
            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
            "/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist",
            "/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry",
            "/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V",
            "/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model",
            "/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan",
            "/imu@sensor_msgs/msg/Imu[gz.msgs.IMU",
        ],
        remappings=[
            ("/cmd_vel", "/flyto/actuator_cmd_vel"),
            ("/odom", "/flyto/raw_odom"),
            ("/scan", "/flyto/raw_scan"),
            ("/imu", "/flyto/imu"),
        ],
        output="screen",
    )
    state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="turtlebot3_state_publisher",
        parameters=[
            {
                "robot_description": urdf_file.read_text(encoding="utf-8"),
                "use_sim_time": True,
            }
        ],
        output="screen",
    )
    sensor_guard = Node(
        package="flyto_robotics",
        executable="ros2_sensor_guard",
        name="flyto_turtlebot3_sensor_guard",
        parameters=[
            {
                "use_sim_time": False,
                "fault_scenario": fault_scenario,
                "raw_odometry_topic": "/flyto/raw_odom",
                "odometry_topic": "/flyto/odom",
                "raw_lidar_topic": "/flyto/raw_scan",
                "lidar_topic": "/flyto/scan",
                "fault_delay_seconds": ParameterValue(
                    LaunchConfiguration("fault_delay_seconds"),
                    value_type=float,
                ),
                "command_topic": "/flyto/cmd_vel",
            }
        ],
        output="screen",
    )
    safety = Node(
        package="flyto_robotics",
        executable="ros2_safety_supervisor",
        namespace="safety",
        name="emergency_supervisor",
        parameters=[
            {
                "use_sim_time": False,
                "cmd_vel_input_topic": "/flyto/cmd_vel",
                "cmd_vel_output_topic": "/flyto/actuator_cmd_vel",
                "sensor_timeout_seconds": 0.40,
                "command_timeout_seconds": 0.30,
            }
        ],
        output="screen",
    )
    return [
        gazebo,
        bridge,
        state_publisher,
        sensor_guard,
        safety,
        TimerAction(period=1.0, actions=[spawn]),
    ]


def generate_launch_description() -> LaunchDescription:
    share = Path(get_package_share_directory("flyto_robotics"))
    turtlebot_share = Path(get_package_share_directory("turtlebot3_gazebo"))
    existing_resources = os.environ.get("GZ_SIM_RESOURCE_PATH", "")
    resource_path = os.pathsep.join(
        value
        for value in (
            str(share / "models"),
            str(turtlebot_share / "models"),
            existing_resources,
        )
        if value
    )
    return LaunchDescription(
        [
            SetEnvironmentVariable("GZ_SIM_RESOURCE_PATH", resource_path),
            SetEnvironmentVariable("TURTLEBOT3_MODEL", "burger"),
            DeclareLaunchArgument(
                "world_file",
                default_value=str(share / "worlds/turtlebot3-fidelity.sdf"),
            ),
            DeclareLaunchArgument("x_pose", default_value="-1.8"),
            DeclareLaunchArgument("y_pose", default_value="-1.1"),
            DeclareLaunchArgument("fault_scenario", default_value="none"),
            DeclareLaunchArgument("fault_delay_seconds", default_value="0.35"),
            DeclareLaunchArgument("headless", default_value="true"),
            OpaqueFunction(function=_launch_runtime),
        ]
    )
