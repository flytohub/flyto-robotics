"""Launch the self-contained Flyto rover, Nav2, safety, and evidence runner."""

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
    TimerAction,
)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

from launch import LaunchDescription


def _launch_runtime(context: object) -> list[object]:
    ros_gz_share = Path(get_package_share_directory("ros_gz_sim"))
    world = Path(LaunchConfiguration("world_file").perform(context))
    map_file = Path(LaunchConfiguration("map_file").perform(context))
    params = Path(LaunchConfiguration("params_file").perform(context))
    for path, suffix in ((world, ".sdf"), (map_file, ".yaml"), (params, ".yaml")):
        if path.suffix != suffix or not path.is_file():
            raise ValueError(f"required local asset is invalid: {path}")
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
    bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="flyto_nav2_gazebo_bridge",
        arguments=[
            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
            "/flyto/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist",
            "/flyto/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry",
            "/flyto/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V",
            "/flyto/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan",
        ],
        remappings=[
            ("/flyto/cmd_vel", "/cmd_vel"),
            ("/flyto/tf", "/tf"),
        ],
        output="screen",
    )
    map_to_odom = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="flyto_map_to_odom",
        arguments=[
            "--x", "0", "--y", "0", "--z", "0",
            "--roll", "0", "--pitch", "0", "--yaw", "0",
            "--frame-id", "map", "--child-frame-id", "odom",
        ],
        output="screen",
    )
    base_to_lidar = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="flyto_base_to_lidar",
        arguments=[
            "--x", "0.10", "--y", "0", "--z", "0.145",
            "--roll", "0", "--pitch", "0", "--yaw", "0",
            "--frame-id", "base_link", "--child-frame-id", "lidar_link",
        ],
        output="screen",
    )
    map_server = Node(
        package="nav2_map_server",
        executable="map_server",
        name="map_server",
        parameters=[str(params), {"yaml_filename": str(map_file)}],
        output="screen",
    )
    map_manager = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="map_lifecycle_manager",
        parameters=[
            {
                "use_sim_time": True,
                "autostart": True,
                "node_names": ["map_server"],
            }
        ],
        output="screen",
    )
    tf_remappings = [("/tf", "tf"), ("/tf_static", "tf_static")]
    navigation_nodes = [
        Node(
            package="nav2_controller",
            executable="controller_server",
            name="controller_server",
            parameters=[str(params)],
            remappings=tf_remappings + [("cmd_vel", "/nav2/cmd_vel")],
            output="screen",
        ),
        Node(
            package="nav2_smoother",
            executable="smoother_server",
            name="smoother_server",
            parameters=[str(params)],
            remappings=tf_remappings,
            output="screen",
        ),
        Node(
            package="nav2_planner",
            executable="planner_server",
            name="planner_server",
            parameters=[str(params)],
            remappings=tf_remappings,
            output="screen",
        ),
        Node(
            package="nav2_behaviors",
            executable="behavior_server",
            name="behavior_server",
            parameters=[str(params)],
            remappings=tf_remappings + [("cmd_vel", "/nav2/cmd_vel")],
            output="screen",
        ),
        Node(
            package="nav2_bt_navigator",
            executable="bt_navigator",
            name="bt_navigator",
            parameters=[str(params)],
            remappings=tf_remappings,
            output="screen",
        ),
        Node(
            package="nav2_lifecycle_manager",
            executable="lifecycle_manager",
            name="lifecycle_manager_navigation",
            parameters=[
                {
                    "use_sim_time": True,
                    "autostart": True,
                    "node_names": [
                        "controller_server",
                        "smoother_server",
                        "planner_server",
                        "behavior_server",
                        "bt_navigator",
                    ],
                }
            ],
            output="screen",
        ),
    ]
    safety = Node(
        package="flyto_robotics",
        executable="ros2_safety_supervisor",
        namespace="safety",
        name="emergency_supervisor",
        parameters=[
            {
                "use_sim_time": True,
                "cmd_vel_input_topic": "/nav2/cmd_vel",
                "cmd_vel_output_topic": "/cmd_vel",
            }
        ],
        output="screen",
    )
    lab = Node(
        package="flyto_robotics",
        executable="ros2_closed_loop_lab",
        name="flyto_nav2_closed_loop_lab",
        parameters=[
            {
                "use_sim_time": True,
                "manifest_file": LaunchConfiguration("manifest_file"),
                "resource_plan_file": LaunchConfiguration("resource_plan_file"),
                "semantic_map_file": LaunchConfiguration("semantic_map_file"),
                "semantic_map_id": LaunchConfiguration("semantic_map_id"),
                "scenario": LaunchConfiguration("scenario"),
                "output_file": LaunchConfiguration("output_file"),
                "cancel_after_displacement_m": ParameterValue(
                    LaunchConfiguration("cancel_after_displacement_m"),
                    value_type=float,
                ),
            }
        ],
        output="screen",
    )
    shutdown = RegisterEventHandler(
        OnProcessExit(target_action=lab, on_exit=[Shutdown(reason="closed-loop finished")])
    )
    delayed_lab = TimerAction(period=6.0, actions=[lab])
    return [
        gazebo,
        bridge,
        map_to_odom,
        base_to_lidar,
        map_server,
        map_manager,
        *navigation_nodes,
        safety,
        delayed_lab,
        shutdown,
    ]


def generate_launch_description() -> LaunchDescription:
    share = Path(get_package_share_directory("flyto_robotics"))
    existing_resources = os.environ.get("GZ_SIM_RESOURCE_PATH", "")
    resource_path = os.pathsep.join(
        value for value in (str(share / "models"), existing_resources) if value
    )
    return LaunchDescription(
        [
            SetEnvironmentVariable("GZ_SIM_RESOURCE_PATH", resource_path),
            DeclareLaunchArgument(
                "world_file",
                default_value=str(share / "worlds/nav2-closed-loop.sdf"),
            ),
            DeclareLaunchArgument(
                "map_file",
                default_value=str(share / "maps/nav2_lab.yaml"),
            ),
            DeclareLaunchArgument(
                "params_file",
                default_value=str(share / "config/nav2_params.yaml"),
            ),
            DeclareLaunchArgument(
                "manifest_file",
                default_value=str(
                    share / "examples/ros2-adapters/flyto2-standard.json"
                ),
            ),
            DeclareLaunchArgument(
                "resource_plan_file",
                default_value=str(
                    share / "examples/resource-plans/nav2-hospital-delivery.json"
                ),
            ),
            DeclareLaunchArgument(
                "semantic_map_file",
                default_value=str(share / "examples/maps/atomic-color-route.json"),
            ),
            DeclareLaunchArgument(
                "semantic_map_id",
                default_value="gazebo.atomic-color-route.v1",
            ),
            DeclareLaunchArgument("scenario", default_value="success"),
            DeclareLaunchArgument(
                "output_file",
                default_value=str(Path.cwd() / "results/nav2-closed-loop.json"),
            ),
            DeclareLaunchArgument(
                "cancel_after_displacement_m",
                default_value="0.25",
            ),
            DeclareLaunchArgument("headless", default_value="true"),
            OpaqueFunction(function=_launch_runtime),
        ]
    )
