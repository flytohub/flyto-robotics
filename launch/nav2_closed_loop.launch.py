"""Launch the self-contained Flyto rover, Nav2, safety, and evidence runner."""

from __future__ import annotations

import os
import signal
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    IncludeLaunchDescription,
    OpaqueFunction,
    RegisterEventHandler,
    SetEnvironmentVariable,
    SetLaunchConfiguration,
    Shutdown,
    TimerAction,
)
from launch.event_handlers import OnProcessExit
from launch.events import matches_action
from launch.events.process import SignalProcess
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
    scenario = LaunchConfiguration("scenario").perform(context)
    supported_scenarios = {
        "success",
        "cancel",
        "emergency_stop",
        "lidar_dropout",
        "odometry_freeze",
        "nav2_lifecycle_failure",
    }
    if scenario not in supported_scenarios:
        raise ValueError("closed-loop scenario is unsupported")
    fault_scenario = scenario if scenario.endswith(("dropout", "freeze", "failure")) else "none"
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
            ("/flyto/odom", "/flyto/raw_odom"),
            ("/flyto/tf", "/tf"),
            ("/flyto/scan", "/flyto/raw_scan"),
        ],
        output="screen",
    )
    sensor_guard = Node(
        package="flyto_robotics",
        executable="ros2_sensor_guard",
        name="flyto_sensor_guard",
        parameters=[
            {
                "use_sim_time": False,
                "fault_scenario": fault_scenario,
                "fault_delay_seconds": ParameterValue(
                    LaunchConfiguration("fault_delay_seconds"),
                    value_type=float,
                ),
            }
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
    ]
    navigation_manager = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_navigation",
        parameters=[
            {
                "use_sim_time": True,
                "autostart": True,
                "attempt_respawn_reconnection": False,
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
    )
    safety = Node(
        package="flyto_robotics",
        executable="ros2_safety_supervisor",
        namespace="safety",
        name="emergency_supervisor",
        parameters=[
            {
                "use_sim_time": False,
                "cmd_vel_input_topic": "/nav2/cmd_vel",
                "cmd_vel_output_topic": "/cmd_vel",
                "fault_state_topic": "/fault_injection/state",
                "sensor_timeout_seconds": 0.40,
                "command_timeout_seconds": 0.30,
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
                "odometry_topic": "/flyto/raw_odom",
                "safety_reason_topic": "/safety/stop_reason",
                "fault_state_topic": "/fault_injection/state",
                "execution_state_topic": "/flyto/navigation_execution_active",
                "cancel_after_displacement_m": ParameterValue(
                    LaunchConfiguration("cancel_after_displacement_m"),
                    value_type=float,
                ),
            }
        ],
        output="screen",
    )
    stop_managers_after_lab = RegisterEventHandler(
        OnProcessExit(
            target_action=lab,
            on_exit=[
                EmitEvent(
                    event=SignalProcess(
                        signal_number=signal.SIGINT,
                        process_matcher=matches_action(navigation_manager),
                    )
                ),
                EmitEvent(
                    event=SignalProcess(
                        signal_number=signal.SIGINT,
                        process_matcher=matches_action(map_manager),
                    )
                ),
                TimerAction(
                    period=30.0,
                    actions=[Shutdown(reason="closed-loop finished")],
                ),
            ],
        )
    )
    stop_navigation_after_manager = RegisterEventHandler(
        OnProcessExit(
            target_action=navigation_manager,
            on_exit=[
                EmitEvent(
                    event=SignalProcess(
                        signal_number=signal.SIGINT,
                        process_matcher=matches_action(node),
                    )
                )
                for node in navigation_nodes
            ],
        )
    )
    stop_map_after_manager = RegisterEventHandler(
        OnProcessExit(
            target_action=map_manager,
            on_exit=[
                EmitEvent(
                    event=SignalProcess(
                        signal_number=signal.SIGINT,
                        process_matcher=matches_action(map_server),
                    )
                )
            ],
        )
    )
    delayed_map_manager = TimerAction(period=2.0, actions=[map_manager])
    delayed_navigation_manager = TimerAction(
        period=10.0, actions=[navigation_manager]
    )
    delayed_lab = TimerAction(period=15.0, actions=[lab])
    return [
        gazebo,
        bridge,
        sensor_guard,
        map_to_odom,
        base_to_lidar,
        map_server,
        delayed_map_manager,
        *navigation_nodes,
        delayed_navigation_manager,
        safety,
        delayed_lab,
        stop_managers_after_lab,
        stop_navigation_after_manager,
        stop_map_after_manager,
    ]


def generate_launch_description() -> LaunchDescription:
    share = Path(get_package_share_directory("flyto_robotics"))
    existing_resources = os.environ.get("GZ_SIM_RESOURCE_PATH", "")
    existing_sdf_paths = os.environ.get("SDF_PATH", "")
    existing_common_paths = os.environ.get("GZ_FILE_PATH", "")
    resource_path = os.pathsep.join(
        value for value in (str(share / "models"), existing_resources) if value
    )
    return LaunchDescription(
        [
            # This simulation launch owns a sealed local ROS graph. Hardware
            # launches must opt into their deployment-specific discovery.
            SetEnvironmentVariable("ROS_AUTOMATIC_DISCOVERY_RANGE", "LOCALHOST"),
            SetEnvironmentVariable("FASTDDS_BUILTIN_TRANSPORTS", "UDPv4"),
            SetEnvironmentVariable("SDF_PATH", existing_sdf_paths),
            SetEnvironmentVariable("GZ_FILE_PATH", existing_common_paths),
            SetEnvironmentVariable("GZ_SIM_RESOURCE_PATH", resource_path),
            SetLaunchConfiguration("sigterm_timeout", "10"),
            SetLaunchConfiguration("sigkill_timeout", "5"),
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
            DeclareLaunchArgument("fault_delay_seconds", default_value="0.35"),
            DeclareLaunchArgument("headless", default_value="true"),
            OpaqueFunction(function=_launch_runtime),
        ]
    )
