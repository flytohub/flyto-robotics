#!/usr/bin/env python3
"""Run ROBOTIS's own bringup, but shut the whole group down if any node dies.

`ros2 launch turtlebot3_bringup robot.launch.py` starts three processes —
robot_state_publisher, the lidar driver, and turtlebot3_ros (the OpenCR
bridge: it is what publishes /odom and relays /cmd_vel to the wheels). Its
default behaviour when one of the three exits unexpectedly is to log it and
keep the other two running. That is the wrong default here.

turtlebot3_ros can lose its very first handshake with the OpenCR board: the
serial port enumerates before the board's own firmware has finished booting,
the connection attempt fails, and the process aborts. This is not rare — it
happened on a real cold boot on 2026-08-07. Because the launch group survives
that one node's death, systemd sees the wrapping process as continuously
"active" and its Restart=always never fires. The robot is left with no
odometry and no cmd_vel path until someone notices and restarts it by hand.

The fix belongs here rather than in ExecStartPre: no bounded wait can
guarantee the OpenCR handshake succeeds, because there is no safe way to probe
its readiness without duplicating ROBOTIS's own connection protocol. What can
be guaranteed is that a failure gets NOTICED and the whole group comes down
cleanly — so systemd's existing Restart=always (already configured for this
exact case) gets the chance to do its job. The handshake is much more likely
to succeed a few seconds into a restart than on the very first attempt right
after power-on, which is exactly what a manual restart confirmed the same day.

This wraps rather than forks ROBOTIS's launch file, so a future upstream
change to robot.launch.py is picked up automatically instead of drifting from
a copy.
"""

from ament_index_python.packages import get_package_share_directory
from launch.actions import EmitEvent, IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch import LaunchDescription


def generate_launch_description():
    upstream = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            get_package_share_directory("turtlebot3_bringup") + "/launch/robot.launch.py"
        )
    )

    def shut_down_on_exit(event, context):
        process_name = event.process_name or "a bringup process"
        print(
            f"turtlebot3_bringup_supervised: {process_name} exited "
            f"(code {event.returncode}); bringing the whole group down so "
            "systemd's Restart=always can start it clean rather than run on "
            "with a missing sensor or a dead cmd_vel path.",
            flush=True,
        )
        return [EmitEvent(event=Shutdown(reason=f"{process_name} exited"))]

    # Matched by target_action=None: this must catch turtlebot3_ros, the lidar
    # driver, or robot_state_publisher — whichever one goes down. There is no
    # single Node action to name, because the three live inside the included
    # launch description, not this file; None means "any process this launch
    # is tracking."
    shutdown_on_any_exit = RegisterEventHandler(
        OnProcessExit(target_action=None, on_exit=shut_down_on_exit)
    )

    return LaunchDescription([upstream, shutdown_on_any_exit])
