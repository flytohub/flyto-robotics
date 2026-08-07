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

OnProcessExit covers a process that dies. On 2026-08-07 turtlebot3_ros also
hung *while alive* — same PID, every topic silent — which nothing here can
see. The bringup_watchdog process below covers that: it pings systemd's
watchdog only while /odom stays fresh, so a silent hang starves the timer and
systemd restarts the whole unit. It must live in this launch group (not a
separate unit) so it shares the service's cgroup and $NOTIFY_SOCKET; being a
tracked process here also means the supervisor catches the watchdog itself
dying, for free.
"""

import sys
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch.actions import (
    EmitEvent,
    ExecuteProcess,
    IncludeLaunchDescription,
    RegisterEventHandler,
)
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

    # Run by module with the interpreter running this launch, cwd'd to the
    # repo this file lives in. flyto_robotics is not pip-installed on the
    # robot at all — the unit loads this file by absolute path, and the
    # delivery service finds the package the same way, via
    # WorkingDirectory=/home/ubuntu/flyto-robotics. Deriving the cwd from
    # __file__ keeps that working no matter what directory the unit or a
    # by-hand `ros2 launch` happens to run from (this exact miss made the
    # first deploy exit 1 with ModuleNotFoundError).
    watchdog = ExecuteProcess(
        cmd=[sys.executable, "-m", "flyto_robotics.bringup_watchdog"],
        name="bringup_watchdog",
        cwd=str(Path(__file__).resolve().parents[1]),
        output="screen",
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

    return LaunchDescription([upstream, watchdog, shutdown_on_any_exit])
