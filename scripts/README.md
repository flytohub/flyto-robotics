# Scripts

Scripts in this directory are external development, simulation, evidence, and
acceptance tools. They are run from a workstation, CI worker, container, or
other execution host; they are not a TurtleBot3 installation payload.

`native_ros2_soak_probe.py` is intentionally streamed to a robot over standard
input during acceptance. It leaves no Flyto2 package, credential, daemon, or
task state on the robot.

Robot-side startup belongs to upstream ROS 2 / TurtleBot3 / Nav2 / SLAM
configuration. Do not add a Flyto-specific scheduler, gateway, job runner, or
credential provisioning path here.
