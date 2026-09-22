# Install and connect a standard ROS 2 robot

This is the current installation model.

Flyto2-specific software is installed on an **external AI Space computer**, not
on the TurtleBot3 Raspberry Pi.

## 1. Prepare the robot from upstream sources

Install the robot using its normal ROS 2 documentation.

For the lab TurtleBot3 that means, at minimum:

- ROS 2 Jazzy;
- TurtleBot3 bringup packages;
- the correct lidar driver;
- camera driver when a camera is attached;
- robot_state_publisher / TF / odometry;
- Nav2 and SLAM Toolbox when navigation/mapping is required.

Do not install a Flyto2 credential, job runner, gateway, scheduler or task
database on the robot.

Example systemd files in `deploy/native_ros2/` use upstream packages only.
They are site examples, not a required Flyto runtime.

## 2. Verify the native robot

Before Flyto2 is involved, verify from a ROS 2 shell:

```bash
ros2 node list
ros2 topic list -t
ros2 topic echo /odom --once
ros2 topic echo /scan --once --qos-reliability best_effort
ros2 topic type /tf
ros2 topic type /tf_static
ros2 action list -t
```

For navigation, the expected standard action surface may include:

```text
/navigate_to_pose  nav2_msgs/action/NavigateToPose
/drive_on_heading  nav2_msgs/action/DriveOnHeading
/backup            nav2_msgs/action/BackUp
/spin              nav2_msgs/action/Spin
```

Do not manufacture a localization PASS by supplying a fake AMCL initial pose.
In an unknown environment use a real localization/mapping procedure.

## 3. Verify zero Flyto2 runtime on the robot

The accepted robot should satisfy:

```bash
systemctl list-unit-files | grep -i flyto
systemctl --type=service --all | grep -i flyto
docker ps
```

The first two commands should return no Flyto service. No Flyto credential or
Flyto-specific daemon/package should exist on the robot.

Historical ROS logs may contain the old hostname or prior Flyto experiment
names; logs are evidence, not runtime.

## 4. Prepare the external computer

On the Mac, laptop, Steam Deck, mini PC, or AI Space computer that can reach the
robot's ROS graph:

```bash
python3 -m venv .venv
.venv/bin/pip install ./flyto_robotics-0.1.0-py3-none-any.whl
```

The production wheel contains external ROS/lab/evidence tools. It deliberately
does not install the old Pi appliance commands.

Connect the computer to the robot through an ordinary ROS 2 mechanism:

- DDS on the same network;
- Zenoh;
- rosbridge when appropriate;
- another generic ROS 2 transport.

Network topology is site configuration, not workflow data.

## 5. Discover and approve capabilities

The external adapter inspects exact standard ROS 2 topic/action names and
message types.

Discovery means "this interface exists". It does not mean "Flyto2 may use it".

Flyto2 records a capability declaration, then the normal approval, permission,
policy, bounds and assignment gates decide whether it is usable.

## 6. Physical acceptance

Only in a physically cleared area:

1. dispatch one bounded movement from the external computer;
2. prove movement independently with odometry;
3. observe LiDAR/camera evidence;
4. cancel or interrupt and confirm a real safe stop;
5. recover/continue if the task requires it;
6. send execution and observation evidence to Cloud;
7. let Cloud verify the original objective;
8. only then accept War Room `Completed`.

Action-server success alone is never sufficient.

## Historical installation material

The retired Pi-appliance implementation is not present in the current source
tree. Historical behavior and migration evidence are available in Git history
and dated handoffs only.

Current installation requires no Flyto2 package, credential, gateway, runner,
or lifecycle manager on the robot.
