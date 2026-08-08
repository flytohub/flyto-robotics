# TurtleBot3 Burger virtual robot lab

This lab is the long-running software twin of the physical `flyto-robot.local`
robot. It uses native Apple Silicon virtualization, Ubuntu 24.04 ARM64, ROS 2
Jazzy, Gazebo Harmonic, and the official ROBOTIS Jazzy TurtleBot3 Burger model.
Docker and Paperclip are not used or started.

## Fidelity boundary

The simulator matches the physical control boundary and the important motion
parameters:

- Burger wheel separation `0.160 m` and wheel radius `0.033 m`;
- official mass, inertia, caster, wheel collision, and differential drive model;
- LDS scan geometry: 360 samples, `0.12–3.5 m`, 5 Hz, and Gaussian noise;
- IMU at 200 Hz with angular velocity and acceleration noise;
- Gazebo physics at a `0.001 s` step, 150 solver iterations, real gravity,
  floor friction, contact, movable mass, walls, and obstacles;
- the same `/flyto/cmd_vel`, `/flyto/odom`, and `/flyto/scan` topics consumed
  by `serve-delivery --backend ros2` on the real robot;
- the same Flyto mission contracts, sensor stabilization gate, obstacle stop,
  stale-sensor stop, final zero command, and evidence writer.

The OpenCR firmware, Dynamixel electrical behavior, battery voltage, USB serial
drivers, real wheel wear, and real Wi-Fi are hardware-only effects. Gazebo is
therefore a high-fidelity software and physics twin, not proof that those
physical components work. The full microSD recovery image is also still a
separate backup task; it is not required to run this lab.

## Start

The first provision downloads Ubuntu and ROS packages and can take 15–30
minutes. Later starts only synchronize the repository and rebuild changed
packages.

```bash
./scripts/run-lima-gazebo.sh
```

The source repository is mounted read-only. A working copy, build output,
runtime logs, and generated secrets stay inside the VM under
`~/.local/share/flyto-robot-gazebo/`. The gateway listens on guest loopback port
`8766`; Lima forwards loopback ports to the Mac.

Use an explicit lab fault when diagnosing fail-safe behavior:

```bash
./scripts/run-lima-gazebo.sh --fault lidar_dropout --no-gateway
./scripts/run-lima-gazebo.sh --fault odometry_freeze --no-gateway
```

The simulator is headless by default. Headless mode uses the same collision,
friction, inertia, LiDAR, IMU, and control physics as the GUI while avoiding a
second Linux desktop and GPU window. This is the safer setting on a 16 GB Mac.

## Verify motion and fail-safe behavior

```bash
./scripts/verify-lima-gazebo.sh
```

Verification checks live odometry, 360-ray LiDAR, IMU, a bounded physical
motion pulse, final velocity, injected LiDAR loss, the independent safety
latch, and post-latch zero actuator commands. It restores the normal gateway
lab when it finishes and writes generated evidence under
`results/virtual-robot/`.

## Stop and release memory

```bash
./scripts/stop-lima-gazebo.sh
```

The stop path publishes zero velocity before terminating the managed ROS
processes, stops the VM, and releases its 4 GiB memory allocation. The VM disk
remains for fast later starts. To remove the disk too, first stop the lab and
then explicitly run `limactl delete flyto-robot-gazebo`; deletion is not part
of the normal scripts.
