# Digital twin of the physical TurtleBot3 (flyto-robot.local).
#
# Same ROS 2 distribution, packages, service arguments, topic names and
# rosbridge endpoint as the robot. Gazebo replaces the hardware bringup; the
# rest is the robot's own stack. See sim/twin/README.md.
FROM ros:jazzy-ros-base

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update \
    && apt-get install -y --no-install-recommends \
        ros-jazzy-turtlebot3 \
        ros-jazzy-turtlebot3-gazebo \
        ros-jazzy-turtlebot3-navigation2 \
        ros-jazzy-nav2-bringup \
        ros-jazzy-navigation2 \
        ros-jazzy-slam-toolbox \
        ros-jazzy-rosbridge-server \
        ros-jazzy-ros-gz-sim \
        ros-jazzy-ros-gz-bridge \
        ros-jazzy-rmw-cyclonedds-cpp \
        ros-jazzy-image-transport-plugins \
        python3-opencv \
        mesa-utils \
        libgl1-mesa-dri \
        xvfb \
        openbox \
        x11vnc \
        novnc \
        websockify \
    && rm -rf /var/lib/apt/lists/*

# The robot's own files, copied verbatim from flyto-robot.local on 2026-10-02.
COPY config/slam_toolbox.yaml /etc/ros/slam_toolbox.yaml
COPY scripts/ros2-laserscan-stabilizer.py /usr/local/libexec/ros2-laserscan-stabilizer.py
COPY scripts/entrypoint.sh /usr/local/bin/twin-entrypoint.sh
# Twin-only pieces: the burger model with the robot's sensors, the world launch,
# the driver stand-ins, and the robot's own description and parameter dumps.
COPY models /opt/flyto-twin/models
COPY launch /opt/flyto-twin/launch
COPY config/real /opt/flyto-twin/config/real
COPY scripts/twin_hardware.py /opt/flyto-twin/scripts/twin_hardware.py
COPY scripts/twin_nav2_params.py /opt/flyto-twin/scripts/twin_nav2_params.py
RUN chmod +x /usr/local/libexec/ros2-laserscan-stabilizer.py /usr/local/bin/twin-entrypoint.sh

# Same environment as every robot service unit.
ENV ROS_DOMAIN_ID=30 \
    RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
    TURTLEBOT3_MODEL=burger \
    LDS_MODEL=LDS-03 \
    LIBGL_ALWAYS_SOFTWARE=1 \
    DISPLAY=:1

# 9090: rosbridge (bound to 127.0.0.1 on the host by compose, like the robot).
# 6080: noVNC view of the Gazebo window.
EXPOSE 9090 6080

ENTRYPOINT ["/usr/local/bin/twin-entrypoint.sh"]
