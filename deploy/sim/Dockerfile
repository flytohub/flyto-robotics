# ROS 2 Jazzy + Gazebo Harmonic, the pairing that matches the real robot.
# Built here rather than pulled ready-made so the simulation runs the same
# flyto-robotics code the TurtleBot3 runs — the only difference between them is
# which topics the mission node is pointed at.
FROM ros:jazzy-ros-base

ENV DEBIAN_FRONTEND=noninteractive \
    ROS_DOMAIN_ID=30 \
    TURTLEBOT3_MODEL=burger \
    PYTHONUNBUFFERED=1

RUN apt-get update && apt-get install -y --no-install-recommends \
        ros-jazzy-ros-gz-sim \
        ros-jazzy-ros-gz-bridge \
        ros-jazzy-geometry-msgs \
        ros-jazzy-nav-msgs \
        ros-jazzy-sensor-msgs \
        python3-pip \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /workspace
COPY . /workspace

# Headless: this exists to measure, not to look at. A run that needs a display
# cannot go in CI, and CI is where a regression in the safety gate should be
# caught rather than on a demo floor.
ENV GZ_SIM_RESOURCE_PATH=/workspace/models:/workspace/worlds

ENTRYPOINT ["/bin/bash", "-lc", "source /opt/ros/jazzy/setup.bash && exec \"$@\"", "--"]
CMD ["bash"]
