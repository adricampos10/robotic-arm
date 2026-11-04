# ROS 2 Jazzy image with the ABB IRB 1200 description, ros2_control and MoveIt.
# Build from the repository root:
#   docker build -f docker/Dockerfile -t robotic-arm:jazzy .
FROM ros:jazzy

ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update && apt-get install -y --no-install-recommends \
      git \
      python3-pytest \
      ros-jazzy-ros2-control \
      ros-jazzy-ros2-controllers \
      ros-jazzy-angles \
      ros-jazzy-xacro \
      ros-jazzy-robot-state-publisher \
      ros-jazzy-moveit \
      ros-jazzy-launch-testing-ament-cmake \
    && rm -rf /var/lib/apt/lists/*

# Upstream packages, pinned to known commits:
# - abb_ros2: only the robot description and the MoveIt config. The EGM
#   driver for the real robot (abb_hardware_interface) needs extra libraries
#   and is not built yet.
# - topic_based_ros2_control: the hardware plugin that talks to Isaac Sim.
#   It has no Jazzy arm64 binary package, so I build it from source.
ARG ABB_ROS2_COMMIT=e6486f7409073cdb2a10d6878bb50191dd50172b
ARG TOPIC_BASED_COMMIT=aa9f9ef9525a60f8becea322537dd1d6d1bf9123
RUN git clone https://github.com/PickNikRobotics/abb_ros2.git /tmp/abb_ros2 \
    && git -C /tmp/abb_ros2 checkout ${ABB_ROS2_COMMIT} \
    && mkdir -p /opt/abb_ws/src \
    && cp -r /tmp/abb_ros2/abb_resources \
             /tmp/abb_ros2/robot_specific_config/abb_irb1200_support \
             /tmp/abb_ros2/robot_specific_config/abb_irb1200_5_90_moveit_config \
             /opt/abb_ws/src/ \
    && rm -rf /tmp/abb_ros2 \
    && git clone https://github.com/PickNikRobotics/topic_based_ros2_control.git \
             /opt/abb_ws/src/topic_based_ros2_control \
    && git -C /opt/abb_ws/src/topic_based_ros2_control checkout ${TOPIC_BASED_COMMIT} \
    && . /opt/ros/jazzy/setup.sh \
    && cd /opt/abb_ws && colcon build --event-handlers console_direct- \
         --cmake-args -DBUILD_TESTING=OFF \
    && rm -rf build log

# My packages.
COPY ros2_ws/src /ws/src
RUN . /opt/abb_ws/install/setup.sh \
    && cd /ws && colcon build --event-handlers console_direct-

# Each container only discovers ROS nodes on its own host, so two runs on the
# same machine don't see each other's topics. Isaac Sim on the same host
# (with --network host) is still visible.
ENV ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST

COPY docker/entrypoint.sh /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
