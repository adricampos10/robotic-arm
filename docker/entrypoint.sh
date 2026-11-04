#!/bin/bash
# Source ROS 2, the upstream ABB packages and my workspace, then run the command.
set -e
source /opt/ros/jazzy/setup.bash
source /opt/abb_ws/install/setup.bash
source /ws/install/setup.bash
exec "$@"
