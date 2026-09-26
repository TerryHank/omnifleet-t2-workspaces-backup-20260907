#!/usr/bin/env bash
set -eo pipefail

BUNDLE_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
: "${ROS_DISTRO:?请先 source /opt/ros/humble/setup.bash}"
source "/opt/ros/${ROS_DISTRO}/setup.bash"
source "${INSTALL_SETUP:-${BUNDLE_ROOT}/ros2_ws/install/setup.bash}"

# The launch defaults intentionally enable Gazebo GUI, RViz, Foxglove (8796),
# and the one-shot three-lane demonstration. Any launch argument can override it.
exec ros2 launch omnifleet_multi_robot_sim multi_robot_classic.launch.py "$@"
