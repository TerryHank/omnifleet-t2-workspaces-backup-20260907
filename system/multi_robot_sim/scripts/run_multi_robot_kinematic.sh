#!/usr/bin/env bash
set -eo pipefail

BUNDLE_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
: "${ROS_DISTRO:?请先 source /opt/ros/humble/setup.bash}"
source "/opt/ros/${ROS_DISTRO}/setup.bash"
source "${INSTALL_SETUP:-${BUNDLE_ROOT}/ros2_ws/install/setup.bash}"

exec ros2 launch omnifleet_multi_robot_sim multi_robot_kinematic.launch.py "$@"
