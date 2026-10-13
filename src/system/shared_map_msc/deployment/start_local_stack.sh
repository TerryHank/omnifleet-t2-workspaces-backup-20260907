#!/usr/bin/env bash
set -e
source /home/iecme/omnifleet_ws/src/system/workspace_support/robot_fleet_env.bash
exec ros2 launch omnifleet_t2_mola_experiments mola_nav2_unified.launch.py "$@"
