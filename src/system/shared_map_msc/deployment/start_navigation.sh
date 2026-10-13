#!/bin/bash
set -e
cd /home/iecme/omnifleet_ws/src/system/shared_map_msc
source /home/iecme/omnifleet_ws/src/system/omnifleet_fleet/env.bash
exec ros2 launch omnifleet_t2_mola_experiments nav2_direct.launch.py odom_topic:=/$OMNIFLEET_ROBOT_ID/lidar_odometry/pose obstacle_topic:=/$OMNIFLEET_ROBOT_ID/navigation/deskewed_points obstacle_clearing:=false
