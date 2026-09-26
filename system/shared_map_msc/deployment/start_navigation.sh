#!/bin/bash
set -e
cd /home/iecme/msc_v1_ws
source /home/iecme/omnifleet_fleet/env.bash
exec ros2 launch omnifleet_t2_mola_experiments nav2_direct.launch.py odom_topic:=/$OMNIFLEET_ROBOT_ID/lidar_odometry/pose obstacle_topic:=/$OMNIFLEET_ROBOT_ID/navigation/deskewed_points obstacle_clearing:=false
