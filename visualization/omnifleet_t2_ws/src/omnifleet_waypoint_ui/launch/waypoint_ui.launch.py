import os
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package="omnifleet_waypoint_ui",
            executable="waypoint_ui_bridge",
            name="omnifleet_t2_waypoint_ui",
            output="screen",
            parameters=[{
                "frame_id": os.environ["OMNIFLEET_ROBOT_ID"]+"/map",
                "route_file": "/var/lib/omnifleet_t2/missions/waypoints.json",
            }],
        )
    ])
