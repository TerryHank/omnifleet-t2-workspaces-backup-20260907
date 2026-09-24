from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


NAVIGATION_NODES = [
    "controller_server",
    "planner_server",
    "behavior_server",
    "bt_navigator",
]


def generate_launch_description():
    share = Path(get_package_share_directory("omnifleet_planner"))
    params_file = LaunchConfiguration("params_file")
    use_sim_time = LaunchConfiguration("use_sim_time")
    common = {
        "parameters": [params_file, {"use_sim_time": use_sim_time}],
        "output": "screen",
    }
    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("autostart", default_value="true"),
        DeclareLaunchArgument(
            "params_file",
            default_value=str(share / "config" / "nav2_t2.yaml"),
        ),
        Node(
            package="nav2_controller",
            executable="controller_server",
            name="controller_server",
            remappings=[("cmd_vel", "/msc/nav_cmd_vel")],
            **common,
        ),
        Node(
            package="nav2_planner",
            executable="planner_server",
            name="planner_server",
            **common,
        ),
        Node(
            package="nav2_behaviors",
            executable="behavior_server",
            name="behavior_server",
            remappings=[("cmd_vel", "/msc/nav_cmd_vel")],
            **common,
        ),
        Node(
            package="nav2_bt_navigator",
            executable="bt_navigator",
            remappings=[("/cmd_vel", "/msc/nav_cmd_vel"),
                        ("navigate_to_pose", "navigation_backend/navigate_to_pose"),
                        ("navigate_through_poses", "navigation_backend/navigate_through_poses"),
                        ("goal_pose", "navigation_backend/goal_pose")],
            name="bt_navigator",
            **common,
        ),
        Node(
            package="nav2_lifecycle_manager",
            executable="lifecycle_manager",
            name="lifecycle_manager_navigation",
            output="screen",
            parameters=[{
                "use_sim_time": use_sim_time,
                "autostart": ParameterValue(
                    LaunchConfiguration("autostart"), value_type=bool
                ),
                "node_names": NAVIGATION_NODES,
            }],
        ),
    ])
