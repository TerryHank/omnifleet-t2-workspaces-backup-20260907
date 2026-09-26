"""Unified T2 launch with selectable pure-MOLA and REP-105 profiles."""
from pathlib import Path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction, RegisterEventHandler, EmitEvent
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

PROFILES={'simple_direct','simple_direct_observation','simple_rep105','smoother_direct'}
DEFAULT_MAP='/home/iecme/maps/foxglove_map.mm'

def _start(context):
    profile=LaunchConfiguration('profile').perform(context).strip()
    map_path=LaunchConfiguration('map_path').perform(context).strip()
    map_dir=LaunchConfiguration('map_dir').perform(context).strip()
    initial_pose=LaunchConfiguration('initial_pose').perform(context).strip()
    if profile not in PROFILES:raise RuntimeError(f'unknown architecture profile: {profile}')
    if not map_path or not Path(map_path).expanduser().is_file():
        raise RuntimeError(f'common .mm map does not exist: {map_path}')
    command=['ros2','launch','omnifleet_t2_mola_experiments','mola_no_wheel.launch.py',
             'profile:='+profile,'map_path:='+map_path,'map_dir:='+map_dir]
    if initial_pose:command.append('initial_pose:='+initial_pose)
    mola=ExecuteProcess(name='mola_gate',output='screen',cmd=command)
    nav2=Node(package='omnifleet_t2_mola_experiments',executable='nav2_readiness_gate.py',output='screen',arguments=['--wait-timeout','0'])
    cleanup=Node(package='omnifleet_t2_mola_experiments',executable='cleanup_unified_application_nodes.py',output='screen')
    def after_cleanup(event,_context):
        return [mola,nav2] if event.returncode==0 else [EmitEvent(event=Shutdown(reason='application cleanup failed'))]
    return [RegisterEventHandler(OnProcessExit(target_action=cleanup,on_exit=after_cleanup)),
            RegisterEventHandler(OnProcessExit(target_action=mola,on_exit=[EmitEvent(event=Shutdown(reason='MOLA process exited'))])),cleanup]

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('profile',default_value='simple_direct',description='simple_direct or simple_rep105'),
        DeclareLaunchArgument('map_path',default_value=DEFAULT_MAP),
        DeclareLaunchArgument('map_dir',default_value='/home/iecme/maps'),
        DeclareLaunchArgument('initial_pose',default_value=''),
        OpaqueFunction(function=_start),
    ])
