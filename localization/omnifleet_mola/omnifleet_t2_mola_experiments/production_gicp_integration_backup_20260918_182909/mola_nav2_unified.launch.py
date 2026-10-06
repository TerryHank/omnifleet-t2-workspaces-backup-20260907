"""Unified T2 launch with selectable pure-MOLA and REP-105 profiles."""
from pathlib import Path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction, RegisterEventHandler, EmitEvent
from launch.event_handlers import OnProcessExit, OnProcessStart
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
    map_reload_mode=LaunchConfiguration('map_reload_mode').perform(context).strip().lower() in {'1','true','yes','on'}
    auto_relocalization=LaunchConfiguration('auto_relocalization').perform(context).strip().lower() in {'1','true','yes','on'}
    startup_mode=LaunchConfiguration('startup_mode').perform(context).strip()
    timing_trace=LaunchConfiguration('timing_trace').perform(context).strip()
    timing_trace_dir=LaunchConfiguration('timing_trace_dir').perform(context).strip()
    if startup_mode not in {'saved_map','mapping'}:
        raise RuntimeError(f'unknown startup mode: {startup_mode}')
    if startup_mode=='mapping':
        map_path=''
        initial_pose=''
        auto_relocalization=False
    elif not map_path:
        raise RuntimeError('saved_map mode requires a nonempty map_path')
    if profile not in PROFILES:raise RuntimeError(f'unknown architecture profile: {profile}')
    if map_path and not Path(map_path).expanduser().is_file():
        raise RuntimeError(f'common .mm map does not exist: {map_path}')
    command=['ros2','launch','omnifleet_t2_mola_experiments','mola_no_wheel.launch.py',
             'profile:='+profile,'map_path:='+(map_path or ' '),'map_dir:='+map_dir]
    command.append('map_reload_mode:='+('true' if map_reload_mode else 'false'))
    command.extend(['timing_trace:='+timing_trace,'timing_trace_dir:='+timing_trace_dir])
    if initial_pose:command.append('initial_pose:='+initial_pose)
    mola=ExecuteProcess(name='mola_gate',output='screen',cmd=command)
    nav2=Node(package='omnifleet_t2_mola_experiments',executable='nav2_readiness_gate.py',output='screen',arguments=['--wait-timeout','0'])
    cleanup=Node(package='omnifleet_t2_mola_experiments',executable='cleanup_unified_application_nodes.py',output='screen')
    auto=None
    if auto_relocalization and not map_reload_mode and not initial_pose:
        script=Path('/home/iecme/omnifleet_fleet/relocalization/mola_auto_relocalize.sh')
        if not script.is_file():
            raise RuntimeError(f'MOLA auto-relocalization helper does not exist: {script}')
        auto=ExecuteProcess(name='mola_auto_relocalization',output='screen',cmd=['bash',str(script),map_path])
    def after_auto(event,_context):
        return [nav2] if event.returncode==0 else [EmitEvent(event=Shutdown(reason='MOLA auto-relocalization failed'))]
    def after_cleanup(event,_context):
        if event.returncode!=0:
            return [EmitEvent(event=Shutdown(reason='application cleanup failed'))]
        if auto is None:
            return [mola, RegisterEventHandler(OnProcessStart(target_action=mola,on_start=[nav2]))]
        return [mola, RegisterEventHandler(OnProcessStart(target_action=mola,on_start=[auto])), RegisterEventHandler(OnProcessExit(target_action=auto,on_exit=after_auto))]
    return [RegisterEventHandler(OnProcessExit(target_action=cleanup,on_exit=after_cleanup)),
            RegisterEventHandler(OnProcessExit(target_action=mola,on_exit=[EmitEvent(event=Shutdown(reason='MOLA process exited'))])),cleanup]

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('profile',default_value='simple_direct',description='simple_direct or simple_rep105'),
        DeclareLaunchArgument('startup_mode',default_value='saved_map',description='saved_map: navigate on loaded map; mapping: build a new map while navigating'),
        DeclareLaunchArgument('map_path',default_value=DEFAULT_MAP),
        DeclareLaunchArgument('map_dir',default_value='/home/iecme/maps'),
        DeclareLaunchArgument('initial_pose',default_value=''),
        DeclareLaunchArgument('map_reload_mode',default_value='true',description='Use the previous active MOLA map-reload and local-ICP startup path'),
        DeclareLaunchArgument('auto_relocalization',default_value='false',description='Run the experimental global MOLA relocalization helper'),
        DeclareLaunchArgument('timing_trace',default_value='false'),
        DeclareLaunchArgument('timing_trace_dir',default_value='/home/iecme/.local/share/omnifleet_t2/mola-timing'),
        OpaqueFunction(function=_start),
    ])


_unscoped_generate = generate_launch_description
def generate_launch_description():
    from fleet_scope import scoped
    return scoped(_unscoped_generate())
