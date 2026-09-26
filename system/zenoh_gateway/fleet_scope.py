"""ROS Humble explicit remaps: local TF and controls, shared fleet interfaces."""
import json,os
from pathlib import Path

ROOT=Path('/home/iecme/omnifleet_fleet')

def frame(name):
    return os.environ['OMNIFLEET_ROBOT_ID']+'/'+name

def remaps():
    ns='/'+os.environ['OMNIFLEET_ROBOT_ID']
    rules=[]
    # Nav2 is a backend of the onboard module; remap the whole action family.
    # Foxglove is explicitly allowed to use the public robot-scoped action names
    # directly.  Other launchers retain the historical backend remap.
    if os.environ.get('OMNIFLEET_FOXGLOVE_DIRECT_ACTIONS') != '1':
        for action in ('navigate_to_pose','navigate_through_poses'):
            for suffix in ('','/_action/send_goal','/_action/get_result','/_action/cancel_goal','/_action/feedback','/_action/status'):
                for source in ('/'+action+suffix,ns+'/'+action+suffix):
                    rules.append(('bt_navigator:'+source,ns+'/navigation_backend/'+action+suffix))
    for node in ('controller_server','behavior_server','bt_navigator'):
        for source in ('cmd_vel','/cmd_vel'):
            rules.append((node+':'+source,ns+'/msc/nav_cmd_vel'))
    for source in ('/lidar_odometry/nav_grid_gridmap',ns+'/lidar_odometry/nav_grid_gridmap'):
        rules.append((source,ns+'/map'))
    for name in json.loads((ROOT/'interfaces.json').read_text()):
        # Foxglove direct-topic mode uses the robot-scoped action names
        # explicitly in its caller.  Do not create a second, generic action
        # namespace through remapping.
        if (os.environ.get('OMNIFLEET_FOXGLOVE_DIRECT_ACTIONS') == '1' and
                name.startswith(('/navigate_to_pose/_action/',
                                 '/navigate_through_poses/_action/'))):
            continue
        if not name.startswith(('/fleet/','/robot_104/','/robot_113/')):
            rules.append((name,ns+name))
    return rules

def ros_args():
    args=['--ros-args','-r','__ns:=/'+os.environ['OMNIFLEET_ROBOT_ID']]
    for src,dst in remaps():args+=['-r',src+':='+dst]
    return args

def scoped(description):
    from launch import LaunchDescription
    from launch_ros.actions import PushRosNamespace,SetRemap
    return LaunchDescription([PushRosNamespace('/'+os.environ['OMNIFLEET_ROBOT_ID']),
        *[SetRemap(src=a,dst=b) for a,b in remaps()],*description.entities])
# --- unified scope API appended by namespace adaptation ---
import re as _scope_re
_SCOPE_ID_RE = _scope_re.compile(r'^[A-Za-z][A-Za-z0-9_]*$')
def robot_id(value=None):
    value = os.environ.get('OMNIFLEET_ROBOT_ID', '') if value is None else str(value)
    value = value.strip()
    if not value or not _SCOPE_ID_RE.fullmatch(value) or value in {'fleet','map','tf'}:
        raise ValueError('SCOPE_INVALID: invalid OMNIFLEET_ROBOT_ID')
    return value
def resolve_scope(value=None):
    rid = robot_id(value); ns = '/' + rid
    return {'schema_version': 1, 'robot_identity': rid, 'ros_namespace': ns,
            'frame_prefix': rid + '/', 'source': 'OMNIFLEET_ROBOT_ID',
            'topics': {'raw_lidar': ns+'/rslidar_points', 'pose': ns+'/lidar_odometry/pose',
                       'odom': ns+'/odom', 'tf': ns+'/tf', 'tf_static': ns+'/tf_static',
                       'deskewed_points': ns+'/navigation/deskewed_points', 'map': ns+'/map'},
            'frames': {'map': rid+'/map', 'odom': rid+'/odom', 'base': rid+'/base_link',
                       'lidar': rid+'/rslidar', 'navigation_lidar': rid+'/navigation_lidar'}}
def resolved_scope(value=None): return resolve_scope(value)
def write_resolved(path, value=None):
    target = Path(path); tmp = target.with_suffix(target.suffix + '.tmp')
    tmp.write_text(json.dumps(resolve_scope(value), indent=2, sort_keys=True) + '\n'); tmp.replace(target); return target
if __name__ == '__main__':
    import argparse
    _p = argparse.ArgumentParser(); _p.add_argument('--robot-id'); _p.add_argument('--write'); _a = _p.parse_args()
    _s = resolve_scope(_a.robot_id)
    if _a.write: write_resolved(_a.write, _a.robot_id)
    print(json.dumps(_s, indent=2, sort_keys=True))

