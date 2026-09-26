"""Apply the user-confirmed stationary scan relation using current local map poses."""
import json,argparse
from pathlib import Path
from omnifleet_msc.protocol import request
from omnifleet_msc.geometry import compose,inverse
parser=argparse.ArgumentParser();parser.add_argument('registration');parser.add_argument('--reference',default='113 map / user-confirmed scan overlay');args=parser.parse_args()
registration=Path(args.registration)
config=json.load(open('/etc/omnifleet_msc/operator.json'))
def call(path,data):return request(config['coordinator'],'operator',config['key'],path,data,timeout=3)
state=call('/v1/status',{})
assert state['motion_enabled'] is False
robots=state['robots']
for robot in robots.values():
 assert robot['online'] and robot['control_gate_ready']
 assert robot['stopped_seconds']>.5 and robot['pose_age']<.8
 assert robot.get('pose_stationary') is True
candidate=json.loads(registration.read_text())['candidates'][0]
relative=[*candidate['translation'],candidate['yaw']]
transform=compose(compose(robots['robot_113']['local_pose'],relative),inverse(robots['robot_104']['local_pose']))
call('/v1/command',{'op':'alignment','robot_id':'robot_113','transform':[0,0,0],'verified':True,'reference':args.reference})
result=call('/v1/command',{'op':'alignment','robot_id':'robot_104','transform':transform,'verified':True,'reference':args.reference})
(registration.parent/'aligned-status.json').write_text(json.dumps(result,ensure_ascii=False,indent=2))
print(json.dumps({'transform_104_map_to_113_map':transform,'motion_enabled':result['motion_enabled'],
 'robot_poses':{r:s['fleet_pose'] for r,s in result['robots'].items()},'distance':result['current_robot_distance']}))
