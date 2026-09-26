"""Identity alignment after both MOLA instances localize in the identical saved-map frame."""
import json,time
from pathlib import Path
from omnifleet_msc.protocol import request
c=json.load(open('/etc/omnifleet_msc/operator.json'))
def call(path,data):return request(c['coordinator'],'operator',c['key'],path,data,timeout=3)
s=call('/v1/status',{});assert s['motion_enabled'] is False
for robot,state in s['robots'].items():
 assert state['online'] and state.get('pose_age',99)<.8 and state.get('velocity')==[0.,0.],robot
 call('/v1/command',{'op':'alignment','robot_id':robot,'verified':True,'transform':[0.,0.,0.],
      'reference':'Shared saved map foxglove_map.mm SHA256 4c08be97f06678b8a20d4233935cb907661fe2c3bca4e19f6c0f641254d8cbe3; initialized by stationary scan matching'})
s=call('/v1/status',{})
Path('/home/iecme/robot_backups/msc_v1_architecture_20260909/shared-alignment.json').write_text(json.dumps(s,indent=2))
print(json.dumps({'map':'foxglove_map.mm','poses':{r:s['fleet_pose'] for r,s in s['robots'].items()},'motion_enabled':s['motion_enabled']}))
