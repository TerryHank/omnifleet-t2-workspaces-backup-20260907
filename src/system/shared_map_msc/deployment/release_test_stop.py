import json,time
from pathlib import Path
from omnifleet_msc.protocol import request
c=json.load(open('/etc/omnifleet_msc/operator.json'))
def call(path,data):return request(c['coordinator'],'operator',c['key'],path,data,timeout=3)
deadline=time.monotonic()+15
while time.monotonic()<deadline:
 s=call('/v1/status',{})
 if all(r['online'] and r.get('control_gate_ready') and r.get('stopped_seconds',0)>.5 for r in s['robots'].values()):break
 time.sleep(.2)
call('/v1/command',{'op':'release_stop'})
deadline=time.monotonic()+10
while time.monotonic()<deadline:
 s=call('/v1/status',{})
 if all(not r.get('estop') and r.get('control_mode')=='LOCAL' for r in s['robots'].values()):break
 time.sleep(.2)
assert s['motion_enabled'] is False
assert all(r['online'] and r.get('control_gate_ready') and not r.get('estop') and r.get('control_mode')=='LOCAL' and r['velocity']==[0.,0.] for r in s['robots'].values())
Path('/home/iecme/robot_backups/msc_v1_20260909/evidence/final-status.json').write_text(json.dumps(s,ensure_ascii=False,indent=2))
print(json.dumps({'motion_enabled':s['motion_enabled'],'distance':s['current_robot_distance'],'robots':{r:{k:v.get(k) for k in ('online','nav_ready','control_gate_ready','control_mode','estop','velocity','fleet_pose')} for r,v in s['robots'].items()}},ensure_ascii=False))
