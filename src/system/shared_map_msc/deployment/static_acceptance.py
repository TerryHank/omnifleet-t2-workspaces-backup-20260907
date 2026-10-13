import json,time
from pathlib import Path
from omnifleet_msc.protocol import request
config=json.load(open('/etc/omnifleet_msc/operator.json'))
root=Path('/home/iecme/robot_backups/msc_v1_20260909/evidence')
def call(path,data):return request(config['coordinator'],'operator',config['key'],path,data,timeout=3)
def wait(predicate):
 end=time.monotonic()+12
 while time.monotonic()<end:
  state=call('/v1/status',{})
  if predicate(state):return state
  time.sleep(.2)
 raise AssertionError('static acceptance timeout')
state=wait(lambda s:all(r['online'] and r.get('control_gate_ready') and r.get('nav_ready') for r in s['robots'].values()))
assert state['motion_enabled'] is False
before=state
try:
 call('/v1/command',{'op':'start','mission_id':'static-rejection-check','formation':'column','waypoints':[[0,0,0]]})
 raise AssertionError('motion lock was bypassed')
except ValueError as e:assert 'locked' in str(e)
call('/v1/command',{'op':'stop'})
stopped=wait(lambda s:s['stop_confirmed'] and all(r.get('estop') for r in s['robots'].values()))
result={'motion_lock_rejected_start':True,'stopped':{r:{k:s.get(k) for k in ('estop','velocity','velocity_age','control_epoch','applied_command_seq','stopped_seconds')} for r,s in stopped['robots'].items()},'before_agent_boots':{r:s['agent_boot'] for r,s in before['robots'].items()}}
(root/'static-stop.json').write_text(json.dumps(result,ensure_ascii=False,indent=2));print(json.dumps(result,ensure_ascii=False))
