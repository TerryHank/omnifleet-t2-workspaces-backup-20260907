"""Record deployment checks without exporting authentication secrets."""
import hashlib,json,subprocess,time
from pathlib import Path
root=Path('/home/iecme/msc_v1_ws');agent=json.load(open('/etc/omnifleet_msc/agent.json'))
assert agent['enable_control'] is True and agent.get('allow_fleet_motion',False) is False
services=['omnifleet-msc-agent','omnifleet-t2-chassis']
if agent['robot_id']=='robot_104':services+=['omnifleet-msc-map']
if agent['robot_id']=='robot_113':
 coordinator=json.load(open('/etc/omnifleet_msc/coordinator.json'))
 assert coordinator.get('allow_motion',False) is False
 services+=['omnifleet-msc-coordinator','omnifleet-t2-dsh-diagnostics','omnifleet-t2-nav2-parameters']
units={name:subprocess.check_output(['systemctl','show',name,'-p','MainPID','-p','ActiveState','-p','UnitFileState'],text=True).strip() for name in services}
application='msc-shared-nav-'+agent['robot_id'].split('_')[-1]
units[application]=subprocess.check_output(['systemctl','--user','show',application,'-p','MainPID','-p','ActiveState'],text=True).strip()
if agent['robot_id']=='robot_104':units['msc-local-stack-104']=subprocess.check_output(['systemctl','--user','show','msc-local-stack-104','-p','MainPID','-p','ActiveState'],text=True).strip()
else:units['msc-shared-stack-113']=subprocess.check_output(['systemctl','--user','show','msc-shared-stack-113','-p','MainPID','-p','ActiveState'],text=True).strip()
temporary=[]
names={'capture_alignment.py','test_guard_ros.py','test_agent_actions.py','freeze_agent_test.py','final_static_probe.py','verify_observers.py','msc_readonly_probe',
       'probe_shared_map.py','verify_shared_planning.py','probe_peer_layer.py','verify_static_task_api.py','verify_fleet_diagnostic.py','verify_diagnostic_websocket.mjs','verify_foxglove_topics.mjs'}
for process in Path('/proc').glob('[0-9]*'):
 try:
  args=(process/'cmdline').read_bytes().split(b'\0')
  if any(Path(a.decode(errors='replace')).name in names for a in args):temporary.append(process.name)
 except OSError:pass
assert not temporary,temporary
source_hashes={str(p.relative_to(root)):hashlib.sha256(p.read_bytes()).hexdigest() for p in (root/'src/omnifleet_msc/omnifleet_msc').glob('*.py')}
result={'time':time.time(),'robot':agent['robot_id'],'motion_locked':True,'services':units,'remaining_test_processes':temporary,'source_sha256':source_hashes}
Path('/home/iecme/robot_backups/msc_shared_map_dsh_20260909/installation.json').write_text(json.dumps(result,indent=2))
print(json.dumps({k:v for k,v in result.items() if k!='source_sha256'}))
