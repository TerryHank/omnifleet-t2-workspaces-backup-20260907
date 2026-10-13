import json,shutil
from pathlib import Path
backup=Path('/home/iecme/robot_backups/msc_shared_map_dsh_20260909');backup.mkdir(exist_ok=True)
for name in ('agent','coordinator'):
 path=Path('/etc/omnifleet_msc')/(name+'.json')
 if not path.exists():continue
 target=backup/(name+'-before-shared-map.json')
 if not target.exists():shutil.copy2(path,target);target.chmod(0o600)
 data=json.loads(path.read_text());data['require_shared_map']=True;data['require_obstacles']=True
 if name=='coordinator':data['measurement_source']='onboard MOLA localization and encoder feedback; no external ground truth'
 if name=='agent':assert data.get('allow_fleet_motion',False) is False
 else:assert data.get('allow_motion',False) is False
 path.write_text(json.dumps(data,indent=2));path.chmod(0o600)
 print(name,'shared map required, movement remains locked')
