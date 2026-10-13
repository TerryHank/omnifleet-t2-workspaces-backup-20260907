import json,time,uuid
from pathlib import Path
from omnifleet_msc.protocol import request
c=json.load(open('/etc/omnifleet_msc/operator.json'))
def call(path,data):return request(c['coordinator'],'operator',c['key'],path,data,timeout=3)
state=call('/v1/status',{});assert state['motion_enabled'] is False
id_='static-api-'+uuid.uuid4().hex
data={'op':'submit','task_id':id_,'frame_id':'fleet_map','goal':[0,0,0],'priority':1}
first=call('/v1/tasks',data);duplicate=call('/v1/tasks',data);assert duplicate['duplicate'] is True
time.sleep(.8);tasks=call('/v1/tasks',{'op':'list'});assert tasks[id_]['state']=='PENDING' and tasks[id_]['owner'] is None
call('/v1/tasks',{'op':'cancel','task_id':id_});report=call('/v1/report',{})
assert report['tasks'][id_]['state']=='CANCELED' and report['static_only'] is True
result={'task_id':id_,'duplicate_rejected':True,'no_claim_in_static_mode':True,'test_task_canceled':True,'report_available':True}
Path('/home/iecme/robot_backups/msc_shared_map_dsh_20260909/task-api.json').write_text(json.dumps(result,indent=2))
print(json.dumps(result))
