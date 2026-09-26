import json
from omnifleet_msc.protocol import request
c=json.load(open('/etc/omnifleet_msc/operator.json'));s=request(c['coordinator'],'operator',c['key'],'/v1/status',{},timeout=3)
print(json.dumps({'shared_map':s.get('shared_map'),'robots':{r:{k:v.get(k) for k in ('online','nav_ready','control_gate_ready','control_mode','health','pose_age','velocity_age','obstacle_age','obstacle_observations_complete','network_error','goal_error','shared_map_ready','runtime_parameters')} for r,v in s['robots'].items()}},ensure_ascii=False))
