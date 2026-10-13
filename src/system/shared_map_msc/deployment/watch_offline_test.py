import json,time
from pathlib import Path
from omnifleet_msc.protocol import request
c=json.load(open('/etc/omnifleet_msc/operator.json'));samples=[]
start=time.monotonic()
while time.monotonic()-start<7:
    s=request(c['coordinator'],'operator',c['key'],'/v1/status',{},timeout=2)
    r=s['robots']['robot_104'];samples.append({'time':time.time(),'online':r['online'],'age':r.get('heartbeat_age'),'estop':r.get('estop'),'velocity':r.get('velocity')})
    time.sleep(.1)
assert any(not s['online'] for s in samples),'offline state never observed'
assert samples[-1]['online'],'robot did not return online'
Path('/home/iecme/robot_backups/msc_v1_20260909/evidence/offline-samples.json').write_text(json.dumps(samples,indent=2))
print(json.dumps({'first_offline_at':next(s['time'] for s in samples if not s['online']),'final_online':samples[-1]['online'],'samples':len(samples)}))
