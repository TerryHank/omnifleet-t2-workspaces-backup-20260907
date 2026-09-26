import asyncio,json,time
from pathlib import Path
import websockets
async def main():
 topics=set()
 async with websockets.connect('ws://127.0.0.1:8765',subprotocols=['foxglove.websocket.v1'],max_size=8_000_000) as ws:
  end=time.monotonic()+5
  while time.monotonic()<end:
   try:message=await asyncio.wait_for(ws.recv(),timeout=max(.1,end-time.monotonic()))
   except asyncio.TimeoutError:break
   if not isinstance(message,str):continue
   data=json.loads(message)
   if data.get('op')=='advertise':topics.update(c['topic'] for c in data['channels'])
 required={'/fleet/map','/fleet/status','/fleet/report','/fleet/robots','/omnifleet_t2/diagnostics/answer'}
 result={'required':sorted(required),'advertised':sorted(required&topics),'missing':sorted(required-topics)}
 Path('/home/iecme/robot_backups/msc_shared_map_dsh_20260909/foxglove-topics.json').write_text(json.dumps(result,indent=2))
 print(json.dumps(result));assert not result['missing']
asyncio.run(main())
