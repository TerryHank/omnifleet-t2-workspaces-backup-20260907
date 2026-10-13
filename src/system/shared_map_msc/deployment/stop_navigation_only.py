"""Stop exactly the inspected Nav2 launch; do not target MOLA or base services."""
import os,signal,time
from pathlib import Path
def find():
 result=[]
 for p in Path('/proc').glob('[0-9]*'):
  try:
   argv=(p/'cmdline').read_bytes().split(b'\0')
   if b'nav2_direct.launch.py' in argv and b'launch' in argv and any(Path(os.fsdecode(a)).name=='ros2' for a in argv):result.append(int(p.name))
  except OSError:pass
 return result
ids=find();print('Nav2 launch PIDs:',ids)
for pid in ids:os.kill(pid,signal.SIGINT)
end=time.monotonic()+15
while find() and time.monotonic()<end:time.sleep(.2)
assert not find(),'Nav2 shutdown did not complete; not starting a duplicate'
