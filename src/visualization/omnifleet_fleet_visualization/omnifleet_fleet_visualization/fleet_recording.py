"""Owned lightweight recording started only by a fleet start operation."""
import datetime
import os
from pathlib import Path
import signal
import subprocess
import time


class FleetRecording:
    def __init__(self):
        self.process=None;self.path=None;self.error='';self.log=None;self.stopping=False

    def start(self, robots):
        if self.process and self.process.poll() is None:return
        root=Path.home()/'omnifleet_data/recordings/bags'
        root.mkdir(parents=True,exist_ok=True)
        self.path=root/('fleet_ui_'+datetime.datetime.now().strftime('%Y%m%d_%H%M%S_%f'))
        self.log=open(str(self.path)+'.log','w');self.error='';self.stopping=False
        topics=['/fleet/ui/state','/fleet/status','/fleet/events','/fleet/mission_state']
        for robot in robots:
            topics+=['/'+robot+suffix for suffix in ('/tf','/tf_static','/odom','/plan','/goal_update',
                '/navigation/local_status','/navigation/status','/msc/local_status','/msc/nav_cmd_vel','/msc/cmd_vel_safe')]
        self.process=subprocess.Popen(['nice','-n','10','ros2','bag','record','-s','mcap','-o',str(self.path),*topics],
            stdout=self.log,stderr=subprocess.STDOUT,start_new_session=True)
        deadline=time.monotonic()+5
        while time.monotonic()<deadline:
            if self.process.poll() is not None:raise RuntimeError('fleet recorder exited before start; see '+str(self.path)+'.log')
            if (self.path/'metadata.yaml').exists() or self.path.exists() and any(self.path.glob('*.mcap')):return
            time.sleep(.05)
        self.stop();raise RuntimeError('fleet recorder did not become ready')

    def stop(self):
        if not self.process or self.process.poll() is not None:return
        self.stopping=True;os.killpg(self.process.pid,signal.SIGINT)
        try:self.process.wait(timeout=10)
        except subprocess.TimeoutExpired:
            self.error='recorder stop timed out';os.killpg(self.process.pid,signal.SIGTERM)
        if self.log:self.log.close();self.log=None

    def state(self):
        return {'active':bool(self.process and self.process.poll() is None),'stopping':self.stopping,
                'path':str(self.path) if self.path else '', 'error':self.error}
