"""Fleet transport adapter. Navigation and cmd_vel ownership are onboard."""
import argparse,copy,json,math,threading,time,uuid
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from std_msgs.msg import String
from omnifleet_navigation_interfaces.action import ExecuteNavigation
from omnifleet_navigation_interfaces.msg import NavigationPoint
from .protocol import request

class Agent(Node):
    def __init__(self,config):
        super().__init__('msc_agent_'+config['robot_id'])
        self.config=config;self.robot=config['robot_id'];self.boot=uuid.uuid4().hex
        self.lock=threading.Lock();self.closed=False;self.local={};self.local_at=0.
        self.mailbox=None;self.sequence=0;self.error='';self.last_key=None;self.pending=None;self.retry_at=0.
        self.handles={};self.latest_command={};self.permission_epoch=None;self.navigation_result=None
        self.output=self.create_publisher(String,'/'+self.robot+'/navigation/fleet_overlay',1)
        self.client=ActionClient(self,ExecuteNavigation,'/'+self.robot+'/navigation/execute')
        self.create_subscription(String,'/'+self.robot+'/navigation/local_status',self.on_local,10)
        self.create_timer(.05,self.tick)
        self.network=threading.Thread(target=self.network_loop,daemon=True);self.network.start()

    def on_local(self,message):
        try:data=json.loads(message.data)
        except ValueError:return
        if data.get('robot_id')!=self.robot:return
        with self.lock:
            if data.get('fleet_permission_epoch')!=self.permission_epoch:
                self.permission_epoch=data.get('fleet_permission_epoch');self.error='';self.pending=None
            self.local=data;self.local_at=time.monotonic()

    def network_loop(self):
        while not self.closed:
            try:
                with self.lock:state=copy.deepcopy(self.local);age=time.monotonic()-self.local_at
                if not state:time.sleep(.1);continue
                for key in ('pose_age','velocity_age','obstacle_age'):state[key]=state.get(key,99.)+age
                if age>.6:state.update(nav_ready=False,control_gate_ready=False,health='LOCAL_MODULE_UNAVAILABLE')
                if self.error:state['goal_error']=self.error
                if self.navigation_result:state['navigation_result']=copy.deepcopy(self.navigation_result)
                self.sequence+=1
                response=request(self.config['coordinator'],self.robot,self.config['key'],'/v1/heartbeat',{'boot':self.boot,'seq':self.sequence,'state':state})
                packet={'robot_id':self.robot,'boot':self.boot,'sequence':self.sequence,'received_at':time.time(),'response':response}
                with self.lock:self.mailbox=packet
            except Exception as error:
                with self.lock:self.mailbox={'robot_id':self.robot,'error':str(error)}
            time.sleep(.2)

    def tick(self):
        with self.lock:packet=self.mailbox;self.mailbox=None;state=copy.deepcopy(self.local)
        if packet is not None:
            self.output.publish(String(data=json.dumps(packet)))
            command=packet.get('response',{}).get('command',{})
            if command:self.latest_command=command
            if command.get('kind')=='navigate':
                key=(command.get('epoch',''),int(command.get('seq',0)))
                if key!=self.last_key:self.pending=command
            elif command:self.pending=None
        if not self.pending or time.monotonic()<self.retry_at:return
        command=self.pending;key=(command.get('epoch',''),int(command.get('seq',0)))
        if not self.config.get('allow_fleet_motion',False):self.error='fleet motion disabled; local navigation remains available';self.last_key=key;self.pending=None;return
        if state.get('local_override') or not state.get('fleet_enabled'):
            self.error='local ownership active; explicit fleet release required';self.last_key=key;self.pending=None;return
        if not self.client.server_is_ready():self.error='local navigation module unavailable';return
        if key in self.handles:return
        goal=ExecuteNavigation.Goal();goal.task_id=command.get('task_id') or command.get('mission_id') or key[0]
        goal.command_epoch=key[0];goal.revision=key[1];goal.source=ExecuteNavigation.Goal.FLEET
        goal.single_point='route' not in command;goal.pass_radius=float(command.get('pass_radius',.25))
        route=command.get('route',[command['target']])
        for i,value in enumerate(route):
            attrs=value if isinstance(value,dict) else {};pose=attrs.get('pose',value)
            p=NavigationPoint();p.name=attrs.get('name',f'协同航点{i+1}')
            p.pose.header.frame_id=self.robot+'/map';p.pose.header.stamp=self.get_clock().now().to_msg()
            p.pose.pose.position.x=float(pose[0]);p.pose.pose.position.y=float(pose[1])
            p.pose.pose.orientation.z=math.sin(pose[2]/2);p.pose.pose.orientation.w=math.cos(pose[2]/2)
            p.kind=NavigationPoint.STOP if i==len(route)-1 or attrs.get('kind')=='stop' else NavigationPoint.PASS
            p.dwell_seconds=float(attrs.get('dwell_seconds',0.));goal.waypoints.append(p)
        self.handles[key]=None;self.pending=None;self.last_key=key;self.error=''
        future=self.client.send_goal_async(goal)
        def accepted(f):
            try:
                h=f.result()
                if not h.accepted:raise RuntimeError('local module rejected fleet task')
                self.handles[key]=h
                latest=self.latest_command
                if latest.get('kind')!='navigate' or (latest.get('epoch',''),int(latest.get('seq',0)))!=key:h.cancel_goal_async()
                def finished(result):
                    self.handles.pop(key,None)
                    try:
                        r=result.result().result
                        self.navigation_result={'task_id':goal.task_id,'command_epoch':key[0],'revision':key[1],'code':r.code,'stopped_verified':r.code in ('CANCELED','PREEMPTED_BY_LOCAL')}
                        if key==self.last_key and not r.success:
                            self.error=r.code+': '+r.message
                            if r.code in ('BUSY','BUSY_CANCELING','NOT_READY'):
                                self.pending=command;self.retry_at=time.monotonic()+.5
                    except Exception as error:self.error=str(error)
                h.get_result_async().add_done_callback(finished)
            except Exception as error:self.handles.pop(key,None);self.error=str(error)
        future.add_done_callback(accepted)

def main():
    p=argparse.ArgumentParser();p.add_argument('--config',required=True);args,ros=p.parse_known_args()
    with open(args.config) as f:config=json.load(f)
    rclpy.init(args=ros);node=Agent(config)
    try:rclpy.spin(node)
    except (KeyboardInterrupt,rclpy.executors.ExternalShutdownException):pass
    finally:
        node.closed=True;node.network.join(timeout=2)
        node.destroy_node();rclpy.try_shutdown()
