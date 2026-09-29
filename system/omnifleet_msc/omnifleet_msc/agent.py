from fleet_scope import frame
import argparse,copy,json,math,threading,time,uuid,os
from collections import deque
from pathlib import Path
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.time import Time
from rclpy.qos import QoSProfile,DurabilityPolicy
from geometry_msgs.msg import Twist,PoseStamped
from nav_msgs.msg import Odometry,OccupancyGrid,Path as NavPath
from nav2_msgs.action import NavigateToPose
from lifecycle_msgs.srv import GetState
from rcl_interfaces.srv import GetParameters,ListParameters
from action_msgs.srv import CancelGoal
from action_msgs.msg import GoalStatusArray
from std_msgs.msg import Empty,Bool,String
from tf2_ros import Buffer,TransformListener
from .geometry import pose,compose,inverse,safe_velocity
from .protocol import request
from .storage import read,save

def yaw(q):return math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))

class Agent(Node):
    def __init__(self,config):
        super().__init__('msc_agent_'+config['robot_id']);self.config=config;self.boot=uuid.uuid4().hex
        if not math.isfinite(config.get('safety_clearance',.2)) or config.get('safety_clearance',.2)<.2:raise ValueError('invalid safety clearance')
        self.lock=threading.RLock();self.closed=False;self.network_error='not connected';self.last_response=0.
        self.response={};self.incoming=[];self.sequence=0;self.applied_seq=0;self.command_epoch=''
        self.mode='STARTUP';self.startup_canceled=False;self.startup_done=False;self.startup_stopped=None
        self.fleet_hold=True;self.estop=False;self.manual_time=0.;self.nav_time=0.
        self.manual=[0.,0.];self.nav=[0.,0.];self.velocity=[0.,0.];self.velocity_time=0.
        self.state_path=Path(config.get('state_file',str(Path.home()/'.local/share/omnifleet_msc'/('agent-'+config['robot_id']+'.json'))))
        self.zero_since=None;self.estop_input=False;self.safety_latched=read(self.state_path,{}).get('safety_latched',False)
        self.estop=self.safety_latched
        self.local_pose=None;self.pose_age=99.;self.pose_stamp_ns=0;self.states={};self.pending_states=set();self.nav_status={}
        self.odom_pose=None;self.odom_stamp_ns=0;self.odom_time=0.
        self.active_goal_ids={};self.manual_blocked_goals=set()
        self.goal_handle=None;self.goal=None;self.pending_goal=False;self.goal_error='';self.completed_seq=0
        self.pending_goal_requests={}
        self.cancel_pending=0;self.control_gate_ready=False;self.grid=None;self.grid_time=0.
        self.safety_reason='';self.epoch='';self.process_epoch='';self.map_generation=0;self.pose_time=0.
        self.trace=[];self.planned_path=[];self.own_snapshot={}
        self.pose_window=deque(maxlen=20)
        self.shared_map_status={};self.shared_map_time=0.
        self.shared_grid=None
        self.runtime_parameters={};self.parameter_time=0.;self.parameter_pending=set();self.selection={}
        self.pending_state_calls={};self.pending_parameter_calls={}
        self.obstacles=[];self.obstacle_stamp=0.;self.obstacle_complete=False
        self.contact=None;self.contact_time=0.
        self.machine_boot=Path('/proc/sys/kernel/random/boot_id').read_text().strip()
        self.tf=Buffer();self.listener=TransformListener(self.tf,self)
        self.client=ActionClient(self,NavigateToPose,'/navigate_to_pose')
        self.output=self.create_publisher(Twist,'/msc/cmd_vel_safe',10)
        self.peer_map_pub=self.create_publisher(OccupancyGrid,'/msc/peer_map',QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.create_timer(.5,self.publish_peers)
        self.wp_stop=self.create_publisher(Empty,'/omnifleet_t2/waypoints/stop',10)
        self.heartbeat_pub=self.create_publisher(String,'/msc/local_status',10)
        self.create_subscription(Twist,'/cmd_vel',self.on_manual,1)
        self.create_subscription(Twist,'/msc/nav_cmd_vel',self.on_nav,1)
        self.create_subscription(Odometry,'/odom',self.on_odom,20)
        self.create_subscription(Bool,'/msc/estop',self.on_estop,10)
        self.create_subscription(Bool,'/msc/contact',self.on_contact,10)
        self.create_subscription(String,'/msc/shared_map_status',self.on_shared_map,10)
        self.create_subscription(OccupancyGrid,'/fleet/map',lambda m:setattr(self,'shared_grid',m),
            QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
        for name in ('planner','controller'):
            self.create_subscription(String,'/'+name+'_selector',lambda m,n=name:self.selection.update({n:m.data}),
                QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.parameter_names={
            'controller_server':['controller_plugins','general_goal_checker.xy_goal_tolerance','general_goal_checker.yaw_goal_tolerance',
                'progress_checker.required_movement_radius','progress_checker.movement_time_allowance',
                'FollowPath.max_vel_x','FollowPath.max_vel_theta','FollowPath.BaseObstacle.scale',
                'FollowPathRPP.desired_linear_vel','FollowPathMPPI.vx_max','FollowPathMPPI.wz_max'],
            'planner_server':['planner_plugins'],
            'global_costmap/global_costmap':['global_frame','resolution','footprint','footprint_padding','static_layer.map_topic',
                'inflation_layer.inflation_radius','inflation_layer.cost_scaling_factor'],
            'local_costmap/local_costmap':['global_frame','resolution','footprint_padding','inflation_layer.inflation_radius',
                'inflation_layer.cost_scaling_factor']}
        self.parameter_clients={name:self.create_client(GetParameters,'/'+name+'/get_parameters') for name in self.parameter_names}
        self.parameter_list_clients={name:self.create_client(ListParameters,'/'+name+'/list_parameters') for name in self.parameter_names}
        self.create_timer(5.,self.read_parameters)
        self.create_subscription(OccupancyGrid,'/global_costmap/costmap',self.on_grid,
            QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.create_subscription(OccupancyGrid,'/local_costmap/costmap',self.on_obstacle_grid,
            QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.create_subscription(NavPath,'/plan',self.on_plan,10)
        actions=('navigate_to_pose','navigate_through_poses','follow_path','spin','backup','drive_on_heading','wait')
        for action in actions:
            self.create_subscription(GoalStatusArray,'/'+action+'/_action/status',
                lambda m,a=action:self.on_nav_status(a,m),
                QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.state_clients={name:self.create_client(GetState,'/'+name+'/get_state') for name in ('controller_server','planner_server','bt_navigator')}
        self.cancel_clients=[self.create_client(CancelGoal,'/'+a+'/_action/cancel_goal') for a in actions]
        self.create_timer(.02,self.control_tick);self.create_timer(.2,self.snapshot);self.create_timer(1.,self.discovery)
        self.network=threading.Thread(target=self.network_loop,daemon=True);self.network.start()

    def on_nav(self,msg):self.nav=[msg.linear.x,msg.angular.z];self.nav_time=time.monotonic()
    def on_contact(self,msg):
        self.contact=bool(msg.data);self.contact_time=time.monotonic()
        if self.contact:
            self.safety_latched=True;save(self.state_path,{'safety_latched':True});self.estop=True;self.fleet_hold=True;self.cancel_all()
    def publish_peers(self):
        source=self.shared_grid
        if source is None or source.header.frame_id!='fleet_map':return
        with self.lock:reply=copy.deepcopy(self.response);received=self.last_response
        m=OccupancyGrid();m.header.frame_id='fleet_map';m.header.stamp=self.get_clock().now().to_msg();m.info=copy.deepcopy(source.info)
        m.data=[-1]*(m.info.width*m.info.height);origin=[m.info.origin.position.x,m.info.origin.position.y,yaw(m.info.origin.orientation)]
        for peer in reply.get('peers',[]):
            if peer.get('pose') is None or peer.get('age',99)+time.monotonic()-received>.8:continue
            p=compose(inverse(origin),peer['pose']);r=peer['radius'];res=m.info.resolution
            cx,cy=math.floor(p[0]/res),math.floor(p[1]/res);span=math.ceil(r/res+.71)
            for y in range(max(0,cy-span),min(m.info.height,cy+span+1)):
                for x in range(max(0,cx-span),min(m.info.width,cx+span+1)):
                    if math.hypot((x+.5)*res-p[0],(y+.5)*res-p[1])<=r+res*.71:m.data[y*m.info.width+x]=100
        self.peer_map_pub.publish(m)
    def on_shared_map(self,msg):
        try:self.shared_map_status=json.loads(msg.data);self.shared_map_time=time.monotonic()
        except ValueError:pass
    def read_parameters(self):
        fields={1:'bool_value',2:'integer_value',3:'double_value',4:'string_value',5:'byte_array_value',6:'bool_array_value',7:'integer_array_value',8:'double_array_value',9:'string_array_value'}
        for name,client in self.parameter_clients.items():
            if name in self.parameter_pending or not client.service_is_ready():continue
            list_client=self.parameter_list_clients[name]
            if not list_client.service_is_ready():continue
            self.parameter_pending.add(name);future=list_client.call_async(ListParameters.Request())
            self.pending_parameter_calls[name]=(list_client,future,time.monotonic())
            def listed(f,name=name,client=client):
                try:
                    available=f.result().result.names;names=[k for k in self.parameter_names[name] if k in available]
                    request_=GetParameters.Request();request_.names=names;pending=client.call_async(request_)
                    self.pending_parameter_calls[name]=(client,pending,time.monotonic())
                    def done(result):
                        self.parameter_pending.discard(name);self.pending_parameter_calls.pop(name,None)
                        try:
                            values={k:getattr(v,fields[v.type]) if v.type in fields else None for k,v in zip(names,result.result().values)}
                            self.runtime_parameters[name]={'values':values,'received_at':time.monotonic()}
                        except Exception:self.runtime_parameters.pop(name,None)
                    pending.add_done_callback(done)
                except Exception:
                    self.parameter_pending.discard(name);self.pending_parameter_calls.pop(name,None);self.runtime_parameters.pop(name,None)
            future.add_done_callback(listed)
    def on_odom(self,msg):
        self.velocity=[msg.twist.twist.linear.x,msg.twist.twist.angular.z];self.velocity_time=time.monotonic()
        self.odom_pose=pose([msg.pose.pose.position.x,msg.pose.pose.position.y,yaw(msg.pose.pose.orientation)])
        self.odom_stamp_ns=msg.header.stamp.sec*10**9+msg.header.stamp.nanosec
        self.odom_time=self.velocity_time
        if abs(self.velocity[0])<.02 and abs(self.velocity[1])<.05:
            if self.zero_since is None:self.zero_since=self.velocity_time
        else:self.zero_since=None
    def on_grid(self,msg):
        self.grid=msg;self.grid_time=time.monotonic()
    def on_obstacle_grid(self,msg):
        self.obstacles=[];self.obstacle_complete=False
        if self.local_pose is None:return
        try:
            origin=[msg.info.origin.position.x,msg.info.origin.position.y,yaw(msg.info.origin.orientation)]
            transform=[0.,0.,0.]
            if msg.header.frame_id!=frame('map'):
                t=self.tf.lookup_transform(frame('map'),msg.header.frame_id,Time());v=t.transform
                transform=[v.translation.x,v.translation.y,yaw(v.rotation)]
            combined=compose(transform,origin);res=msg.info.resolution
            for index,value in enumerate(msg.data):
                if value<100:continue
                p=compose(combined,[(index%msg.info.width+.5)*res,(index//msg.info.width+.5)*res,0.])
                if math.dist(p[:2],self.local_pose[:2])<4:self.obstacles.append(p[:2])
            if len(self.obstacles)>10000:self.obstacles=[];self.obstacle_stamp=0.;return
            self.obstacle_complete=True
            self.obstacle_stamp=time.monotonic()
        except Exception:self.obstacles=[];self.obstacle_stamp=0.
    def on_plan(self,msg):
        stride=max(1,len(msg.poses)//100)
        points=[[p.pose.position.x,p.pose.position.y] for p in msg.poses[::stride]][:101]
        try:
            if msg.header.frame_id!=frame('map'):
                t=self.tf.lookup_transform(frame('map'),msg.header.frame_id,Time());v=t.transform
                points=[compose([v.translation.x,v.translation.y,yaw(v.rotation)],[*p,0.])[:2] for p in points]
            self.planned_path=points
        except Exception:self.planned_path=[]

    def on_manual(self,msg):
        if not self.control_gate_ready:return
        self.manual=[msg.linear.x,msg.angular.z];self.manual_time=time.monotonic()
        if self.mode!='MANUAL':
            self.manual_blocked_goals=set().union(*self.active_goal_ids.values()) if self.active_goal_ids else set()
            self.mode='MANUAL';self.fleet_hold=True;self.cancel_all()

    def on_nav_status(self,action,msg):
        active={bytes(s.goal_info.goal_id.uuid).hex() for s in msg.status_list if s.status in (1,2,3)}
        self.active_goal_ids[action]=active;self.nav_status[action]=bool(active)
        if self.mode=='MANUAL' and time.monotonic()-self.manual_time>.35 and self.cancel_pending==0:
            if active-self.manual_blocked_goals and time.monotonic()-self.velocity_time<.5 and abs(self.velocity[0])<.02 and abs(self.velocity[1])<.05:
                self.mode='LOCAL';self.nav_time=0.;self.startup_done=True

    def on_estop(self,msg):
        self.estop_input=bool(msg.data)
        if self.estop_input:self.safety_latched=True;save(self.state_path,{'safety_latched':True})
        self.estop=self.estop_input or self.safety_latched
        if self.estop:
            self.manual_blocked_goals=set().union(*self.active_goal_ids.values()) if self.active_goal_ids else set()
            self.mode='MANUAL';self.manual=[0.,0.];self.manual_time=time.monotonic()
            self.fleet_hold=True;self.cancel_all()

    def cancel_all(self):
        self.fleet_hold=True;self.pending_goal=False;self.goal=None;self.wp_stop.publish(Empty())
        for c in self.cancel_clients:
            if c.service_is_ready():
                self.cancel_pending+=1;f=c.call_async(CancelGoal.Request())
                def done(f):
                    self.cancel_pending=max(0,self.cancel_pending-1)
                    try:
                        if f.result().return_code not in (0,3):self.goal_error='navigation cancellation rejected'
                    except Exception:self.goal_error='navigation cancellation failed'
                f.add_done_callback(done)
        if self.goal_handle:self.cancel_handle(self.goal_handle);self.goal_handle=None

    def cancel_handle(self,handle):
        self.cancel_pending+=1
        future=handle.cancel_goal_async()
        def done(f):
            self.cancel_pending=max(0,self.cancel_pending-1)
            try:
                if f.result().return_code not in (0,3):self.goal_error='goal cancellation rejected'
            except Exception:self.goal_error='goal cancellation response unavailable'
        future.add_done_callback(done)

    def discovery(self):
        now=time.monotonic()
        for calls,pending,values in ((self.pending_state_calls,self.pending_states,self.states),
                                      (self.pending_parameter_calls,self.parameter_pending,self.runtime_parameters)):
            for name,(client,future,sent) in list(calls.items()):
                if now-sent>2.:
                    client.remove_pending_request(future);calls.pop(name,None);pending.discard(name);values.pop(name,None)
        legacy=self.get_publishers_info_by_topic(self.resolve_topic_name('/cmd_vel'))
        consumers=self.get_subscriptions_info_by_topic(self.resolve_topic_name('/msc/cmd_vel_safe'))
        legacy_consumers=self.get_subscriptions_info_by_topic(self.resolve_topic_name('/cmd_vel'))
        self.control_gate_ready=(self.config.get('enable_control',False) and
            any(i.node_name=='omnifleet_t2_driver' for i in consumers) and
            not any(i.node_name=='omnifleet_t2_driver' for i in legacy_consumers) and
            not any(i.node_name in ('controller_server','behavior_server') for i in legacy) and
            self.count_publishers(self.resolve_topic_name('/msc/cmd_vel_safe'))==1)
        for name,c in self.state_clients.items():
            if name not in self.pending_states and c.service_is_ready():
                self.pending_states.add(name);f=c.call_async(GetState.Request())
                self.pending_state_calls[name]=(c,f,time.monotonic())
                def done(f,name=name):
                    self.pending_states.discard(name);self.pending_state_calls.pop(name,None)
                    try:self.states[name]=(f.result().current_state.id,time.monotonic())
                    except Exception:self.states.pop(name,None)
                f.add_done_callback(done)
        ids=[]
        for p in Path('/proc').glob('[0-9]*'):
            try:
                argv=(p/'cmdline').read_bytes().split(b'\0')
                if argv and Path(os.fsdecode(argv[0])).name=='mola-cli':ids.append(p.name+':'+(p/'stat').read_text().split()[21])
            except (OSError,IndexError):pass
        epoch=self.machine_boot+'|'+'|'.join(sorted(ids)) if ids else (
            self.machine_boot+'|odom' if self.config.get('localization_source')=='odom' else '')
        if self.process_epoch and epoch!=self.process_epoch and self.mode=='FLEET':
            self.fleet_hold=True;self.goal_error='localization restarted';self.cancel_all()
        if epoch!=self.process_epoch:self.map_generation=0
        self.process_epoch=epoch;self.epoch=epoch+':'+str(self.map_generation) if epoch else ''

    def goal_safe(self,target):
        m=self.grid
        if m is None or time.monotonic()-self.grid_time>3:return False
        try:
            if m.header.frame_id!=frame('map'):
                t=self.tf.lookup_transform(m.header.frame_id,frame('map'),Time());v=t.transform
                target=compose([v.translation.x,v.translation.y,yaw(v.rotation)],target)
        except Exception:return False
        origin=[m.info.origin.position.x,m.info.origin.position.y,yaw(m.info.origin.orientation)]
        x,y,theta=compose(inverse(origin),target);res=m.info.resolution
        hl=self.config['length']/2+self.config['padding']+res*.71
        hw=self.config['width']/2+self.config['padding']+res*.71
        radius=math.hypot(hl,hw)
        x0,x1=math.floor((x-radius)/res),math.ceil((x+radius)/res)
        y0,y1=math.floor((y-radius)/res),math.ceil((y+radius)/res)
        if x0<0 or y0<0 or x1>=m.info.width or y1>=m.info.height:return False
        c,s=math.cos(theta),math.sin(theta)
        for iy in range(y0,y1+1):
            for ix in range(x0,x1+1):
                dx,dy=(ix+.5)*res-x,(iy+.5)*res-y
                if abs(c*dx+s*dy)<=hl and abs(-s*dx+c*dy)<=hw:
                    value=m.data[iy*m.info.width+ix]
                    if value<0 or value>=100:return False
        return True

    def apply(self,command):
        seq=int(command.get('seq',0));kind=command['kind'];epoch=command.get('epoch','')
        if seq<=self.applied_seq and epoch==self.command_epoch:return
        self.applied_seq=seq;self.command_epoch=epoch
        if kind=='observe':
            if self.mode=='FLEET':self.fleet_hold=True;self.cancel_all()
            return
        if not self.control_gate_ready:
            self.goal_error='exclusive control gate not ready';return
        if kind=='safety_stop':
            self.safety_latched=True;save(self.state_path,{'safety_latched':True});self.estop=True;self.fleet_hold=True;self.cancel_all();return
        if kind=='release_stop':
            now=time.monotonic()
            if not self.estop_input and not self.contact and self.zero_since is not None and now-self.zero_since>=.5 and now-self.velocity_time<.5 and not any(self.nav_status.values()) and not self.cancel_pending and not self.pending_goal_requests and now-self.manual_time>.35:
                self.safety_latched=False;save(self.state_path,{'safety_latched':False});self.estop=self.estop_input;self.nav_time=0.
                if not self.estop:self.mode='LOCAL';self.fleet_hold=True
            return
        if kind=='release':
            if self.mode=='FLEET':self.fleet_hold=True;self.cancel_all();self.mode='LOCAL';self.goal_error=''
            return
        if kind=='release_manual':
            if not any(self.nav_status.values()) and self.cancel_pending==0 and time.monotonic()-self.velocity_time<.5 and max(abs(v) for v in self.velocity)<.02:
                self.mode='LOCAL';self.nav_time=0.;self.manual_blocked_goals.clear();self.startup_done=True
            return
        if kind=='hold':
            if self.mode=='FLEET':self.fleet_hold=True;self.cancel_all()
            return
        if kind=='claim':
            if self.mode=='MANUAL' or self.estop:self.goal_error='manual/estop active';return
            self.mode='FLEET';self.fleet_hold=True;self.goal_error='';self.cancel_all();return
        if kind!='navigate':self.goal_error='unsupported agent command';return
        if not self.config.get('allow_fleet_motion',False):self.goal_error='fleet motion locked for static acceptance';self.fleet_hold=True;return
        if self.estop or self.mode=='MANUAL' or self.cancel_pending or self.pending_goal_requests:
            self.goal_error='manual/estop/cancellation active';return
        target=pose(command['target'])
        if not self.goal_safe(target):
            self.goal_error='unsafe target footprint or stale map';self.fleet_hold=True;self.cancel_all();return
        if not self.client.server_is_ready():self.goal_error='Nav2 unavailable';self.fleet_hold=True;return
        self.mode='FLEET';self.fleet_hold=False;self.goal_error='';self.goal=target;self.pending_goal=True
        goal=NavigateToPose.Goal();goal.pose.header.frame_id=frame('map');goal.pose.header.stamp=self.get_clock().now().to_msg()
        goal.pose.pose.position.x=target[0];goal.pose.pose.position.y=target[1]
        goal.pose.pose.orientation.z=math.sin(target[2]/2);goal.pose.pose.orientation.w=math.cos(target[2]/2)
        future=self.client.send_goal_async(goal)
        request_token=(epoch,seq);self.pending_goal_requests[request_token]=time.monotonic()
        def accepted(f):
            self.pending_goal_requests.pop(request_token,None)
            try:
                handle=f.result()
                if seq!=self.applied_seq or epoch!=self.command_epoch or self.fleet_hold:
                    if handle.accepted:self.cancel_handle(handle)
                    return
                self.pending_goal=False
                if not handle.accepted:self.goal_error='Nav2 rejected goal';self.fleet_hold=True;return
                self.goal_handle=handle;result=handle.get_result_async()
                def finished(f):
                    if seq!=self.applied_seq or epoch!=self.command_epoch:return
                    self.goal_handle=None;self.goal=None
                    try:
                        if f.result().status==4:self.completed_seq=seq
                        else:self.goal_error='Nav2 action ended with status '+str(f.result().status);self.fleet_hold=True
                    except Exception:self.goal_error='Nav2 result unavailable';self.fleet_hold=True
                result.add_done_callback(finished)
            except Exception:self.pending_goal=False;self.goal_error='Nav2 goal request failed';self.fleet_hold=True
        future.add_done_callback(accepted)

    def control_tick(self):
        now=time.monotonic()
        with self.lock:
            commands=self.incoming;self.incoming=[];reply=copy.deepcopy(self.response);received=self.last_response
        for command in commands:self.apply(command)
        command=[0.,0.];self.safety_reason=''
        if self.control_gate_ready:
            if self.mode=='STARTUP':
                if not self.startup_canceled:self.cancel_all();self.startup_canceled=True
                stopped=not any(self.nav_status.values()) and not self.cancel_pending and not self.pending_goal_requests and now-self.velocity_time<.5 and max(abs(v) for v in self.velocity)<.02
                if stopped:
                    if self.startup_stopped is None:self.startup_stopped=now
                    if now-self.startup_stopped>=.5:self.mode='LOCAL';self.nav_time=0.;self.startup_done=True
                else:self.startup_stopped=None
            elif self.mode=='MANUAL':
                if now-self.manual_time<.35:command=self.manual
            elif self.mode=='LOCAL':
                if self.config.get('require_local_shared_map',False) and (now-self.shared_map_time>3 or not self.shared_map_status.get('alignment_valid')):
                    self.safety_reason='shared map unavailable for local navigation'
                elif now-self.nav_time<.35:command=self.nav
            elif self.mode=='FLEET':
                if now-received>.6:
                    if not self.fleet_hold:
                        self.fleet_hold=True;self.goal_error='coordinator lease expired; resume required';self.cancel_all()
                    self.safety_reason='coordinator lease expired'
                elif self.config.get('require_shared_map') and (now-self.shared_map_time>3 or not self.shared_map_status.get('alignment_valid')):
                    self.safety_reason='shared map unavailable'
                elif self.config.get('require_obstacles') and (not self.obstacle_complete or now-self.obstacle_stamp>2.5):
                    self.safety_reason='obstacle observations unavailable'
                elif not self.fleet_hold and now-self.nav_time<.35:command=self.nav
        if not all(math.isfinite(v) for v in command):command=[0.,0.];self.safety_reason='nonfinite command'
        # Local single-point and waypoint navigation is always an onboard path.
        # Coordinator state becomes a safety overlay only after an explicit fleet claim.
        fleet_safety=self.mode=='FLEET'
        if fleet_safety:command=[max(-.4,min(.4,command[0])),max(-1.2,min(1.2,command[1]))]
        if fleet_safety and any(command):
            alignment=reply.get('alignment');fleet_pose=compose(alignment['transform'],self.local_pose) if alignment and alignment.get('epoch')==self.epoch and self.local_pose and self.pose_age<.8 else None
            peers=reply.get('peers',[])
            for peer in peers:peer['age']+=max(0.,now-received)
            command,self.safety_reason=safe_velocity({'pose':fleet_pose,'radius':self.config['radius'],'age':self.pose_age},command,peers,
                clearance=max(self.config.get('safety_clearance',.2),reply.get('safety_clearance',.2)))
        if self.estop:command=[0.,0.];self.safety_reason='estop'
        m=Twist();m.linear.x=command[0];m.angular.z=command[1];self.output.publish(m)

    def snapshot(self):
        now=time.monotonic()
        try:
            t=self.tf.lookup_transform(frame('map'),frame('base_link'),Time())
            age=(self.get_clock().now().nanoseconds-(t.header.stamp.sec*10**9+t.header.stamp.nanosec))/1e9
            self.pose_stamp_ns=t.header.stamp.sec*10**9+t.header.stamp.nanosec
            if age<-.5:raise ValueError('localization timestamp is in the future')
            current=pose([t.transform.translation.x,t.transform.translation.y,yaw(t.transform.rotation)])
        except Exception:
            if self.config.get('localization_source')!='odom' or self.odom_pose is None or not self.odom_time:
                self.pose_age=99.;self.local_pose=None;return
            age=now-self.odom_time
            self.pose_stamp_ns=self.odom_stamp_ns
            current=list(self.odom_pose)
            if age<-.5:raise ValueError('odometry timestamp is in the future')
        try:
            if self.local_pose is not None:
                dt=max(.01,now-self.pose_time)
                jump=math.dist(current[:2],self.local_pose[:2])>.4*dt+.2 or abs(math.atan2(math.sin(current[2]-self.local_pose[2]),math.cos(current[2]-self.local_pose[2])))>1.2*dt+.3
                if jump:
                    self.map_generation+=1;self.epoch=self.process_epoch+':'+str(self.map_generation)
                    if self.mode=='FLEET':self.fleet_hold=True;self.goal_error='localization jump; alignment must be checked';self.cancel_all()
            self.pose_age=max(0.,age);self.local_pose=current;self.pose_time=now
            self.pose_window.append((now,current))
        except Exception:self.pose_age=99.;self.local_pose=None
        while self.pose_window and now-self.pose_window[0][0]>1.5:self.pose_window.popleft()
        pose_stationary=bool(self.local_pose is not None and self.pose_age<.8 and self.pose_window and now-self.pose_window[0][0]>=1. and
            all(math.dist(p[:2],self.local_pose[:2])<.015 and abs(math.atan2(math.sin(p[2]-self.local_pose[2]),math.cos(p[2]-self.local_pose[2])))<.05 for _,p in self.pose_window))
        ready=all(self.states.get(name,(0,0))[0]==3 and now-self.states[name][1]<3 for name in self.state_clients) and self.client.server_is_ready()
        state={'local_pose':self.local_pose,'pose_age':self.pose_age,'velocity':self.velocity,'velocity_age':now-self.velocity_time,
            'pose_stamp_ns':self.pose_stamp_ns,
            'pose_stationary':pose_stationary,
            'contact':self.contact if now-self.contact_time<1. else None,
            'shared_map':copy.deepcopy(self.shared_map_status),
            'shared_map_ready':bool(now-self.shared_map_time<3 and self.shared_map_status.get('alignment_valid') and self.shared_map_status.get('sha256')),
            'selected_algorithms':copy.deepcopy(self.selection),
            'obstacles':self.obstacles,'obstacle_age':now-self.obstacle_stamp if self.obstacle_stamp else 99.,
            'obstacle_observations_complete':self.obstacle_complete,
            'runtime_parameters':{k:{'values':v['values'],'age':now-v['received_at']} for k,v in self.runtime_parameters.items()},
            'stopped_seconds':now-self.zero_since if self.zero_since is not None and now-self.velocity_time<.5 else 0.,
            'localization_epoch':self.epoch,'nav_ready':ready,'control_gate_ready':self.control_gate_ready and self.startup_done,
            'control_mode':self.mode,'local_navigation_independent':True,
            'fleet_overlay_active':self.mode=='FLEET','manual_active':self.mode=='MANUAL','estop':self.estop,
            'control_epoch':self.command_epoch,'applied_command_seq':self.applied_seq,
            'coordinator_age':now-self.last_response if self.last_response else 99.,'network_error':self.network_error,
            'nav_active':any(self.nav_status.values()),'pending_goal':self.pending_goal or bool(self.pending_goal_requests) or self.cancel_pending>0,
            'goal':self.goal,'goal_error':self.goal_error,'completed_seq':self.completed_seq,
            'planned_path':self.planned_path,'safety_stop_reason':self.safety_reason,
            'health':'READY' if ready and self.pose_age<.8 else 'WAITING_LOCALIZATION_OR_NAV2'}
        with self.lock:state['leader_id']=self.response.get('leader_id');state['mission_state']=self.response.get('mission_state','IDLE')
        with self.lock:self.own_snapshot=state
        msg=String();msg.data=json.dumps({'robot_id':self.config['robot_id'],**state});self.heartbeat_pub.publish(msg)

    def network_loop(self):
        while not self.closed:
            try:
                with self.lock:state=copy.deepcopy(self.own_snapshot)
                if not state:time.sleep(.1);continue
                self.sequence+=1
                response=request(self.config['coordinator'],self.config['robot_id'],self.config['key'],'/v1/heartbeat',
                    {'boot':self.boot,'seq':self.sequence,'state':state})
                with self.lock:
                    self.last_response=time.monotonic();self.response=response;self.network_error=''
                    self.incoming=[response['command']]
            except Exception as error:self.network_error=str(error)
            time.sleep(.2)

def main():
    p=argparse.ArgumentParser();p.add_argument('--config',required=True);args,ros=p.parse_known_args()
    rclpy.init(args=ros);node=Agent(json.load(open(args.config)))
    try:rclpy.spin(node)
    except (KeyboardInterrupt,rclpy.executors.ExternalShutdownException):pass
    finally:
        node.closed=True
        if rclpy.ok():
            if node.mode=='FLEET':node.cancel_all()
            for _ in range(3):node.output.publish(Twist())
        node.network.join(timeout=2);node.destroy_node()
        rclpy.try_shutdown()
