#!/usr/bin/env python3
"""Manual Nav2 demos; no service installation and no direct chassis commands."""
import argparse,json,math,os,signal,subprocess,time
from pathlib import Path
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile,DurabilityPolicy,ReliabilityPolicy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid
from nav2_msgs.action import NavigateToPose,FollowWaypoints,ComputePathToPose
from nav2_msgs.msg import SpeedLimit
from lifecycle_msgs.srv import ChangeState
from std_srvs.srv import Trigger
from rcl_interfaces.srv import GetParameters,SetParametersAtomically
from rclpy.parameter import Parameter
from omnifleet_msc.protocol import request
from omnifleet_msc.geometry import compose,inverse,angle

ROBOTS=('robot_113','robot_104')
def yaw(q):return math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
def body_safe(grid,p):
    o=grid.info.origin;x,y,t=compose(inverse([o.position.x,o.position.y,yaw(o.orientation)]),p)
    res=grid.info.resolution;hl=.3+res*.71;hw=.235+res*.71;rad=math.hypot(hl,hw);c,s=math.cos(t),math.sin(t)
    for iy in range(math.floor((y-rad)/res),math.ceil((y+rad)/res)+1):
        for ix in range(math.floor((x-rad)/res),math.ceil((x+rad)/res)+1):
            dx,dy=(ix+.5)*res-x,(iy+.5)*res-y
            if abs(c*dx+s*dy)<=hl and abs(-s*dx+c*dy)<=hw:
                if not(0<=ix<grid.info.width and 0<=iy<grid.info.height):return False
                value=grid.data[iy*grid.info.width+ix]
                if value<0 or value>=100:return False
    return True

class Demo(Node):
    def __init__(self,args):
        super().__init__('manual_nav2_modes_demo');self.args=args;self.config=json.load(open('/etc/omnifleet_msc/operator.json'))
        self.routes=json.load(open(args.routes));self.maps={};self.map_times={};self.children=[];self.goals=[];self.record=None;self.guard_enabled=False;self.last_poll=0.;self.last_print=0.;self.following=False;self.saved_parameters={}
        self.output=Path(args.output);self.output.mkdir(parents=True,exist_ok=False);self.events=open(self.output/'events.jsonl','w')
        q=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL,reliability=ReliabilityPolicy.RELIABLE)
        self.subs=[self.create_subscription(OccupancyGrid,'/'+r+'/global_costmap/costmap',lambda m,r=r:self.map_message(r,m),q) for r in ROBOTS]
        self.nav={r:ActionClient(self,NavigateToPose,'/'+r+'/navigate_to_pose') for r in ROBOTS}
        self.planner={r:ActionClient(self,ComputePathToPose,'/'+r+'/compute_path_to_pose') for r in ROBOTS}
        self.waypoints={r:ActionClient(self,FollowWaypoints,'/'+r+'/follow_waypoints') for r in ROBOTS}
        self.stop={r:self.create_client(Trigger,'/'+r+'/navigation/stop') for r in ROBOTS}
        self.limits={r:self.create_publisher(SpeedLimit,'/'+r+'/speed_limit',10) for r in ROBOTS}
        self.follower=self.routes.get('follower','robot_104');self.leader=self.routes.get('leader','robot_113');self.spacing=float(self.routes.get('spacing',1.5))
        if {self.leader,self.follower}!=set(ROBOTS) or not 1.4<=self.spacing<=5:raise ValueError('invalid leader/follower or spacing')
        self.updates=self.create_publisher(PoseStamped,'/'+self.follower+'/goal_update',10)
    def map_message(self,r,m):self.maps[r]=m;self.map_times[r]=time.monotonic()
    def status(self):return request(self.config['coordinator'],'operator',self.config['key'],'/v1/status',{},timeout=.8)
    def event(self,kind,**data):self.events.write(json.dumps({'time':time.time(),'event':kind,**data})+'\n');self.events.flush()
    def limit(self,value):
        for pub in self.limits.values():
            m=SpeedLimit();m.header.stamp=self.get_clock().now().to_msg();m.percentage=False;m.speed_limit=value;pub.publish(m)
    def pose(self,r,p):
        m=PoseStamped();m.header.frame_id=r+'/map';m.header.stamp=self.get_clock().now().to_msg();m.pose.position.x=p[0];m.pose.position.y=p[1];m.pose.orientation.z=math.sin(p[2]/2);m.pose.orientation.w=math.cos(p[2]/2);return m
    def local(self,r,p):return compose(inverse(self.state['robots'][r]['alignment']['transform']),p)
    def target(self):return self.local(self.follower,compose(self.state['robots'][self.leader]['fleet_pose'],[-self.spacing,0.,0.]))
    def poll(self):
        self.state=self.status();s=self.state
        self.event('guard_sample',robots={r:{k:v.get(k) for k in ['pose_age','online','estop','fleet_pose','velocity']} for r,v in s['robots'].items()})
        assert s['mission_state']=='IDLE' and not s['motion_enabled'],'MSC mission changed during manual demo'
        for r in ROBOTS:
            v=s['robots'][r]
            assert v['nav_ready'] and v['local_ready'],'Nav2 not ready: '+r
            assert v['online'] and v['fleet_pose'] is not None and v['pose_age']<.8 and not v['estop'],'stale localization, offline robot or estop'
            assert v['localization_epoch']==self.epochs[r],'localization epoch changed'
            assert not v.get('manual_active'),'manual takeover'
            assert abs(v['velocity'][0])<=.09 and abs(v['velocity'][1])<=.5,'observed speed envelope exceeded'
            assert math.dist(v['fleet_pose'][:2],self.initial[r]['fleet_pose'][:2])<=1.8,'travel envelope exceeded'
            assert time.monotonic()-self.map_times[r]<3.,'costmap stale'
            assert body_safe(self.maps[r],v['local_pose']),'current full footprint not clear'
        assert s['current_robot_distance']>=1.1,'separation below test stop margin'
        self.event('status',distance=s['current_robot_distance'],robots={r:{k:s['robots'][r].get(k) for k in ['fleet_pose','local_pose','velocity','pose_age','nav_active','goal_error']} for r in ROBOTS})
        if self.following:
            p=self.target();assert body_safe(self.maps[self.follower],p),'dynamic following target blocked'
            self.updates.publish(self.pose(self.follower,p));self.event('goal_update',robot=self.follower,local_target=p)
        if time.monotonic()-self.last_print>4:
            print(json.dumps({'distance':s['current_robot_distance'],'poses':{r:s['robots'][r]['fleet_pose'] for r in ROBOTS}}),flush=True);self.last_print=time.monotonic()
    def pump(self):
        rclpy.spin_once(self,timeout_sec=.03)
        if self.guard_enabled:self.limit(self.args.speed)
        if self.guard_enabled and time.monotonic()-self.last_poll>.2:
            self.limit(self.args.speed);self.poll();self.last_poll=time.monotonic()
    def wait(self,f,timeout):
        end=time.monotonic()+timeout
        while not f.done() and time.monotonic()<end:self.pump()
        if not f.done():raise RuntimeError('ROS request timeout')
        return f.result()
    def goal(self,client,message):
        assert client.wait_for_server(timeout_sec=3),'action server unavailable'
        h=self.wait(client.send_goal_async(message),4)
        assert h.accepted,'action rejected';result=h.get_result_async();self.goals.append((h,result));return result
    def preflight(self,r,points):
        start=self.state['robots'][r]['local_pose'];all_points=[]
        for fleet_target in points:
            target=self.local(r,fleet_target);assert body_safe(self.maps[r],target),'goal footprint blocked: '+r
            g=ComputePathToPose.Goal();g.goal=self.pose(r,target);g.start=self.pose(r,start);g.use_start=True;g.planner_id='GridBased'
            f=self.goal(self.planner[r],g);result=self.wait(f,8);assert result.status==4 and result.result.path.poses,'planner failed: '+r
            path=[[p.pose.position.x,p.pose.position.y,yaw(p.pose.orientation)] for p in result.result.path.poses]
            # Smac2D uses zero quaternions for intermediate grid points. Check
            # the footprint along the tangent, choosing forward/reverse nearest
            # the current heading; retain the requested final orientation.
            reference=start[2]
            for i in range(len(path)-1):
                direction=math.atan2(path[i+1][1]-path[i][1],path[i+1][0]-path[i][0])
                path[i][2]=min((direction,direction+math.pi),key=lambda v:abs(angle(v-reference)));reference=path[i][2]
            path[-1][2]=target[2]
            for a,b in zip(path,path[1:]):
                count=max(1,math.ceil(math.dist(a[:2],b[:2])/.025),math.ceil(abs(angle(b[2]-a[2]))/.08))
                for i in range(count+1):
                    p=[a[0]+(b[0]-a[0])*i/count,a[1]+(b[1]-a[1])*i/count,a[2]+angle(b[2]-a[2])*i/count]
                    if not body_safe(self.maps[r],p):
                        self.event('preflight_unsafe',robot=r,pose=p,goal=target);raise RuntimeError('planned full footprint blocked: '+r)
            all_points+=path;start=target
        self.event('preflight',robot=r,path=all_points);return all_points
    def prepare(self):
        end=time.monotonic()+8
        while (len(self.maps)<2 or any(pub.get_subscription_count()!=1 for pub in self.limits.values())) and time.monotonic()<end:rclpy.spin_once(self,timeout_sec=.1)
        assert len(self.maps)==2 and all(pub.get_subscription_count()==1 for pub in self.limits.values()),'map or speed limiter unavailable'
        self.state=self.status();end=time.monotonic()+8
        while not all(v.get('nav_ready') and v.get('local_ready') for v in self.state['robots'].values()) and time.monotonic()<end:
            rclpy.spin_once(self,timeout_sec=.2);self.state=self.status()
        self.initial=self.state['robots'];self.epochs={r:self.initial[r]['localization_epoch'] for r in ROBOTS}
        assert all(not v['nav_active'] and not v['pending_goal'] and max(abs(x) for x in v['velocity'])<.01 for v in self.initial.values()),'robots must be idle and stopped'
        self.poll();paths={r:self.preflight(r,self.routes['routes'].get(r,[])) for r in ROBOTS}
        if self.args.mode=='leader':self.preflight(self.follower,[compose(self.initial[self.leader]['fleet_pose'],[-self.spacing,0.,0.])])
        else:
            transforms={r:self.initial[r]['alignment']['transform'] for r in ROBOTS};fleet={r:[compose(transforms[r],p) for p in paths[r]] for r in ROBOTS}
            assert all(math.dist(a[:2],b[:2])>=1.1 for a in fleet[ROBOTS[0]] for b in fleet[ROBOTS[1]]),'fixed routes intersect safety margin'
        for r,m in self.maps.items():
            o=m.info.origin;(self.output/(r+'-costmap-before.json')).write_text(json.dumps({'frame':m.header.frame_id,'width':m.info.width,'height':m.info.height,'resolution':m.info.resolution,'origin':[o.position.x,o.position.y,yaw(o.orientation)],'data':list(m.data)}))
        self.event('preflight_pass',mode=self.args.mode);print('PREFLIGHT PASS',flush=True)
    def start_waypoints(self,r):
        if self.waypoints[r].server_is_ready():raise RuntimeError('existing FollowWaypoints server: do not take over its configuration')
        log=open(self.output/(r+'-waypoint-server.log'),'w')
        p=subprocess.Popen(['ros2','run','nav2_waypoint_follower','waypoint_follower','--ros-args','-r','__ns:=/'+r,'-r','__node:=manual_waypoint_follower','-p','stop_on_failure:=true','-p','wait_at_waypoint.enabled:=false','-r','bond:=/manual_nav2_modes/'+r+'/bond'],stdout=log,stderr=subprocess.STDOUT,start_new_session=True);self.children.append((r,p))
        c=self.create_client(ChangeState,'/'+r+'/manual_waypoint_follower/change_state');assert c.wait_for_service(timeout_sec=8),'waypoint lifecycle service unavailable'
        for transition in [1,3]:
            q=ChangeState.Request();q.transition.id=transition;assert self.wait(c.call_async(q),4).success,'waypoint lifecycle failed'
        assert self.waypoints[r].wait_for_server(timeout_sec=3),'FollowWaypoints not activated'
    def fixed_goal(self,r):
        g=FollowWaypoints.Goal();g.poses=[self.pose(r,self.local(r,p)) for p in self.routes['routes'][r]];return self.goal(self.waypoints[r],g)
    def set_parameters(self,r,values):
        c=self.create_client(SetParametersAtomically,'/'+r+'/controller_server/set_parameters_atomically');assert c.wait_for_service(timeout_sec=3),'controller parameter service unavailable'
        q=SetParametersAtomically.Request();q.parameters=[Parameter(k,value=v).to_parameter_msg() for k,v in values.items()]
        result=self.wait(c.call_async(q),4);assert result.result.successful,result.result.reason
    def cap_parameters(self):
        names=['FollowPathMPPI.vx_max','FollowPathMPPI.vx_min','FollowPathMPPI.wz_max','general_goal_checker.xy_goal_tolerance']
        for r in ROBOTS:
            assert self.state['robots'][r]['selected_algorithms']['controller']=='FollowPathMPPI','manual demo requires selected MPPI'
            c=self.create_client(GetParameters,'/'+r+'/controller_server/get_parameters');assert c.wait_for_service(timeout_sec=3)
            q=GetParameters.Request();q.names=names;result=self.wait(c.call_async(q),4)
            assert all(v.type==3 for v in result.values),'unexpected controller parameter types'
            self.saved_parameters[r]={k:v.double_value for k,v in zip(names,result.values)}
            (self.output/'controller-parameters.before.json').write_text(json.dumps(self.saved_parameters,indent=2))
            values=dict(zip(names,[self.args.speed,-self.args.speed,.08,.08]));self.set_parameters(r,values);self.event('temporary_controller_parameters',robot=r,values=values)
    def execute(self):
        self.cap_parameters();self.guard_enabled=False
        topics=['/fleet/status','/fleet/mission_state']
        for r in ROBOTS:topics+=['/'+r+x for x in ['/tf','/tf_static','/odom','/msc/nav_cmd_vel','/msc/cmd_vel_safe','/navigation/local_status','/msc/local_status','/plan','/goal_update','/speed_limit']]
        log=open(self.output/'record.log','w');self.record=subprocess.Popen(['nice','-n','10','ros2','bag','record','-s','mcap','-o',str(self.output/'bag')]+topics,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
        end=time.monotonic()+2
        while time.monotonic()<end:self.pump()
        assert self.record.poll() is None,'bag recorder failed'
        for r in (ROBOTS if self.args.mode=='fixed' else (self.leader,)):self.start_waypoints(r)
        end=time.monotonic()+15.;stable=None
        while time.monotonic()<end:
            rclpy.spin_once(self,timeout_sec=.1);self.limit(self.args.speed);self.state=self.status()
            assert all(not v.get('manual_active') and not v['estop'] and max(abs(x) for x in v['velocity'])<.01 for v in self.state['robots'].values()),'robots moved during startup'
            ready=all(v.get('nav_ready') and v.get('local_ready') and v.get('fleet_pose') and v['pose_age']<.6 for v in self.state['robots'].values())
            stable=(stable or time.monotonic()) if ready else None
            if stable and time.monotonic()-stable>=2.:break
        assert stable and time.monotonic()-stable>=2.,'localization did not settle after startup'
        self.guard_enabled=True;self.poll()
        if self.args.mode=='fixed':
            results={};
            for r in ROBOTS:
                results[r]=self.fixed_goal(r);warm=time.monotonic()+2.
                while time.monotonic()<warm:self.pump()
            end=time.monotonic()+self.args.duration
            while not all(f.done() for f in results.values()) and time.monotonic()<end:self.pump()
            for r,f in results.items():
                assert f.done(),'fixed waypoint timeout';v=f.result();self.event('result',robot=r,status=v.status,missed=list(v.result.missed_waypoints));assert v.status==4 and not v.result.missed_waypoints,'fixed waypoint failed: '+r
        else:
            heading_end=time.monotonic()+40.
            while abs(angle(self.state['robots'][self.follower]['local_pose'][2]-self.target()[2]))>.25:
                assert time.monotonic()<heading_end,'initial heading alignment timed out'
                p=list(self.state['robots'][self.follower]['local_pose']);delta=angle(self.target()[2]-p[2]);p[2]+=max(-.4,min(.4,delta))
                assert body_safe(self.maps[self.follower],p),'initial heading footprint blocked'
                step=NavigateToPose.Goal();step.pose=self.pose(self.follower,p)
                result=self.wait(self.goal(self.nav[self.follower],step),8);assert result.status==4,'heading step failed'
                end=time.monotonic()+.6
                while time.monotonic()<end:self.pump()
            g=NavigateToPose.Goal();g.pose=self.pose(self.follower,self.target());g.behavior_tree=str(Path(__file__).with_name('nav2_dynamic_follow.xml'))
            follower_result=self.goal(self.nav[self.follower],g);self.following=True;end=time.monotonic()+8
            while self.updates.get_subscription_count()==0 and not follower_result.done() and time.monotonic()<end:self.pump()
            assert self.updates.get_subscription_count()>0 and not follower_result.done(),'GoalUpdater did not start'
            self.limit(self.args.speed)
            leader_result=self.fixed_goal(self.leader);end=time.monotonic()+self.args.duration;settled=None
            while time.monotonic()<end:
                self.pump();assert not follower_result.done(),'dynamic follower stopped unexpectedly'
                if not leader_result.done():continue
                v=leader_result.result();assert v.status==4 and not v.result.missed_waypoints,'leader waypoints failed'
                current=self.state['robots'][self.follower]['local_pose'];p=self.target();close=math.dist(current[:2],p[:2])<=.15 and abs(angle(current[2]-p[2]))<=.3
                if close:
                    settled=settled or time.monotonic()
                    if time.monotonic()-settled>=2.:break
                else:settled=None
            assert settled and time.monotonic()-settled>=2.,'follower did not settle at target'
            for r in ROBOTS:
                displacement=math.dist(self.state['robots'][r]['fleet_pose'][:2],self.initial[r]['fleet_pose'][:2]);assert displacement>=.12,'insufficient physical displacement: '+r
            self.event('following_complete',spacing=self.spacing,distance=self.state['current_robot_distance'])
        self.event('accepted',mode=self.args.mode);print('MODE PASS '+self.args.mode,flush=True)
    def close(self):
        self.guard_enabled=False;self.following=False
        emergency=[]
        for r in ROBOTS:
            if self.stop[r].service_is_ready():emergency.append(self.stop[r].call_async(Trigger.Request()))
        for f in emergency:
            try:self.wait(f,3)
            except Exception:pass
        for h,result in self.goals:
            if result.done():continue
            try:self.wait(h.cancel_goal_async(),2)
            except Exception:pass
        for _ in range(15):self.limit(0.);rclpy.spin_once(self,timeout_sec=.1)
        if self.record and self.record.poll() is None:
            os.killpg(self.record.pid,signal.SIGINT)
            try:self.record.wait(timeout=12)
            except subprocess.TimeoutExpired:os.killpg(self.record.pid,signal.SIGTERM);self.record.wait(timeout=5)
        for r,p in self.children:
            if p.poll() is None:
                c=self.create_client(ChangeState,'/'+r+'/manual_waypoint_follower/change_state')
                for transition in [4,2]:
                    try:
                        q=ChangeState.Request();q.transition.id=transition;self.wait(c.call_async(q),3)
                    except Exception:pass
                os.killpg(p.pid,signal.SIGINT)
                try:p.wait(timeout=5)
                except subprocess.TimeoutExpired:os.killpg(p.pid,signal.SIGTERM);p.wait(timeout=5)
        for r,values in self.saved_parameters.items():
            try:self.set_parameters(r,values);self.event('controller_parameters_restored',robot=r,values=values)
            except Exception as e:self.event('restore_error',robot=r,error=str(e));print('PARAMETER RESTORE ERROR '+r+' '+str(e),flush=True)
        s=self.status();self.event('final',motion_enabled=s['motion_enabled'],robots={r:{k:s['robots'][r].get(k) for k in ['velocity','nav_active','pending_goal','fleet_pose']} for r in ROBOTS});self.events.close()

def main():
    parser=argparse.ArgumentParser();parser.add_argument('--mode',choices=['leader','fixed'],required=True);parser.add_argument('--routes',required=True);parser.add_argument('--output',required=True);parser.add_argument('--execute',action='store_true');parser.add_argument('--duration',type=float,default=90.);parser.add_argument('--speed',type=float,default=.025);args=parser.parse_args()
    if not 0.<args.speed<=.05 or not 10<=args.duration<=90:parser.error('speed must be <=0.05 m/s; duration must be 10..90 s')
    def interrupted(signum,frame):raise RuntimeError('demo interrupted: '+str(signum))
    for s in [signal.SIGTERM,signal.SIGINT,signal.SIGHUP]:signal.signal(s,interrupted)
    rclpy.init();node=Demo(args)
    try:
        node.prepare()
        if args.execute:node.execute()
    except Exception as e:
        node.event('failed',error=str(e));print('MODE FAIL '+str(e),flush=True);raise
    finally:
        if args.execute:node.close()
        else:node.events.close()
        node.destroy_node();rclpy.shutdown()
if __name__=='__main__':main()
