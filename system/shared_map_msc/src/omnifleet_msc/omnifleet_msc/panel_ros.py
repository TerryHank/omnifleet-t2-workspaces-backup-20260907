import json,math,time,uuid,threading
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path
from std_msgs.msg import String
from geometry_msgs.msg import PointStamped,PoseStamped,Point
from visualization_msgs.msg import Marker,MarkerArray
from rclpy.qos import QoSProfile,DurabilityPolicy,ReliabilityPolicy
from .panel_model import FleetPanelModel
from .online_robots import DomainVehicleDiscovery
from .geometry import compose
from .urdf_visuals import load_visuals,combine,fleet_root
from ament_index_python.packages import get_package_share_directory
from std_srvs.srv import SetBool

class FleetPanelIO:
    def __init__(self,node):
        self.n=node;self.model=FleetPanelModel(node.core,Path(node.config['journal']).with_name('foxglove-fleet.json'))
        self.discovery=DomainVehicleDiscovery(node)
        self.visuals=load_visuals(Path(get_package_share_directory('omnifleet_description'))/'urdf/omnifleet_t2.urdf')
        self.operation_epoch=0;self.allow_clients={}
        self.ack={};self.seen={};self.pool=ThreadPoolExecutor(max_workers=1);self.future=None;self.future_id=None;self.route_counts={}
        q=QoSProfile(depth=1,reliability=ReliabilityPolicy.RELIABLE,durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.state_pub=node.create_publisher(String,'/fleet/ui/state',q)
        self.markers=node.create_publisher(MarkerArray,'/fleet/ui/markers',q)
        node.create_subscription(String,'/fleet/ui/command',self.command,10)
        node.create_subscription(PointStamped,'/fleet/ui/add_point',self.point,10)
        node.create_subscription(PoseStamped,'/fleet/ui/add_pose',self.point,10)
        node.create_timer(.5,self.tick)

    def command(self,message):
        id_=''
        try:
            if len(message.data)>32768:raise ValueError('请求过大')
            req=json.loads(message.data);id_=req['id']
            if not isinstance(id_,str) or not 1<=len(id_)<=80:raise ValueError('请求编号不合法')
            if id_ in self.seen:self.ack=self.seen[id_];self.tick();return
            if abs(time.time()-float(req['created_at']))>10:raise ValueError('请求已过期，请检查工作站与车载时钟')
            if req.get('op')=='stop':self.operation_epoch+=1
            if req.get('op') in ('preview','start'):
                if self.future is not None:raise ValueError('预检正在进行，请等待完成')
                self.future_id=id_;self.future=self.pool.submit(self.async_command,req,self.operation_epoch)
                self.ack={'id':id_,'pending':True,'message':'正在检查整条路线或申请本地模块执行'};self.seen[id_]=self.ack;self.tick();return
            with self.n.lock:result=self.model.command(req)
            self.ack={'id':id_,'ok':True,'message':result if isinstance(result,str) else '请求已受理，请观察任务状态'}
        except Exception as error:self.ack={'id':id_,'ok':False,'message':str(error)}
        self.seen[id_]=self.ack
        while len(self.seen)>128:self.seen.pop(next(iter(self.seen)))
        self.tick()

    def fleet_permission(self,robot,enabled):
        client=self.allow_clients.get(robot)
        if client is None:
            client=self.n.create_client(SetBool,'/'+robot+'/navigation/allow_fleet');self.allow_clients[robot]=client
        if not client.wait_for_service(timeout_sec=1):raise ValueError(robot+' 的本地导航模块不可用')
        event=threading.Event();future=client.call_async(SetBool.Request(data=enabled))
        def complete(f):
            # A timed-out grant must not silently arm a robot later.
            if enabled and expired[0]:client.call_async(SetBool.Request(data=False))
            event.set()
        expired=[False];future.add_done_callback(complete)
        if not event.wait(3):
            expired[0]=True
            if enabled:client.call_async(SetBool.Request(data=False))
            raise ValueError(robot+' 执行权响应超时，已请求撤销')
        result=future.result()
        if not result.success:raise ValueError(robot+'：'+result.message)

    def async_command(self,request,epoch):
        if request['op']=='preview':return self.model.preview(request.get('robot_id',self.model.config['selected']))
        granted=[];committed=False
        with self.n.lock:
            core=self.n.core
            if not core.config.get('allow_motion'):raise ValueError('协同运动尚未启用；本地导航可从本地入口调用')
            config=self.model.config
            robots=list(config['active']) if config['mode']=='leader' else [request.get('robot_id',config['selected'])]
            if any(r not in config['active'] for r in robots):raise ValueError('请先激活目标车辆')
            if self.model.busy():raise ValueError('已有协同任务，请先结束或取消')
            for robot in robots:
                state=core.robot(robot)
                if not state.get('online') or state.get('nav_active') or state.get('pending_goal'):
                    raise ValueError(robot+' 离线或正在导航，无法申请协同')
            core.ui_start_in_progress=True
        try:
            for robot in robots:
                if epoch!=self.operation_epoch:raise ValueError('开始请求已取消')
                # Include attempted grants so a lost response also gets revoked.
                granted.append(robot);self.fleet_permission(robot,True)
            deadline=time.monotonic()+3
            ready=False
            while time.monotonic()<deadline:
                if epoch!=self.operation_epoch:raise ValueError('开始请求已取消')
                with self.n.lock:
                    ready=all(core.robot(r).get('fleet_enabled') and not core.robot(r).get('local_override') for r in robots)
                if ready:break
                time.sleep(.05)
            if not ready:raise ValueError('协同执行权状态未确认')
            with self.n.lock:
                if epoch!=self.operation_epoch:raise ValueError('开始请求已取消')
                result=self.model.start(request);committed=True
                return result
        finally:
            failures=[]
            if not committed:
                for robot in granted:
                    try:self.fleet_permission(robot,False)
                    except Exception as error:failures.append(str(error))
            with self.n.lock:core.ui_start_in_progress=False
            if failures:raise ValueError('启动失败，执行权撤销未确认：'+'；'.join(failures))

    def point(self,message):
        try:
            if message.header.frame_id not in ('fleet_map',self.n.config['reference_robot']+'/map'):
                raise ValueError('请在多机协同3D场景的共享地图坐标系中选点')
            if hasattr(message,'point'):
                p=message.point;r=self.model.config['selected'];route=self.model.config['routes'].get(r,[])
                angle=math.atan2(p.y-route[-1][1],p.x-route[-1][0]) if route else 0.
            else:
                p=message.pose.position;q=message.pose.orientation
                angle=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
            with self.n.lock:result=self.model.add_point([p.x,p.y,angle])
            self.ack={'id':'map-point','ok':True,'message':result}
        except Exception as error:self.ack={'id':'map-point','ok':False,'message':str(error)}
        self.tick()

    def tick(self):
        if self.future is not None and self.future.done():
            try:self.ack={'id':self.future_id,'ok':True,'message':self.future.result()}
            except Exception as error:self.ack={'id':self.future_id,'ok':False,'message':str(error)}
            self.seen[self.future_id]=self.ack;self.future=None
        with self.n.lock:state=self.discovery.decorate(self.model.state())
        state['ack']=self.ack;self.state_pub.publish(String(data=json.dumps(state,ensure_ascii=False,allow_nan=False)))
        array=MarkerArray();stamp=self.n.get_clock().now().to_msg()
        for index,robot in enumerate(state['robots']):
            if not robot.get('registered',True):continue
            color=(.2,.55,1.) if index%2==0 else (1.,.48,.12)
            def marker(id_,type_,xyz=(0,0,0),scale=(1,1,1),text='',points=(),alpha=1.):
                m=Marker();m.header.frame_id='fleet_map';m.header.stamp=stamp;m.ns='fleet_ui_'+robot['id'];m.id=id_;m.type=type_
                m.pose.orientation.w=1.;m.pose.position.x,m.pose.position.y,m.pose.position.z=map(float,xyz)
                m.scale.x,m.scale.y,m.scale.z=map(float,scale);m.color.r,m.color.g,m.color.b=color;m.color.a=alpha
                m.text=text;m.points=[Point(x=float(p[0]),y=float(p[1]),z=.035) for p in points];m.lifetime.sec=2
                if not robot['active'] or (type_==Marker.LINE_STRIP and len(points)<2):m.action=Marker.DELETE
                array.markers.append(m);return m
            p=robot['pose'] if robot['online'] and robot['pose_age']<.8 else None
            # Remove the old hand-built proxy shapes. Geometry now comes only
            # from the same URDF used by the single-robot scene.
            for old_id in range(5):marker(old_id,Marker.CUBE).action=Marker.DELETE
            root=fleet_root(robot) if p is not None else None
            kinds={'box':Marker.CUBE,'cylinder':Marker.CYLINDER,'sphere':Marker.SPHERE,'mesh':Marker.MESH_RESOURCE}
            for i,part in enumerate(self.visuals):
                m=marker(10000+i,kinds[part['shape']],scale=part['size'])
                if root is None:m.action=Marker.DELETE;continue
                xyz,q=combine(root,part['transform'])
                m.pose.position.x,m.pose.position.y,m.pose.position.z=xyz
                m.pose.orientation.x,m.pose.orientation.y,m.pose.orientation.z,m.pose.orientation.w=q
                m.color.r,m.color.g,m.color.b,m.color.a=part['color']
                if part['shape']=='mesh':m.mesh_resource=part['mesh'];m.mesh_use_embedded_materials=True
            arrow=marker(20000,Marker.ARROW,scale=(.4,.04,.04))
            label=marker(20001,Marker.TEXT_VIEW_FACING,scale=(0.,0.,.17),text=f'{robot["rank"]} · {robot["name"]}')
            if root is None:arrow.action=label.action=Marker.DELETE
            else:
                for m,z in ((arrow,.28),(label,.5)):
                    m.pose.position.x,m.pose.position.y=p[:2];m.pose.position.z=root[0][2]+z
                arrow.pose.orientation.z=math.sin(p[2]/2);arrow.pose.orientation.w=math.cos(p[2]/2)
            preview=state['previews'].get(robot['id'],{}).get('points',[])
            marker(10,Marker.LINE_STRIP,scale=(.025,0.,0.),points=preview,alpha=.45)
            alignment=robot.get('alignment');actual=[compose(alignment,[*p,0.]) for p in robot['planned_path']] if alignment and p else []
            marker(11,Marker.LINE_STRIP,scale=(.045,0.,0.),points=actual)
            route=state['config']['routes'][robot['id']]
            for i,target in enumerate(route):
                marker(100+i,Marker.SPHERE,(target[0],target[1],.06),(.1,.1,.1))
                marker(300+i,Marker.TEXT_VIEW_FACING,(target[0],target[1],.25),(0.,0.,.13),str(i+1))
            for i in range(len(route),self.route_counts.get(robot['id'],0)):
                marker(100+i,Marker.SPHERE).action=Marker.DELETE
                marker(300+i,Marker.TEXT_VIEW_FACING).action=Marker.DELETE
            self.route_counts[robot['id']]=len(route)
        self.markers.publish(array)
