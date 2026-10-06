import json,math,time,uuid,threading
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path
from std_msgs.msg import String
from geometry_msgs.msg import PointStamped,PoseStamped,PoseWithCovarianceStamped,Point
from visualization_msgs.msg import Marker,MarkerArray
from rclpy.qos import QoSProfile,DurabilityPolicy,ReliabilityPolicy
from rclpy.action import ActionClient
from omnifleet_navigation_interfaces.action import ExecuteNavigation
from omnifleet_navigation_interfaces.msg import NavigationPoint
from .panel_model import FleetPanelModel
from .online_robots import DomainVehicleDiscovery
from .geometry import compose
from .panel_routing import navigation_mode,pose_in_robot_map,covariance_in_robot_map,local_goal_rejection
from .urdf_visuals import load_visuals,combine,fleet_root
from ament_index_python.packages import get_package_share_directory
from std_srvs.srv import SetBool

class FleetPanelIO:
    def __init__(self,node):
        self.n=node;self.model=FleetPanelModel(node.core,Path(node.config['journal']).with_name('foxglove-fleet.json'))
        self.discovery=DomainVehicleDiscovery(node)
        self.visuals=load_visuals(Path(get_package_share_directory('omnifleet_description'))/'urdf/omnifleet_t2.urdf')
        self.operation_epoch=0;self.allow_clients={}
        self.local_action_clients={};self.local_handles={};self.local_task_ids={};self.initialpose_publishers={}
        self.pending_stops={};self.pending_local_cancels={}
        self.ack={};self.seen={};self.pool=ThreadPoolExecutor(max_workers=1);self.future=None;self.future_id=None;self.route_counts={}
        self.last_state_time=0.
        q=QoSProfile(depth=1,reliability=ReliabilityPolicy.RELIABLE,durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.state_pub=node.create_publisher(String,'/fleet/ui/state',q)
        self.markers=node.create_publisher(MarkerArray,'/fleet/ui/markers',q)
        node.create_subscription(String,'/fleet/ui/command',self.command,10)
        node.create_subscription(PointStamped,'/fleet/ui/add_point',self.point,10)
        node.create_subscription(PoseStamped,'/fleet/ui/add_pose',self.point,10)
        node.create_subscription(PoseWithCovarianceStamped,'/fleet/ui/set_initialpose',self.initial_pose,10)
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
            if req.get('op')=='stop' and not self.model.busy():
                local=sorted(self.local_task_ids)
                if local:
                    self.request_local_stop(id_,local);self.seen[id_]=self.ack;self.tick();return
            with self.n.lock:result=self.model.command(req)
            self.ack={'id':id_,'ok':True,'message':result if isinstance(result,str) else '请求已受理，请观察任务状态'}
        except Exception as error:self.ack={'id':id_,'ok':False,'message':str(error)}
        self.seen[id_]=self.ack
        while len(self.seen)>128:self.seen.pop(next(iter(self.seen)))
        self.tick()

    def state_snapshot(self,refresh=False):
        if refresh:self.last_state_time=time.monotonic()
        with self.n.lock:state=self.discovery.decorate(self.model.state())
        fresh=self.last_state_time>0 and time.monotonic()-self.last_state_time<=2.
        mode=navigation_mode(state['robots'],fresh)
        state['routing_mode']=mode['mode'];state['routing_reason']=mode['reason']
        state['online_count']=mode['online_count'];state['single_robot_id']=mode['robot_id']
        state['local_busy']=any(r.get('local_busy') for r in state['robots']) or bool(self.local_task_ids)
        state['local_cancel_available']=bool(self.local_task_ids)
        return state

    def request_local_stop(self,request_id,robots):
        self.pending_stops[request_id]={'remaining':set(robots),'errors':[]}
        self.ack={'id':request_id,'pending':True,'message':'正在请求停止本地导航'}
        for robot in robots:
            if robot in self.pending_local_cancels:
                self._local_stop_result(request_id,robot,error=ValueError(robot+' 已有停止请求等待确认'))
                continue
            handle=self.local_handles.get(robot)
            if handle is None and self.local_task_ids.get(robot):
                self.pending_local_cancels[robot]=request_id
                continue
            if handle is None:
                self._local_stop_result(request_id,robot,error=ValueError(robot+' 没有统一地图面板提交的可取消目标'))
                continue
            self.pending_local_cancels[robot]=request_id
            self._cancel_local_goal(request_id,robot,handle)

    def _cancel_local_goal(self,request_id,robot,handle):
        try:
            handle.cancel_goal_async().add_done_callback(
                lambda future,rid=request_id,r=robot:self._local_cancel_response(rid,r,future))
        except Exception as error:self._local_stop_result(request_id,robot,error=error)

    def _local_cancel_response(self,request_id,robot,future):
        try:
            response=future.result()
            if response.return_code!=0 or not response.goals_canceling:
                raise ValueError(robot+' 本地导航未确认取消')
            self._local_stop_result(request_id,robot)
        except Exception as error:self._local_stop_result(request_id,robot,error=error)

    def _local_stop_result(self,request_id,robot,error=None):
        pending=self.pending_stops.get(request_id)
        if pending is None or robot not in pending['remaining']:return
        if self.pending_local_cancels.get(robot)==request_id:self.pending_local_cancels.pop(robot,None)
        pending['remaining'].remove(robot)
        if error:pending['errors'].append(str(error))
        if pending['remaining']:return
        self.pending_stops.pop(request_id,None)
        ok=not pending['errors']
        message='取消请求已确认；请等待导航结束并确认停稳' if ok else '停止请求失败：'+'；'.join(pending['errors'])
        self.ack={'id':request_id,'ok':ok,'message':message};self.seen[request_id]=self.ack;self.tick()

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
            state=self.state_snapshot();mode=state['routing_mode']
            if mode=='WAITING':raise ValueError(state['routing_reason'])
            if message.header.frame_id not in ('fleet_map',self.n.config['reference_robot']+'/map'):
                raise ValueError('目标必须位于共享地图坐标系')
            robot_row=None
            if mode=='SINGLE':
                robot_row=next(r for r in state['robots'] if r['id']==state['single_robot_id'])
            if hasattr(message,'point'):
                p=message.point
                if mode=='SINGLE':angle=robot_row['pose'][2]
                else:
                    r=self.model.config['selected'];route=self.model.config['routes'].get(r,[])
                    angle=math.atan2(p.y-route[-1][1],p.x-route[-1][0]) if route else 0.
            else:
                p=message.pose.position;q=message.pose.orientation
                angle=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
            value=[p.x,p.y,angle]
            if mode=='SINGLE':self.send_local_goal(robot_row,value)
            else:
                selected=next((r for r in state['robots'] if r['id']==self.model.config['selected']),None)
                if selected is None or not selected.get('online') or selected.get('pose') is None or selected.get('pose_age',99)>.8:
                    raise ValueError('请先选择一辆在线且已在共享地图中定位的车辆')
                with self.n.lock:result=self.model.add_point(value)
                self.ack={'id':'map-point','ok':True,'message':result}
        except Exception as error:self.ack={'id':'map-point','ok':False,'message':str(error)}
        self.tick()

    def send_local_goal(self,robot,value):
        robot_id=robot['id'];core_state=self.n.core.robot(robot_id)
        client=self.local_action_clients.get(robot_id)
        if client is None:
            client=ActionClient(self.n,ExecuteNavigation,'/'+robot_id+'/navigation/execute')
            self.local_action_clients[robot_id]=client
        admission_row=dict(robot,local_busy=(robot.get('local_busy') or robot_id in self.local_task_ids or
                                              robot_id in self.pending_local_cancels))
        rejection=local_goal_rejection(admission_row,core_state,client.server_is_ready(),
                                       self.n.config['reference_robot'],self.model.busy())
        if rejection:raise ValueError(rejection)
        local=pose_in_robot_map(value,robot.get('alignment'),robot_id,self.n.config['reference_robot'])
        task_id='foxglove-local-'+robot_id+'-'+uuid.uuid4().hex[:16]
        request=ExecuteNavigation.Goal();request.task_id=task_id;request.command_epoch='local-'+uuid.uuid4().hex
        request.revision=0;request.source=ExecuteNavigation.Goal.LOCAL;request.single_point=True;request.pass_radius=.25
        waypoint=NavigationPoint();waypoint.name='统一地图目标';waypoint.kind=NavigationPoint.STOP
        waypoint.pose.header.frame_id=robot_id+'/map';waypoint.pose.header.stamp=self.n.get_clock().now().to_msg()
        waypoint.pose.pose.position.x=local[0];waypoint.pose.pose.position.y=local[1]
        waypoint.pose.pose.orientation.z=math.sin(local[2]/2);waypoint.pose.pose.orientation.w=math.cos(local[2]/2)
        request.waypoints=[waypoint]
        future=client.send_goal_async(request);self.local_task_ids[robot_id]=task_id
        future.add_done_callback(lambda result,r=robot_id,t=task_id:self._local_goal_response(r,t,result))
        self.ack={'id':'map-point','pending':True,'message':robot_id+' 单车目标已提交给本地 Nav2'}

    def _local_goal_response(self,robot,task_id,future):
        try:
            handle=future.result()
            if self.local_task_ids.get(robot)!=task_id:
                if handle.accepted:handle.cancel_goal_async()
                return
            if not handle.accepted:
                self.local_task_ids.pop(robot,None)
                self.ack={'id':'map-point','ok':False,'message':robot+' 本地导航拒绝了目标'}
                cancel_request=self.pending_local_cancels.get(robot)
                if cancel_request:self._local_stop_result(cancel_request,robot,error=ValueError(robot+' 导航未接受目标'))
            else:
                self.local_handles[robot]=handle
                self.ack={'id':'map-point','ok':True,'message':robot+' 已接受单车导航目标'}
                handle.get_result_async().add_done_callback(
                    lambda result,r=robot,t=task_id:self._local_goal_result(r,t,result))
                cancel_request=self.pending_local_cancels.get(robot)
                if cancel_request:self._cancel_local_goal(cancel_request,robot,handle)
        except Exception as error:
            if self.local_task_ids.get(robot)==task_id:self.local_task_ids.pop(robot,None)
            cancel_request=self.pending_local_cancels.get(robot)
            if cancel_request:self._local_stop_result(cancel_request,robot,error=error)
            self.ack={'id':'map-point','ok':False,'message':robot+' 本地导航提交失败：'+str(error)}
        self.tick()

    def _local_goal_result(self,robot,task_id,future):
        if self.local_task_ids.get(robot)!=task_id:return
        self.local_task_ids.pop(robot,None);self.local_handles.pop(robot,None)
        try:
            result=future.result().result
            self.ack={'id':'map-point','ok':bool(result.success),'message':robot+' 单车导航'+('完成' if result.success else '结束：'+result.message)}
        except Exception as error:self.ack={'id':'map-point','ok':False,'message':robot+' 单车导航结果读取失败：'+str(error)}
        self.tick()

    def initial_pose(self,message):
        try:
            state=self.state_snapshot();mode=state['routing_mode']
            if mode=='WAITING':raise ValueError(state['routing_reason'])
            if message.header.frame_id not in ('fleet_map',self.n.config['reference_robot']+'/map'):
                raise ValueError('初始位姿必须位于共享地图坐标系')
            if mode=='SINGLE':robot_id=state['single_robot_id']
            else:robot_id=self.model.config['selected']
            robot=next((r for r in state['robots'] if r['id']==robot_id and r.get('registered',True)),None)
            if robot is None or not robot.get('online'):raise ValueError('请选择一辆在线且已登记的车辆')
            if robot.get('local_busy') or self.model.robot_busy(robot_id):raise ValueError(robot_id+' 正在执行任务，不能重置初始位姿')
            core_state=self.n.core.robot(robot_id)
            if core_state.get('estop') or core_state.get('manual_active') or not core_state.get('pose_stationary'):
                raise ValueError(robot_id+' 需解除急停/手动控制并确认停稳')
            alignment=robot.get('alignment')
            local=pose_in_robot_map([message.pose.pose.position.x,message.pose.pose.position.y,
                math.atan2(2*(message.pose.pose.orientation.w*message.pose.pose.orientation.z+
                              message.pose.pose.orientation.x*message.pose.pose.orientation.y),
                           1-2*(message.pose.pose.orientation.y**2+message.pose.pose.orientation.z**2))],
                alignment,robot_id,self.n.config['reference_robot'])
            output=PoseWithCovarianceStamped();output.header.frame_id=robot_id+'/map';output.header.stamp=self.n.get_clock().now().to_msg()
            output.pose.pose.position.x=local[0];output.pose.pose.position.y=local[1]
            output.pose.pose.orientation.z=math.sin(local[2]/2);output.pose.pose.orientation.w=math.cos(local[2]/2)
            yaw=0. if robot_id==self.n.config['reference_robot'] and alignment is None else alignment[2]
            output.pose.covariance=covariance_in_robot_map(message.pose.covariance,yaw)
            publisher=self.initialpose_publishers.get(robot_id)
            if publisher is None:
                publisher=self.n.create_publisher(PoseWithCovarianceStamped,'/'+robot_id+'/initialpose',10)
                self.initialpose_publishers[robot_id]=publisher
            publisher.publish(output)
            self.ack={'id':'map-initialpose','ok':True,'message':robot_id+' 初始位姿已发送'}
        except Exception as error:self.ack={'id':'map-initialpose','ok':False,'message':str(error)}
        self.tick()

    def tick(self):
        if self.future is not None and self.future.done():
            try:self.ack={'id':self.future_id,'ok':True,'message':self.future.result()}
            except Exception as error:self.ack={'id':self.future_id,'ok':False,'message':str(error)}
            self.seen[self.future_id]=self.ack;self.future=None
        state=self.state_snapshot(refresh=True)
        state['ack']=self.ack;self.state_pub.publish(String(data=json.dumps(state,ensure_ascii=False,allow_nan=False)))
        array=MarkerArray();stamp=self.n.get_clock().now().to_msg()
        for index,robot in enumerate(state['robots']):
            if not robot.get('registered',True):continue
            color=(.2,.55,1.) if index%2==0 else (1.,.48,.12)
            def marker(id_,type_,xyz=(0,0,0),scale=(1,1,1),text='',points=(),alpha=1.,enabled=True):
                m=Marker();m.header.frame_id='fleet_map';m.header.stamp=stamp;m.ns='fleet_ui_'+robot['id'];m.id=id_;m.type=type_
                m.pose.orientation.w=1.;m.pose.position.x,m.pose.position.y,m.pose.position.z=map(float,xyz)
                m.scale.x,m.scale.y,m.scale.z=map(float,scale);m.color.r,m.color.g,m.color.b=color;m.color.a=alpha
                m.text=text;m.points=[Point(x=float(p[0]),y=float(p[1]),z=.035) for p in points];m.lifetime.sec=2
                if not enabled or (type_==Marker.LINE_STRIP and len(points)<2):m.action=Marker.DELETE
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
            label=marker(20001,Marker.TEXT_VIEW_FACING,scale=(0.,0.,.17),
                         text=(f'{robot["rank"]} · ' if robot.get('rank') else '')+robot['name'])
            if root is None:arrow.action=label.action=Marker.DELETE
            else:
                for m,z in ((arrow,.28),(label,.5)):
                    m.pose.position.x,m.pose.position.y=p[:2];m.pose.position.z=root[0][2]+z
                arrow.pose.orientation.z=math.sin(p[2]/2);arrow.pose.orientation.w=math.cos(p[2]/2)
            preview=state['previews'].get(robot['id'],{}).get('points',[])
            marker(10,Marker.LINE_STRIP,scale=(.025,0.,0.),points=preview,alpha=.45,enabled=robot['active'])
            alignment=robot.get('alignment');actual=[compose(alignment,[*p,0.]) for p in robot['planned_path']] if alignment and p else []
            marker(11,Marker.LINE_STRIP,scale=(.045,0.,0.),points=actual,enabled=robot['active'])
            route=state['config']['routes'][robot['id']]
            for i,target in enumerate(route):
                marker(100+i,Marker.SPHERE,(target[0],target[1],.06),(.1,.1,.1),enabled=robot['active'])
                marker(300+i,Marker.TEXT_VIEW_FACING,(target[0],target[1],.25),(0.,0.,.13),str(i+1),enabled=robot['active'])
            for i in range(len(route),self.route_counts.get(robot['id'],0)):
                marker(100+i,Marker.SPHERE).action=Marker.DELETE
                marker(300+i,Marker.TEXT_VIEW_FACING).action=Marker.DELETE
            self.route_counts[robot['id']]=len(route)
        self.markers.publish(array)
