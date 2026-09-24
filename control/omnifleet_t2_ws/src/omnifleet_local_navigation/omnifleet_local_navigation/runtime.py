"""One onboard navigation owner; UI and fleet are clients of this node."""
import argparse,copy,json,math,os,time
from dataclasses import dataclass
from types import SimpleNamespace
import rclpy
from rclpy.action import ActionServer,ActionClient,CancelResponse
from rclpy.task import Future
from rclpy.qos import QoSProfile,DurabilityPolicy,ReliabilityPolicy
from nav2_msgs.action import NavigateToPose,NavigateThroughPoses
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
from std_srvs.srv import Trigger,SetBool
from omnifleet_navigation_interfaces.action import ExecuteNavigation
from omnifleet_navigation_interfaces.msg import NavigationPoint
from .control import LocalControl
from .route_execution import ContinuousRoute
from .scope import topic,frame
from .task_policy import TaskPolicy,Ticket

@dataclass
class Job:
    ticket: Ticket
    points: list
    single: bool
    radius: float
    handle: object
    action_type: object
    done: object
    tree: str = ''
    @property
    def source(self):return self.ticket.source

class LocalNavigation(LocalControl):
    def __init__(self,config):
        self.policy=TaskPolicy();self.current_job=None;self.pending_job=None
        self.navigation_active=False;self.inhibited=True;self.message='等待本地导航后端'
        self.armed=False
        self.fleet_enabled=False;self.completed={};self.cancel_deadline=0.
        self.fleet_permission_epoch=0
        self.stop_generation=0;self.accepted_generations={}
        super().__init__(config)
        self.frame_id=frame('map');self.pass_radius=.25
        self.preview_planner_id=config.get('default_planner','GridBased')
        self.preview_revision=0;self.preview_dirty=False;self.preview_goal_handle=None
        q=QoSProfile(depth=1,reliability=ReliabilityPolicy.RELIABLE,durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.path_pub=self.create_publisher(Path,'/omnifleet_t2/waypoints/path',q)
        self.status_pub=self.create_publisher(String,'/navigation/status',q)
        self.execution=ContinuousRoute(self)
        self.servers=[
            ActionServer(self,ExecuteNavigation,topic('/navigation/execute'),self.execute_task,cancel_callback=self.cancel_request,handle_accepted_callback=self.accept_handle),
            ActionServer(self,NavigateToPose,topic('/navigate_to_pose'),self.execute_single,cancel_callback=self.cancel_request,handle_accepted_callback=self.accept_handle),
            ActionServer(self,NavigateThroughPoses,topic('/navigate_through_poses'),self.execute_route,cancel_callback=self.cancel_request,handle_accepted_callback=self.accept_handle),
        ]
        self.create_service(Trigger,topic('/navigation/stop'),self.stop_request)
        self.create_service(SetBool,topic('/navigation/allow_fleet'),self.allow_fleet)
        self.create_service(Trigger,topic('/navigation/reset_estop'),self.reset_estop)
        self.topic_client=ActionClient(self,ExecuteNavigation,topic('/navigation/execute'))
        self.create_subscription(PoseStamped,'/goal_pose',self.goal_pose,10)
        self.create_timer(.05,self.update_jobs)

    def goal_pose(self,message):
        request=ExecuteNavigation.Goal();request.task_id='map-click-'+str(self.get_clock().now().nanoseconds)
        request.source=ExecuteNavigation.Goal.LOCAL;request.single_point=True
        p=NavigationPoint();p.name='地图目标';p.pose=message;p.kind=NavigationPoint.STOP;request.waypoints=[p]
        generation=self.stop_generation
        def accepted(f):
            try:
                h=f.result()
                if h.accepted and generation!=self.stop_generation:h.cancel_goal_async()
                elif not h.accepted:self.publish_status('本地导航模块拒绝地图目标')
            except Exception as error:self.publish_status('地图目标提交失败：'+str(error))
        self.topic_client.send_goal_async(request).add_done_callback(accepted)

    def inhibit_output(self):self.inhibited=True;self.armed=False;self.nav_time=0.

    def arm_navigation_output(self):
        if not self.armed:self.nav_time=0.
        self.armed=True

    def accept_handle(self,handle):
        self.accepted_generations[bytes(handle.goal_id.uuid).hex()]=self.stop_generation
        handle.execute()

    def output_permitted(self):
        return self.armed and not self.inhibited and super().output_permitted()

    def publish_status(self,text):
        if hasattr(self,'execution') and self.execution.active and self.execution.phase in ('tracking','replanning','preplanning') and not self.policy.canceling:self.inhibited=False
        self.message=text;self.get_logger().info(text);self.publish_catalog()

    def readiness_reason(self):
        if self.estop:return '急停或接触保护尚未解除'
        if self.policy.canceling or self.execution.phase in ('canceling','cancel_unconfirmed'):return '正在取消或确认停稳，尚不能启动后续任务'
        if not self.control_gate_ready:return '本地底盘控制通道尚未就绪'
        if not self.execution.single.server_is_ready() or not self.execution.navigator.server_is_ready():return '本地Nav2执行接口尚未就绪'
        if not all(name in self.states and self.states[name][0]==3 and time.monotonic()-self.states[name][1]<2 for name in self.state_clients):return 'Nav2生命周期尚未激活'
        if not self.startup_done:return '启动时的目标取消及停稳确认尚未完成'
        if not self.execution.inputs_ready():return self.execution.health.reason+'；同时需要新鲜MOLA位姿和代价地图'
        return ''

    def publish_catalog(self):
        if not hasattr(self,'execution'):return
        reason=self.readiness_reason()
        data={**self.execution.catalog(),'robot_id':self.config['robot_id'],
              'task_id':self.current_job.ticket.task_id if self.current_job else '',
              'source':self.current_job.source if self.current_job else 'NONE',
              'navigation_active':self.navigation_active,'local_ready':not bool(reason),
              'readiness_reason':reason,'message':self.message,'local_override':self.policy.local_override,
              'fleet_enabled':self.fleet_enabled,'control_mode':self.mode}
        self.status_pub.publish(String(data=json.dumps(data,ensure_ascii=False)))
        if self.current_job and self.current_job.action_type is ExecuteNavigation and self.current_job.handle.is_active:
            f=ExecuteNavigation.Feedback();f.task_id=self.current_job.ticket.task_id
            f.phase=self.execution.phase;f.message=self.message
            f.passed_points=sum(s in ('已通过','已停靠') for s in self.execution.states)
            f.tf_age_ms=float(self.execution.health.age_ms if self.execution.health.age_ms is not None else -1)
            self.current_job.handle.publish_feedback(f)

    def request_route_preview(self):pass

    def navigation_feedback(self,message):
        job=self.current_job
        if job and job.handle.is_active and job.action_type is not ExecuteNavigation:
            job.handle.publish_feedback(message.feedback)

    def points(self,values):
        result=[]
        for i,v in enumerate(values):
            stamped=v.pose if hasattr(v,'kind') else v
            if stamped.header.frame_id not in (self.frame_id,'map'):raise ValueError('目标必须在本车map坐标系')
            p=stamped.pose;q=p.orientation
            nums=[p.position.x,p.position.y,p.position.z,q.x,q.y,q.z,q.w]
            if not all(math.isfinite(x) for x in nums) or abs(sum(x*x for x in nums[3:])-1)>.01:raise ValueError('目标坐标或四元数无效')
            kind=getattr(v,'kind',0);wait=float(getattr(v,'dwell_seconds',0))
            if kind not in (0,1) or not math.isfinite(wait) or not 0<=wait<=600:raise ValueError('航点类型或等待时间无效')
            result.append(SimpleNamespace(name=getattr(v,'name','') or f'航点{i+1}',pose=copy.deepcopy(p),kind='stop' if kind==1 else 'pass',dwell_seconds=wait))
        if not 1<=len(result)<=100:raise ValueError('航点数量必须为1～100')
        result[-1].kind='stop'
        return result

    async def execute_task(self,handle):
        request=handle.request
        ticket=Ticket(request.task_id or bytes(handle.goal_id.uuid).hex(),{0:'LOCAL',1:'FLEET'}.get(request.source,'INVALID'),request.revision,request.command_epoch)
        return await self.admit(handle,ExecuteNavigation,ticket,request.waypoints,request.single_point or len(request.waypoints)==1,request.pass_radius)

    async def execute_single(self,handle):
        return await self.admit(handle,NavigateToPose,Ticket(bytes(handle.goal_id.uuid).hex(),'LOCAL'),[handle.request.pose],True,.25,handle.request.behavior_tree)

    async def execute_route(self,handle):
        return await self.admit(handle,NavigateThroughPoses,Ticket(bytes(handle.goal_id.uuid).hex(),'LOCAL'),handle.request.poses,False,.25,handle.request.behavior_tree)

    async def admit(self,handle,action_type,ticket,values,single,radius,tree=''):
        job=Job(ticket,[],single,radius,handle,action_type,Future(),tree)
        try:
            generation=self.accepted_generations.pop(bytes(handle.goal_id.uuid).hex(),self.stop_generation)
            if handle.is_cancel_requested or generation!=self.stop_generation:
                self.finish_job(job,False,'CANCELED','目标接受前已被取消')
                return await job.done
            job.points=self.points(values)
            if not 0.05<=radius<=.5 or not math.isfinite(radius):raise ValueError('途经半径必须为0.05～0.50米')
            if single and len(job.points)!=1:raise ValueError('单点任务只允许一个目标')
            if not ticket.task_id or len(ticket.task_id)>160:raise ValueError('任务编号无效')
            if ticket in self.completed:
                ok,code,message=self.completed[ticket];self.finish_job(job,ok,code,message)
                return await job.done
            reason=self.readiness_reason()
            if reason:raise RuntimeError(reason)
            if self.mode=='MANUAL':
                if time.monotonic()-self.manual_time<.35 or not self.stationary():raise RuntimeError('人工遥控尚未停止')
                self.mode='LOCAL';self.manual_blocked_goals.clear()
            if ticket.source=='FLEET':
                if not self.fleet_enabled:raise RuntimeError('尚未显式允许协同任务；本地导航不受此开关限制')
                if time.monotonic()-self.last_response>.6:raise RuntimeError('协同租约不新鲜')
                command=self.response.get('command',{})
                if command.get('kind')!='navigate' or command.get('epoch')!=ticket.command_epoch or int(command.get('seq',0))!=ticket.revision:
                    self.finish_job(job,False,'STALE_FLEET_COMMAND','协同指令已撤销或被较新指令替代')
                    return await job.done
                existing=self.current_job
                if existing and existing.source=='FLEET' and ticket.task_id==existing.ticket.task_id and ticket.revision>existing.ticket.revision:
                    stop_first=not (single and existing.single and self.execution.phase=='tracking')
                    if self.policy.update_fleet(ticket,stop_first):
                        if stop_first:
                            self.pending_job=job;self.inhibited=True;self.cancel_deadline=time.monotonic()+5
                            self.execution.stop('协同路线已修改，取消旧路线后重新预检')
                        else:self.update_single(job)
                        return await job.done
            decision=self.policy.request(ticket)
            if decision=='START':self.start_job(job)
            elif decision=='CANCEL_THEN_TAKEOVER':
                self.pending_job=job;self.mode='TAKEOVER';self.inhibited=True;self.cancel_deadline=time.monotonic()+5
                self.execution.stop('本地请求接管：取消当前任务并等待底盘停稳')
            else:self.finish_job(job,False,decision,decision)
        except ValueError as error:self.finish_job(job,False,'INVALID_GOAL',str(error))
        except RuntimeError as error:self.finish_job(job,False,'NOT_READY',str(error))
        return await job.done

    def start_job(self,job):
        self.current_job=job;self.pending_job=None;self.inhibited=False;self.goal_error=''
        self.armed=False;self.nav_time=0.
        self.mode=job.source;self.fleet_hold=False;self.pass_radius=job.radius
        p=job.points[-1].pose;q=p.orientation
        self.goal=[p.position.x,p.position.y,math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))]
        if job.source=='FLEET':self.applied_seq=job.ticket.revision;self.command_epoch=job.ticket.command_epoch
        self.preview_planner_id=self.selection.get('planner',self.preview_planner_id)
        self.execution.native_behavior_tree=job.tree
        self.execution.start(job.points,single_point=job.single)
        if not self.execution.active:
            self.policy.finish(job.ticket);self.current_job=None;self.inhibited=True
            self.finish_job(job,False,'NOT_READY',self.message)

    def update_single(self,job):
        # Same fleet task, newer target: use Nav2's native preemption. This is
        # not an ownership handover and must not stop on every leader update.
        old=self.current_job;self.current_job=job
        self.execution.epoch+=1;self.execution.points=job.points;self.execution.states=['正在通过']
        self.execution.native_behavior_tree=job.tree
        self.applied_seq=job.ticket.revision;self.command_epoch=job.ticket.command_epoch
        p=job.points[-1].pose;q=p.orientation
        self.goal=[p.position.x,p.position.y,math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))]
        self.finish_job(old,False,'TARGET_UPDATED','同一协同任务已更新目标')
        self.execution.send_block()

    def finish_job(self,job,success,code,message):
        if job.done.done():return
        if job.handle.is_active:
            if success:job.handle.succeed()
            elif job.handle.is_cancel_requested:job.handle.canceled()
            else:job.handle.abort()
        result=job.action_type.Result()
        if job.action_type is ExecuteNavigation:
            result.success=success;result.code=code;result.message=message
        if code not in ('NOT_READY','BUSY','BUSY_CANCELING'):self.completed[job.ticket]=(success,code,message)
        while len(self.completed)>128:self.completed.pop(next(iter(self.completed)))
        job.done.set_result(result)

    def cancel_request(self,handle):
        if self.pending_job and self.pending_job.handle==handle:
            job=self.pending_job;self.pending_job=None;self.policy.pending=None
            # Result is completed by the next timer after ROS marks canceling.
            self.cancelled_pending=job
        elif self.current_job and self.current_job.handle==handle:self.stop_owned('调用方取消导航')
        return CancelResponse.ACCEPT

    def stop_owned(self,reason):
        if self.pending_job:self.finish_job(self.pending_job,False,'CANCELED',reason);self.pending_job=None
        self.policy.cancel();self.inhibited=True;self.cancel_deadline=time.monotonic()+5
        if self.execution.active:self.execution.stop(reason)

    def stop_request(self,request,response):
        self.stop_generation+=1
        self.manual=[0.,0.];self.manual_time=0.
        if self.mode=='MANUAL':self.mode='LOCAL'
        self.stop_owned('用户停止本车导航')
        response.success=True;response.message='已请求停止；请观察状态确认取消和停稳'
        return response

    def reset_estop(self,request,response):
        response.success=not self.estop_input and not self.contact and self.stationary() and not self.navigation_active and not self.execution.requests
        if response.success:
            from .storage import save
            self.safety_latched=False;self.estop=False
            save(self.state_path,{'safety_latched':False})
        response.message='已解除本地急停锁存' if response.success else '需释放物理急停/接触输入，取消导航并确认停稳'
        return response

    def allow_fleet(self,request,response):
        if not request.data:
            self.fleet_enabled=False;self.policy.local_override=True
            if self.current_job and self.current_job.source=='FLEET':self.stop_owned('用户退出协同')
            response.success=True;response.message='协同任务已禁用，本地导航仍可调用'
        else:
            response.success=not self.estop and self.policy.release_to_fleet(self.stationary())
            if response.success:self.fleet_enabled=True;self.fleet_permission_epoch+=1;self.goal_error=''
            response.message='已允许新的协同任务' if response.success else '必须先完成/停止本地任务并确认停稳'
        return response

    def update_jobs(self):
        if hasattr(self,'cancelled_pending'):
            self.finish_job(self.cancelled_pending,False,'CANCELED','已取消待接管任务');del self.cancelled_pending
        job=self.current_job
        if job:
            if self.execution.phase in ('canceling','cancel_unconfirmed') and not self.policy.canceling:
                self.policy.cancel();self.cancel_deadline=time.monotonic()+5;self.inhibited=True
            if self.policy.canceling:
                if not self.execution.active and not self.execution.requests and self.stationary():
                    next_job=self.pending_job
                    code=('PREEMPTED_BY_LOCAL' if next_job.source=='LOCAL' else 'ROUTE_UPDATED') if next_job else ('NAVIGATION_FAILED' if getattr(self.execution,'failed',False) else 'CANCELED')
                    self.finish_job(job,False,code,self.message)
                    self.policy.canceled_and_stopped();self.current_job=None;self.pending_job=None
                    if next_job:self.start_job(next_job)
                    elif self.mode=='TAKEOVER':self.mode='LOCAL'
                elif time.monotonic()>self.cancel_deadline:
                    self.inhibited=True;self.message='取消或停稳尚未确认，不允许后续目标启动'
                    if self.pending_job:self.finish_job(self.pending_job,False,'CANCEL_UNCONFIRMED',self.message);self.pending_job=None;self.policy.pending=None
                    self.finish_job(job,False,'CANCEL_UNCONFIRMED',self.message)
            elif not self.execution.active:
                success=self.execution.phase=='completed'
                self.finish_job(job,success,'SUCCEEDED' if success else 'NAVIGATION_FAILED',self.message)
                if job.source=='FLEET':
                    if success:self.completed_seq=job.ticket.revision
                    else:self.goal_error=self.message
                self.policy.finish(job.ticket);self.current_job=None;self.inhibited=True
        self.publish_catalog()

def main():
    p=argparse.ArgumentParser();p.add_argument('--config',required=True);args,ros=p.parse_known_args()
    with open(args.config) as f:config=json.load(f)
    os.environ['OMNIFLEET_ROBOT_ID']=config['robot_id']
    rclpy.init(args=ros);node=LocalNavigation(config)
    try:rclpy.spin(node)
    except (KeyboardInterrupt,rclpy.executors.ExternalShutdownException):pass
    finally:
        node.inhibited=True
        if rclpy.ok():node.cancel_all()
        node.destroy_node()
        if rclpy.ok():rclpy.try_shutdown()
