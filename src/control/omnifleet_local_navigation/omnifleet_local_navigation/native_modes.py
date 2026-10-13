"""Native Nav2 execution inside the existing FLEET navigation owner."""
import hashlib
import math
from pathlib import Path
from xml.sax.saxutils import quoteattr

from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.task import Future

from .scope import topic

NATIVE_BACKENDS = ('nav2_single', 'nav2_dynamic', 'nav2_waypoints')


def native_tree(planner, dynamic=False):
    compute = '<ComputePathToPose goal="{updated_goal}" path="{path}" planner_id='+quoteattr(planner)+'/>' if dynamic else '<ComputePathToPose goal="{goal}" path="{path}" planner_id='+quoteattr(planner)+'/>'
    if dynamic:
        compute = '<GoalUpdater input_goal="{goal}" output_goal="{updated_goal}">'+compute+'</GoalUpdater>'
    follow = '<FollowPath path="{path}" controller_id="FleetMPPI" goal_checker_id="fleet_goal_checker"/>'
    if dynamic:
        follow = '<KeepRunningUntilFailure>'+follow+'</KeepRunningUntilFailure>'
    text = '<root main_tree_to_execute="MainTree"><BehaviorTree ID="MainTree"><PipelineSequence><RateController hz="1.0">'+compute+'</RateController>'+follow+'</PipelineSequence></BehaviorTree></root>'
    folder = Path.home()/'.local/share/omnifleet_t2/native-modes'
    folder.mkdir(parents=True, exist_ok=True)
    path = folder/(hashlib.sha256(text.encode()).hexdigest()[:16]+'.xml')
    path.write_text(text)
    return str(path)


class FleetWaypointProxy:
    """Waypoint Follower children reuse the parent's FLEET lease and output gate."""
    def __init__(self, node):
        self.n=node;self.pending=[];self.index=0;self.owner=None
        self.server=ActionServer(node,NavigateToPose,topic('/navigation_backend/fleet_waypoint_to_pose'),
            self.execute,goal_callback=self.accept,cancel_callback=lambda _:CancelResponse.ACCEPT)

    def reset(self, job):
        self.owner=job;self.index=0

    def valid(self, request):
        job=self.n.current_job
        if not (job is self.owner and job and job.source=='FLEET' and job.backend=='nav2_waypoints' and
                self.n.execution.active and self.n.execution.phase=='tracking' and
                self.n.execution.strategy=='nav2_waypoints' and self.n.agent_lease_fresh() and
                self.n.agent_command_current(job.ticket,allow_newer=True)):
            return False
        if self.index>=len(job.points) or request.pose.header.frame_id!=self.n.frame_id:
            return False
        stamp=request.pose.header.stamp.sec*10**9+request.pose.header.stamp.nanosec
        if not getattr(job,'native_dispatch_stamp',0) or stamp<job.native_dispatch_stamp:return False
        expected=job.points[self.index].pose;actual=request.pose.pose
        a,b=actual.orientation,expected.orientation
        return (math.hypot(actual.position.x-expected.position.x,actual.position.y-expected.position.y)<1e-5 and
                abs(abs(a.x*b.x+a.y*b.y+a.z*b.z+a.w*b.w)-1.)<1e-5)

    def accept(self, request):
        return GoalResponse.ACCEPT if self.valid(request) else GoalResponse.REJECT

    async def execute(self, handle):
        if not self.valid(handle.request):
            handle.abort();return NavigateToPose.Result()
        job=self.owner;done=Future();entry={'job':job,'handle':handle,'done':done,'key':None}
        goal=NavigateToPose.Goal();goal.pose=handle.request.pose
        goal.behavior_tree=native_tree(self.n.execution.planner_id)
        def completed(_result):
            if done.done():return
            self.index+=1
            if handle.is_active:handle.succeed()
            done.set_result(NavigateToPose.Result())
        entry['key']=self.n.execution.submit(self.n.execution.single,goal,completed,'navigation')
        self.pending.append(entry)
        return await done

    def tick(self):
        for entry in list(self.pending):
            if entry['done'].done():self.pending.remove(entry);continue
            handle=entry['handle'];request=self.n.execution.requests.get(entry['key'])
            invalid=(self.n.current_job is not entry['job'] or not self.n.execution.active or
                     self.n.execution.phase in ('canceling','cancel_unconfirmed') or handle.is_cancel_requested)
            if not invalid and request is not None:continue
            if request is not None:self.n.execution.cancel(request)
            if handle.is_active:
                if handle.is_cancel_requested:handle.canceled()
                else:handle.abort()
            entry['done'].set_result(NavigateToPose.Result());self.pending.remove(entry)
