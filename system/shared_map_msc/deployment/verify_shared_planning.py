"""Only ComputePathToPose is used; never NavigateToPose or velocity publishing."""
import json,math,time
from pathlib import Path
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rcl_interfaces.srv import GetParameters
from nav_msgs.msg import OccupancyGrid
from nav2_msgs.action import ComputePathToPose
from geometry_msgs.msg import PoseStamped,Twist
from tf2_ros import Buffer,TransformListener
from omnifleet_msc.shared_map import QOS
rclpy.init();node=Node('msc_shared_planning_probe');tf=Buffer();listener=TransformListener(tf,node);grids=[];commands=[]
node.create_subscription(OccupancyGrid,'/global_costmap/costmap',grids.append,QOS)
node.create_subscription(Twist,'/motor_command_sent',lambda m:commands.append([m.linear.x,m.angular.z]),100)
def wait(future,seconds=8):
 rclpy.spin_until_future_complete(node,future,timeout_sec=seconds)
 assert future.done(),'service/action deadline';return future.result()
def params(name,keys):
 client=node.create_client(GetParameters,name+'/get_parameters');assert client.wait_for_service(timeout_sec=5)
 req=GetParameters.Request();req.names=keys;return wait(client.call_async(req)).values
end=time.monotonic()+5
while time.monotonic()<end:rclpy.spin_once(node,timeout_sec=.1)
values=params('/global_costmap/global_costmap',['global_frame','static_layer.map_topic'])
assert [v.string_value for v in values]==['fleet_map','/fleet/map']
planners=params('/planner_server',['planner_plugins'])[0].string_array_value
assert grids and grids[-1].header.frame_id=='fleet_map';grid=grids[-1]
t=tf.lookup_transform('fleet_map','base_link',rclpy.time.Time());x=t.transform.translation.x;y=t.transform.translation.y
candidates=[];res=grid.info.resolution;ox=grid.info.origin.position.x;oy=grid.info.origin.position.y
for row in range(grid.info.height):
 for col in range(grid.info.width):
  v=grid.data[row*grid.info.width+col];px=ox+(col+.5)*res;py=oy+(row+.5)*res;distance=math.hypot(px-x,py-y)
  if 0<=v<40 and .7<distance<1.3:candidates.append((v+abs(distance-1.),px,py))
assert candidates,'no static planning candidate near current pose'
action=ActionClient(node,ComputePathToPose,'/compute_path_to_pose');assert action.wait_for_server(timeout_sec=5)
result=None
for _,gx,gy in sorted(candidates)[:8]:
 goal=ComputePathToPose.Goal();goal.use_start=True;goal.planner_id=planners[0]
 for pose_,px,py in ((goal.start,x,y),(goal.goal,gx,gy)):
  pose_.header.frame_id='fleet_map';pose_.header.stamp=node.get_clock().now().to_msg()
  pose_.pose.position.x=px;pose_.pose.position.y=py;pose_.pose.orientation.w=1.
 handle=wait(action.send_goal_async(goal));assert handle.accepted
 response=wait(handle.get_result_async())
 if response.status==4 and response.result.path.poses:
  result={'planner_id':goal.planner_id,'start':[x,y],'goal':[gx,gy],'frame':response.result.path.header.frame_id,'poses':len(response.result.path.poses)};break
assert result,'no planning request succeeded'
assert commands and all(v==0 and w==0 for v,w in commands)
result.update(motor_samples=len(commands),nonzero_commands=0,global_frame='fleet_map',map_topic='/fleet/map')
Path('/home/iecme/robot_backups/msc_shared_map_dsh_20260909/planning.json').write_text(json.dumps(result,indent=2))
print(json.dumps(result));node.destroy_node();rclpy.try_shutdown()
