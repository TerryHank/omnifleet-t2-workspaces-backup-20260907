"""Set a measured stationary-map pose; no navigation or velocity is published."""
import argparse,json,math
from pathlib import Path
import yaml,rclpy
from rclpy.node import Node
from mola_msgs.srv import RelocalizeNearPose,MolaRuntimeParamGet,MolaRuntimeParamSet
p=argparse.ArgumentParser();p.add_argument('--pose',nargs=3,type=float,required=True);p.add_argument('--output',required=True);a=p.parse_args()
rclpy.init();node=Node('msc_saved_map_initialization')
def call(kind,name,request):
 client=node.create_client(kind,name);assert client.wait_for_service(timeout_sec=10),name
 future=client.call_async(request);rclpy.spin_until_future_complete(node,future,timeout_sec=12)
 assert future.done() and future.result() is not None,name
 return future.result()
before=yaml.safe_load(call(MolaRuntimeParamGet,'/mola_runtime_param_get',MolaRuntimeParamGet.Request()).parameters)
keys=[k for k in before if 'LidarOdometry' in k];assert len(keys)==1,keys
req=RelocalizeNearPose.Request();req.pose.header.frame_id='map';req.pose.header.stamp=node.get_clock().now().to_msg()
req.pose.pose.pose.position.x=a.pose[0];req.pose.pose.pose.position.y=a.pose[1]
req.pose.pose.pose.orientation.z=math.sin(a.pose[2]/2);req.pose.pose.pose.orientation.w=math.cos(a.pose[2]/2)
req.pose.pose.covariance[0]=.01;req.pose.pose.covariance[7]=.01;req.pose.pose.covariance[14]=.0001
req.pose.pose.covariance[21]=.0001;req.pose.pose.covariance[28]=.0001;req.pose.pose.covariance[35]=.0076
assert call(RelocalizeNearPose,'/relocalize_near_pose',req).accepted
change=MolaRuntimeParamSet.Request();change.parameters=yaml.safe_dump({keys[0]:{'active':True}})
result=call(MolaRuntimeParamSet,'/mola_runtime_param_set',change);assert result.success,result.error_message
after=yaml.safe_load(call(MolaRuntimeParamGet,'/mola_runtime_param_get',MolaRuntimeParamGet.Request()).parameters)
report={'seed':a.pose,'relocalization_request_accepted':True,'runtime_before':before,'runtime_after':after,'motion_command_sent':False}
Path(a.output).write_text(json.dumps(report,indent=2));print(json.dumps(report))
node.destroy_node();rclpy.try_shutdown()
