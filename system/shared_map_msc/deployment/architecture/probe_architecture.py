"""Static architecture acceptance: source map, TF ownership, real driver output, lifecycle."""
import argparse,json,time,hashlib,math
from pathlib import Path
import rclpy,yaml
from rclpy.node import Node
from rclpy.time import Time
from rclpy.executors import SingleThreadedExecutor
from tf2_ros import Buffer,TransformListener
from tf2_msgs.msg import TFMessage
from nav_msgs.msg import OccupancyGrid,Odometry
from geometry_msgs.msg import Twist
from std_msgs.msg import String,Float32,Float64
from diagnostic_msgs.msg import DiagnosticArray
from mola_msgs.srv import MolaRuntimeParamGet
from rcl_interfaces.srv import GetParameters
from omnifleet_msc.shared_map import encode_grid,QOS
p=argparse.ArgumentParser();p.add_argument('--mode',choices=['pure_mola','rep105'],required=True);p.add_argument('--output',required=True);p.add_argument('--require-nav',action='store_true');a=p.parse_args()
rclpy.init();node=Node('msc_architecture_probe');tf=Buffer();listener=TransformListener(tf,node)
edges={};owners={};maps={};state={};commands=[];velocity=[];poses=[];quality=[];quality_by_topic={};diagnostics={}

def diagnostic(msg):
 for status in msg.status:
  if 'LidarOdometry' in status.name:
   diagnostics[status.name]={'level':int.from_bytes(status.level,'little') if isinstance(status.level,bytes) else int(status.level),'message':status.message,'values':{v.key:v.value for v in status.values}}
def transforms(msg,info):
 for t in msg.transforms:
  edge=t.header.frame_id+'->'+t.child_frame_id;edges[edge]=edges.get(edge,0)+1
  gid=info.get('publisher_gid') if isinstance(info,dict) else getattr(info,'publisher_gid',None)
  owners.setdefault(edge,set()).add(bytes(gid).hex() if gid is not None else 'unavailable')
def grid(name,msg):
 frame=msg.header.frame_id;result=encode_grid(msg);result.pop('cdr_zlib');result['original_frame']=frame;maps[name]=result
tf_sub=node.create_subscription(TFMessage,'/tf',lambda m:None,100)
class ProbeExecutor(SingleThreadedExecutor):
 def _take_subscription(self,sub):
  if sub is tf_sub:
   with sub.handle:return sub.handle.take_message(sub.msg_type,sub.raw)
  return super()._take_subscription(sub)
 async def _execute_subscription(self,sub,msg):
  if sub is tf_sub:
   if msg:transforms(*msg)
   return
  await super()._execute_subscription(sub,msg)
executor=ProbeExecutor();executor.add_node(node)
for topic in ('/map','/fleet/map','/local_costmap/costmap','/global_costmap/costmap'):
 node.create_subscription(OccupancyGrid,topic,lambda m,t=topic:grid(t,m),QOS)
node.create_subscription(String,'/msc/local_status',lambda m:state.update(json.loads(m.data)),10)
node.create_subscription(Twist,'/motor_command_sent',lambda m:commands.append([m.linear.x,m.angular.z]),100)
node.create_subscription(Odometry,'/odom',lambda m:velocity.append([m.twist.twist.linear.x,m.twist.twist.angular.z]),100)
node.create_subscription(DiagnosticArray,'/diagnostics',diagnostic,10)
deadline=time.monotonic()+10;quality_subscribed=False
while time.monotonic()<deadline:
 executor.spin_once(timeout_sec=.05)
 if not quality_subscribed:
  for topic,types in node.get_topic_names_and_types():
   if 'quality' in topic and types[0] in ('std_msgs/msg/Float32','std_msgs/msg/Float64'):
    def received_quality(m,t=topic):quality.append(m.data);quality_by_topic.setdefault(t,[]).append(m.data)
    node.create_subscription(Float32 if types[0].endswith('Float32') else Float64,topic,received_quality,10)
    quality_subscribed=True
 try:
  t=tf.lookup_transform('map','base_link',Time());v=t.transform;q=v.rotation
  poses.append([v.translation.x,v.translation.y,math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))])
 except Exception:pass
def call(kind,topic,request):
 c=node.create_client(kind,topic);assert c.wait_for_service(timeout_sec=5),topic
 f=c.call_async(request);executor.spin_until_future_complete(f,timeout_sec=5);assert f.done(),topic
 return f.result()
req=GetParameters.Request();req.names=['publish_odom_tf'];driver_tf=call(GetParameters,'/omnifleet_t2_driver/get_parameters',req).values[0].bool_value
runtime=yaml.safe_load(call(MolaRuntimeParamGet,'/mola_runtime_param_get',MolaRuntimeParamGet.Request()).parameters)
environments=[]
for process in Path('/proc').glob('[0-9]*'):
 try:
  argv=(process/'cmdline').read_bytes().split(b'\0')
  if not argv or Path(argv[0].decode()).name!='mola-cli':continue
  env=dict(pair.split('=',1) for pair in (process/'environ').read_bytes().decode().split('\0') if '=' in pair)
  environments.append({k:env.get(k) for k in ('MOLA_LOAD_MM','MOLA_MAPPING_ENABLED','MOLA_LOCALIZ_USE_REP105','MOLA_START_ACTIVE','MOLA_ROS2_PUBLISH_IN_SIM_TIME')})
 except OSError:pass
assert len(environments)==1,environments
assert environments[0]['MOLA_LOAD_MM'].strip('"')=='/home/iecme/maps/foxglove_map.mm'
assert str(environments[0]['MOLA_MAPPING_ENABLED']).lower()=='false'
assert driver_tf==(a.mode=='rep105')
required=['map->base_link'] if a.mode=='pure_mola' else ['map->odom','odom->base_link']
for edge in required:assert edges.get(edge,0)>5 and len(owners[edge])==1,(edge,edges,owners)
for edge in (['map->odom','odom->base_link'] if a.mode=='pure_mola' else ['map->base_link']):assert edges.get(edge,0)==0,(edge,edges)
assert poses and commands and velocity and '/map' in maps
assert all(v==0 and w==0 for v,w in commands) and all(abs(v)<.02 and abs(w)<.05 for v,w in velocity)
if a.require_nav:assert state.get('nav_ready') and state.get('shared_map_ready') and '/global_costmap/costmap' in maps,state.get('health')
age=(node.get_clock().now().nanoseconds-Time.from_msg(tf.lookup_transform('map','base_link',Time()).header.stamp).nanoseconds)/1e9
assert -.5<=age<1.,age
report={'mode':a.mode,'robot':state.get('robot_id'),'map_file_sha256':hashlib.sha256(Path('/home/iecme/maps/foxglove_map.mm').read_bytes()).hexdigest(),
 'mola_environment':environments[0],'runtime_parameters':runtime,'driver_publish_odom_tf':driver_tf,'tf_edges':edges,
 'tf_owner_count':{k:len(v) if 'unavailable' not in v else None for k,v in owners.items()},
 'tf_publisher_nodes':[i.node_name for i in node.get_publishers_info_by_topic('/tf')],
 'map_base_tf_age':age,'maps':maps,'pose':poses[-1],
 'pose_translation_span':max(math.dist(p[:2],poses[0][:2]) for p in poses),'quality_min':min(quality) if quality else None,
 'quality':{k:{'samples':len(v),'min':min(v),'max':max(v),'mean':sum(v)/len(v),'last':v[-1],'zeros':sum(x==0 for x in v)} for k,v in quality_by_topic.items()},
 'diagnostics':diagnostics,'nav_ready':state.get('nav_ready'),'shared_map_ready':state.get('shared_map_ready'),'motor_samples':len(commands),'nonzero_motor_commands':0}
Path(a.output).write_text(json.dumps(report,indent=2));print(json.dumps(report));node.destroy_node();rclpy.try_shutdown()
