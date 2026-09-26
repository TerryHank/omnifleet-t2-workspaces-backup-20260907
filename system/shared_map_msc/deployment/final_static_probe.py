"""Read-only final evidence. Never publishes goals or velocity commands."""
import json,time,os
from pathlib import Path
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from tf2_ros import Buffer,TransformListener

rclpy.init();node=Node('msc_final_static_probe');tf=Buffer();listener=TransformListener(tf,node)
states=[];commands=[];fleet=[]
node.create_subscription(String,'/msc/local_status',lambda m:states.append(json.loads(m.data)),20)
node.create_subscription(Twist,'/motor_command_sent',lambda m:commands.append([m.linear.x,m.angular.z]),100)
node.create_subscription(String,'/fleet/status',lambda m:fleet.append(json.loads(m.data)),20)
end=time.monotonic()+8
while time.monotonic()<end:rclpy.spin_once(node,timeout_sec=.05)
assert states and commands,'missing state or driver feedback'
state=states[-1]
assert all(v==0 and w==0 for v,w in commands),'nonzero motor command observed'
assert state['velocity']==[0.,0.] and state['control_gate_ready'] and state['nav_ready']
assert not state['nav_active'] and not state['pending_goal']
assert node.count_publishers('/msc/cmd_vel_safe')==1
graph={topic:{'publishers':[i.node_name for i in node.get_publishers_info_by_topic(topic)],
              'subscribers':[i.node_name for i in node.get_subscriptions_info_by_topic(topic)]}
       for topic in ('/cmd_vel','/msc/nav_cmd_vel','/msc/cmd_vel_safe')}
assert 'omnifleet_t2_driver' in graph['/msc/cmd_vel_safe']['subscribers']
assert 'omnifleet_t2_driver' not in graph['/cmd_vel']['subscribers']
assert 'controller_server' in graph['/msc/nav_cmd_vel']['publishers']
result={'robot':state['robot_id'],'domain':os.environ.get('ROS_DOMAIN_ID'),'rmw':os.environ.get('RMW_IMPLEMENTATION'),
        'state':state,'command_samples':len(commands),'nonzero_commands':0,'graph':graph}
if fleet:
 s=fleet[-1];assert s['motion_enabled'] is False
 assert all(r['online'] and r['fleet_pose'] is not None and r['pose_stationary'] for r in s['robots'].values())
 required=['/fleet/'+n for n in ('status','events','mission_state','formation_state')]
 required += ['/'+r+'/'+n for r in s['robots'] for n in ('heartbeat','health','mission_state','formation_error')]
 result['fleet_topic_publishers']={t:node.count_publishers(t) for t in required}
 assert all(v==1 for v in result['fleet_topic_publishers'].values())
 result['fleet_tf']={}
 for robot in s['robots']:
  t=tf.lookup_transform('fleet_map',robot+'/fleet_pose',rclpy.time.Time())
  result['fleet_tf'][robot]=[t.transform.translation.x,t.transform.translation.y]
 result['fleet']=s
root=Path('/home/iecme/robot_backups/msc_v1_20260909/evidence')
(root/'final-static-probe.json').write_text(json.dumps(result,ensure_ascii=False,indent=2))
print(json.dumps({k:v for k,v in result.items() if k not in ('state','fleet')},ensure_ascii=False))
node.destroy_node();rclpy.try_shutdown()
