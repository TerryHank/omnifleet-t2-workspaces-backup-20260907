"""Read-only final evidence. Never publishes goals or velocity commands."""
import json,time,os
from pathlib import Path
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from tf2_ros import Buffer,TransformListener

rclpy.init();node=Node('msc_final_static_probe');tf=Buffer();listener=TransformListener(tf,node)
robot=os.environ.get('OMNIFLEET_ROBOT_ID','robot_113');robot_ns='/'+robot
safe_topic=robot_ns+'/msc/cmd_vel_safe';peer_topic=robot_ns+'/msc/peer_map'
agent_status_topic=robot_ns+'/msc/local_status';navigation_status_topic=robot_ns+'/navigation/local_status'
states=[];commands=[];fleet=[]
node.create_subscription(String,agent_status_topic,lambda m:states.append(json.loads(m.data)),20)
node.create_subscription(Twist,robot_ns+'/motor_command_sent',lambda m:commands.append([m.linear.x,m.angular.z]),100)
node.create_subscription(String,'/fleet/status',lambda m:fleet.append(json.loads(m.data)),20)
end=time.monotonic()+8
while time.monotonic()<end:rclpy.spin_once(node,timeout_sec=.05)
assert states and commands,'missing state or driver feedback'
state=states[-1]
assert all(v==0 and w==0 for v,w in commands),'nonzero motor command observed'
assert state['velocity']==[0.,0.] and state['control_gate_ready'] and state['nav_ready']
assert not state['nav_active'] and not state['pending_goal']
safe_publishers=node.get_publishers_info_by_topic(safe_topic)
peer_publishers=node.get_publishers_info_by_topic(peer_topic)
status_publishers=node.get_publishers_info_by_topic(agent_status_topic)
navigation_status_publishers=node.get_publishers_info_by_topic(navigation_status_topic)
assert len(safe_publishers)==1 and safe_publishers[0].node_name=='local_navigation_'+robot
assert len(peer_publishers)==1 and peer_publishers[0].node_name=='msc_agent_'+robot
assert len(status_publishers)==1 and status_publishers[0].node_name=='msc_agent_'+robot
assert len(navigation_status_publishers)==1 and navigation_status_publishers[0].node_name=='local_navigation_'+robot
assert node.count_subscribers(safe_topic)==1
graph={topic:{'publishers':[i.node_name for i in node.get_publishers_info_by_topic(topic)],
              'subscribers':[i.node_name for i in node.get_subscriptions_info_by_topic(topic)]}
       for topic in (robot_ns+'/cmd_vel',robot_ns+'/msc/nav_cmd_vel',safe_topic)}
assert 'omnifleet_t2_driver' in graph[safe_topic]['subscribers']
assert 'omnifleet_t2_driver' not in graph[robot_ns+'/cmd_vel']['subscribers']
assert 'controller_server' in graph[robot_ns+'/msc/nav_cmd_vel']['publishers']
result={'robot':state['robot_id'],'domain':os.environ.get('ROS_DOMAIN_ID'),'rmw':os.environ.get('RMW_IMPLEMENTATION'),
        'state':state,'command_samples':len(commands),'nonzero_commands':0,'graph':graph,
        'publisher_owners':{safe_topic:[p.node_name for p in safe_publishers],
                            peer_topic:[p.node_name for p in peer_publishers],
                            agent_status_topic:[p.node_name for p in status_publishers],
                            navigation_status_topic:[p.node_name for p in navigation_status_publishers]}}
if fleet:
 s=fleet[-1];robots=s.get('robots',[])
 if isinstance(robots,list):
  robot_ids=[r['robot_id'] for r in robots]
  by_id={r['robot_id']:r for r in robots}
  assert robot in by_id and by_id[robot]['online'] and by_id[robot]['ready']
  assert not any(r.get('navigation_active') for r in robots)
  assert not any(r.get('fleet_enabled') for r in robots)
  result['fleet_robot_status']={r['robot_id']:{k:r.get(k) for k in ('online','ready','fleet_enabled','navigation_active','heartbeat_age_s')} for r in robots}
 else:
  robot_ids=list(robots)
  assert not s.get('motion_enabled',False)
  assert robot in robots and not any(r.get('navigation_active') for r in robots.values())
  result['fleet_robot_status']={r:{k:v.get(k) for k in ('online','fleet_ready','fleet_enabled','navigation_active','pose_stationary','heartbeat_age')} for r,v in robots.items()}
 required=['/fleet/'+n for n in ('status','events','mission_state','formation_state')]
 required += ['/'+r+'/'+n for r in robot_ids for n in ('heartbeat','health','mission_state','formation_error')]
 result['fleet_topic_publishers']={t:node.count_publishers(t) for t in required}
 assert node.count_publishers('/'+robot+'/heartbeat')==1
 result['fleet_topic_owner_warnings']={t:v for t,v in result['fleet_topic_publishers'].items() if v!=1}
 if not isinstance(robots,list):
  result['fleet_tf']={}
  for robot_id,v in robots.items():
   if not v.get('online') or v.get('fleet_pose') is None:continue
   t=tf.lookup_transform('fleet_map',robot_id+'/fleet_pose',rclpy.time.Time())
   result['fleet_tf'][robot_id]=[t.transform.translation.x,t.transform.translation.y]
 result['fleet']=s
root=Path('/home/iecme/robot_backups/msc_v1_20260909/evidence')
(root/'final-static-probe.json').write_text(json.dumps(result,ensure_ascii=False,indent=2))
print(json.dumps({k:v for k,v in result.items() if k not in ('state','fleet')},ensure_ascii=False))
node.destroy_node();rclpy.try_shutdown()
