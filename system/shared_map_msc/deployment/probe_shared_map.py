import json,time
from pathlib import Path
import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import String
from tf2_ros import Buffer,TransformListener
from omnifleet_msc.shared_map import QOS,encode_grid
rclpy.init();node=Node('msc_shared_map_probe');buffer=Buffer();listener=TransformListener(buffer,node)
maps={};status={};local={}
def map_sample(topic,msg):
 frame=msg.header.frame_id;maps[topic]=encode_grid(msg);maps[topic]['received_frame']=frame
def update(dest,msg):dest.update(json.loads(msg.data))
for topic in ('/map','/fleet/map','/global_costmap/costmap'):
 node.create_subscription(OccupancyGrid,topic,lambda m,t=topic:map_sample(t,m),QOS)
node.create_subscription(String,'/msc/shared_map_status',lambda m:update(status,m),10)
node.create_subscription(String,'/msc/local_status',lambda m:update(local,m),10)
end=time.monotonic()+7
while time.monotonic()<end:rclpy.spin_once(node,timeout_sec=.05)
assert '/fleet/map' in maps and status.get('alignment_valid'),status
assert local.get('velocity')==[0.,0.] and not local.get('nav_active'),local
t=buffer.lookup_transform('fleet_map','base_link',rclpy.time.Time())
result={'robot':local['robot_id'],'maps':{k:{a:b for a,b in v.items() if a!='cdr_zlib'} for k,v in maps.items()},
 'shared_status':status,'fleet_base_position':[t.transform.translation.x,t.transform.translation.y],
 'shared_map_ready':local.get('shared_map_ready'),'velocity':local['velocity']}
Path('/home/iecme/robot_backups/msc_shared_map_dsh_20260909/shared-map-probe.json').write_text(json.dumps(result,indent=2))
print(json.dumps(result));node.destroy_node();rclpy.try_shutdown()
