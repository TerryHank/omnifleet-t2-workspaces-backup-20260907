import json,time
from pathlib import Path
import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import String
from omnifleet_msc.shared_map import QOS
from omnifleet_msc.protocol import request
rclpy.init();node=Node('msc_peer_layer_probe');maps={};states={}
for topic in ('/msc/peer_map','/global_costmap/costmap'):
 node.create_subscription(OccupancyGrid,topic,lambda m,t=topic:maps.update({t:m}),QOS)
node.create_subscription(String,'/msc/local_status',lambda m:states.update(json.loads(m.data)),10)
end=time.monotonic()+5
while time.monotonic()<end:rclpy.spin_once(node,timeout_sec=.1)
config=json.load(open('/etc/omnifleet_msc/agent.json'))
response=request(config['coordinator'],config['robot_id'],config['key'],'/v1/map',{},timeout=3)
assert response['alignment_valid'] and len(maps)==2
# The sole occupied cluster is the other robot, with no free-cell overwrite elsewhere.
peer=maps['/msc/peer_map'];assert set(peer.data).issubset({-1,100}) and 100 in peer.data
occupied=[i for i,v in enumerate(peer.data) if v==100]
cost=maps['/global_costmap/costmap'];assert cost.header.frame_id=='fleet_map'
assert cost.info.width==peer.info.width and cost.info.height==peer.info.height
overlap=sum(cost.data[i]==100 for i in occupied)
assert overlap>=len(occupied)*.8,(overlap,len(occupied))
assert states['velocity']==[0.,0.] and states['nav_ready']
result={'robot':config['robot_id'],'peer_cells':len(occupied),'lethal_overlap_in_global_costmap':overlap,
 'unmarked_peer_cells_are_unknown':True,'velocity':states['velocity'],'nav_ready':states['nav_ready']}
Path('/home/iecme/robot_backups/msc_shared_map_dsh_20260909/peer-layer.json').write_text(json.dumps(result,indent=2))
print(json.dumps(result));node.destroy_node();rclpy.try_shutdown()
