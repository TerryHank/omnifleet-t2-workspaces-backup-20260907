import json,time,uuid
from pathlib import Path
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from rclpy.qos import QoSProfile,DurabilityPolicy
rclpy.init();node=Node('msc_fleet_diagnostic_probe');id_='fleet-test-'+uuid.uuid4().hex;answers=[]
def received(msg):
 data=json.loads(msg.data)
 if data.get('request_id')==id_:answers.append(data)
node.create_subscription(String,'/omnifleet_t2/diagnostics/answer',received,QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
pub=node.create_publisher(String,'/omnifleet_t2/diagnostics/question',10)
deadline=time.monotonic()+5
while pub.get_subscription_count()==0 and time.monotonic()<deadline:rclpy.spin_once(node,timeout_sec=.1)
assert pub.get_subscription_count()>0
pub.publish(String(data=json.dumps({'request_id':id_,'question':'仅做静态核对：104和113是否都以113的地图作为公共导航底图？请分别核对两车底图来源、全局坐标、分辨率和导航就绪状态，列出证据没有覆盖的项。不要建议解除运动锁或发送导航目标。'},ensure_ascii=False)))
deadline=time.monotonic()+130
while time.monotonic()<deadline and not any(a.get('status') in ('complete','error') for a in answers):rclpy.spin_once(node,timeout_sec=.2)
Path('/home/iecme/robot_backups/msc_shared_map_dsh_20260909/fleet-diagnostic.json').write_text(json.dumps(answers,indent=2,ensure_ascii=False))
assert answers and answers[-1]['status']=='complete',answers[-1] if answers else 'no response'
print(json.dumps(answers[-1],ensure_ascii=False));node.destroy_node();rclpy.try_shutdown()
