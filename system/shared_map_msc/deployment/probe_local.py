import json,time
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist
rclpy.init();n=Node('msc_readonly_probe');state=[];commands=[]
n.create_subscription(String,'/msc/local_status',lambda m:state.append(json.loads(m.data)),10)
n.create_subscription(Twist,'/motor_command_sent',lambda m:commands.append([m.linear.x,m.angular.z]),20)
start=time.monotonic()
while time.monotonic()-start<3:rclpy.spin_once(n,timeout_sec=.1)
assert state and commands
print(json.dumps({'state':state[-1],'command_samples':len(commands),'nonzero_commands':sum(v!=0 or w!=0 for v,w in commands)},ensure_ascii=False))
n.destroy_node();rclpy.try_shutdown()
