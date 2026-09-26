"""Native ROS isolation test. Fake driver records output; no serial hardware is opened."""
import os,time,threading,tempfile
import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool
from action_msgs.msg import GoalStatusArray,GoalStatus
from rclpy.qos import QoSProfile,DurabilityPolicy
from omnifleet_msc.agent import Agent

assert os.environ.get('ROS_DOMAIN_ID')=='179' and os.environ.get('ROS_LOCALHOST_ONLY')=='1'
rclpy.init();driver=Node('omnifleet_t2_driver');nav=Node('controller_server');manual=Node('operator_test')
received=[];driver.create_subscription(Twist,'/msc/cmd_vel_safe',lambda m:received.append([m.linear.x,m.angular.z]),20)
odom=driver.create_publisher(Odometry,'/odom',10);nav_pub=nav.create_publisher(Twist,'/msc/nav_cmd_vel',10)
manual_pub=manual.create_publisher(Twist,'/cmd_vel',10);stop_pub=manual.create_publisher(Bool,'/msc/estop',10)
status_pub=nav.create_publisher(GoalStatusArray,'/navigate_to_pose/_action/status',QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
temporary=tempfile.TemporaryDirectory()
agent=Agent({'robot_id':'isolated','coordinator':'http://127.0.0.1:1','key':'test','enable_control':True,'length':.5,'width':.37,'padding':.05,'radius':.37,'state_file':temporary.name+'/state.json'})
executor=MultiThreadedExecutor(num_threads=2)
for n in (driver,nav,manual,agent):executor.add_node(n)
worker=threading.Thread(target=executor.spin,daemon=True);worker.start()
def spin(seconds,command=None,manual_command=None,goal_id=None):
 end=time.monotonic()+seconds
 while time.monotonic()<end:
  odom.publish(Odometry())
  if command is not None:
   m=Twist();m.linear.x=command;nav_pub.publish(m)
  if manual_command is not None:
   m=Twist();m.linear.x=manual_command;manual_pub.publish(m)
  if goal_id:
   a=GoalStatusArray();s=GoalStatus();s.status=2;s.goal_info.goal_id.uuid=[goal_id]*16;a.status_list=[s];status_pub.publish(a)
  time.sleep(.02)
try:
 spin(2.);assert agent.control_gate_ready and agent.startup_done
 spin(.4,command=.2,goal_id=1);assert received[-1]==[.2,0.],received[-1]
 spin(.3,command=.2,manual_command=.1,goal_id=1);assert received[-1]==[.1,0.],received[-1]
 spin(.6,command=.2,goal_id=1);assert received[-1]==[0.,0.],received[-1]
 spin(.6,command=.2,goal_id=2);assert received[-1]==[.2,0.],received[-1]
 spin(.2,command=.5,goal_id=2);assert received[-1]==[.5,0.],'fleet cap must not alter ordinary local navigation'
 stop_pub.publish(Bool(data=True));spin(.3,command=.2,goal_id=2);assert received[-1]==[0.,0.]
 stop_pub.publish(Bool(data=False));spin(.6,command=.2,goal_id=2);assert received[-1]==[0.,0.]
 agent.mode='FLEET';agent.fleet_hold=False;agent.last_response=time.monotonic()-2
 spin(.2,command=.2);assert received[-1]==[0.,0.] and agent.fleet_hold
 print('PASS: startup hold, unique output, manual priority, old goal suppression, fresh goal resync, estop latch, lease expiry')
finally:
 agent.closed=True;agent.network.join(timeout=2);executor.shutdown();worker.join(timeout=2)
 for n in (agent,driver,nav,manual):n.destroy_node()
 temporary.cleanup()
 rclpy.shutdown()
