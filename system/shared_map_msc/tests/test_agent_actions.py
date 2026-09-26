"""Isolated action race regression. No hardware node or serial connection is used."""
import os,time,tempfile,threading
assert os.environ.get('ROS_DOMAIN_ID')=='179' and os.environ.get('ROS_LOCALHOST_ONLY')=='1'
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer,GoalResponse,CancelResponse
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.task import Future
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from omnifleet_msc.agent import Agent
rclpy.init();driver=Node('omnifleet_t2_driver');server_node=Node('bt_navigator');received=[];goals=[];cancelled=[]
driver.create_subscription(Twist,'/msc/cmd_vel_safe',lambda m:received.append([m.linear.x,m.angular.z]),50)
odom=driver.create_publisher(Odometry,'/odom',10);delay=[.6]
def accept(request):time.sleep(delay[0]);return GoalResponse.ACCEPT
def execute(handle):
 goals.append(handle)
 for _ in range(60):
  if handle.is_cancel_requested:
   handle.canceled();cancelled.append(handle);return NavigateToPose.Result()
  time.sleep(.02)
 handle.succeed();return NavigateToPose.Result()
server=ActionServer(server_node,NavigateToPose,'/navigate_to_pose',execute,goal_callback=accept,
 cancel_callback=lambda _:CancelResponse.ACCEPT,callback_group=ReentrantCallbackGroup())
temporary=tempfile.TemporaryDirectory()
agent=Agent({'robot_id':'action_test','coordinator':'http://127.0.0.1:1','key':'test','enable_control':True,
 'allow_fleet_motion':True,'length':.5,'width':.37,'padding':.05,'radius':.39,'state_file':temporary.name+'/state.json'})
agent.goal_safe=lambda _:True
executor=MultiThreadedExecutor(num_threads=6)
for node in (driver,server_node,agent):executor.add_node(node)
thread=threading.Thread(target=executor.spin,daemon=True);thread.start()
def spin(seconds):
 end=time.monotonic()+seconds
 while time.monotonic()<end:
  odom.publish(Odometry())
  with agent.lock:agent.last_response=time.monotonic()
  time.sleep(.02)
try:
 spin(2);assert agent.startup_done and agent.control_gate_ready
 stale=Future();agent.pending_states.add('planner_server')
 agent.pending_state_calls['planner_server']=(agent.state_clients['planner_server'],stale,time.monotonic()-5)
 agent.discovery();assert 'planner_server' not in agent.pending_states,'stale lifecycle request must be removed'
 agent.config['require_shared_map']=True;agent.mode='LOCAL'
 agent.nav=[.2,0.];agent.nav_time=time.monotonic();agent.control_tick()
 assert agent.safety_reason=='shared map unavailable for local navigation'
 agent.config['require_shared_map']=False
 agent.apply({'seq':1,'epoch':'test','kind':'navigate','target':[1,0,0]})
 assert agent.pending_goal_requests
 agent.cancel_all();assert agent.pending_goal_requests,'cancel cannot erase an unacknowledged goal request'
 spin(1.5);assert cancelled and not agent.pending_goal_requests and agent.cancel_pending==0
 assert agent.completed_seq==0,'late canceled result must not count as success'
 delay[0]=0
 agent.apply({'seq':2,'epoch':'test','kind':'navigate','target':[2,0,0]})
 spin(1.8);assert agent.completed_seq==2
 assert all(v==0 and w==0 for v,w in received)
 print('PASS late acceptance is canceled, pending requests retained, fresh goal completes, zero physical output')
finally:
 agent.closed=True;agent.network.join(timeout=2);executor.shutdown();thread.join(timeout=2)
 server.destroy()
 for node in (agent,server_node,driver):node.destroy_node()
 temporary.cleanup();rclpy.try_shutdown()
