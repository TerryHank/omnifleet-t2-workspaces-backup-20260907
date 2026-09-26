import json,time
from pathlib import Path
import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
rclpy.init();n=Node('msc_observer_verify');client=n.create_client(Trigger,'/omnifleet_t2/nav2/read_saved')
end=time.monotonic()+5
while time.monotonic()<end:rclpy.spin_once(n,timeout_sec=.1)
topics=('/msc/nav_cmd_vel','/msc/cmd_vel_safe','/motor_command_sent')
subscriptions={t:[i.node_name for i in n.get_subscriptions_info_by_topic(t)] for t in topics}
for topic in topics:assert 'omnifleet_t2_dsh_diagnostics' in subscriptions[topic],subscriptions
for topic in ('/msc/nav_cmd_vel','/motor_command_sent'):
 assert 'omnifleet_t2_nav2_parameter_store' in subscriptions[topic],subscriptions
assert client.wait_for_service(timeout_sec=2)
future=client.call_async(Trigger.Request());rclpy.spin_until_future_complete(n,future,timeout_sec=10)
assert future.done() and future.result().success
result={'subscriptions':subscriptions,'read_saved_success':True}
Path('/home/iecme/robot_backups/msc_v1_20260909/evidence/observers.json').write_text(json.dumps(result,indent=2))
print(json.dumps(result));n.destroy_node();rclpy.try_shutdown()
