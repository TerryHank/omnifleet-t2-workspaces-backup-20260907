"""Isolated MSC-to-local-navigation action regression; no hardware node is used."""
import os
import json
import tempfile
import threading
import time

assert os.environ.get('ROS_DOMAIN_ID') == '179' and os.environ.get('ROS_LOCALHOST_ONLY') == '1'
os.environ['OMNIFLEET_ROBOT_ID'] = 'action_test'

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from omnifleet_navigation_interfaces.action import ExecuteNavigation
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import String

from omnifleet_msc.agent import Agent


rclpy.init()
driver = Node('omnifleet_t2_driver')
local_navigation = Node('local_navigation_action_test')
action_node = Node('local_navigation_action_server')
observed_commands = []
received_goals = []
cancelled_goals = []
goal_delay = [0.45]
echo_command = [True]
odom = local_navigation.create_publisher(Odometry, '/odom', 10)
safe_output = local_navigation.create_publisher(Twist, '/msc/cmd_vel_safe', 10)


def observe_safe_command(msg):
    observed_commands.append((msg.linear.x, msg.angular.z))


driver.create_subscription(Twist, '/msc/cmd_vel_safe', observe_safe_command, 20)
local_status_pub = local_navigation.create_publisher(String, '/action_test/navigation/local_status', 10)
catalog_pub = local_navigation.create_publisher(String, '/action_test/navigation/status', 10)


def accept_goal(_request):
    time.sleep(goal_delay[0])
    return GoalResponse.ACCEPT


def execute_goal(handle):
    received_goals.append(handle.request)
    feedback = ExecuteNavigation.Feedback()
    feedback.task_id = handle.request.task_id
    feedback.phase = 'tracking'
    feedback.message = 'test route active'
    feedback.passed_points = 0
    feedback.tf_age_ms = 12.0
    handle.publish_feedback(feedback)
    for _ in range(100):
        if handle.is_cancel_requested:
            handle.canceled()
            cancelled_goals.append(handle.request)
            result = ExecuteNavigation.Result()
            result.success = False
            result.code = 'CANCELED'
            result.message = 'test cancellation acknowledged'
            return result
        time.sleep(0.02)
    handle.succeed()
    result = ExecuteNavigation.Result()
    result.success = True
    result.code = 'SUCCEEDED'
    result.message = 'test goal completed'
    return result


action_server = ActionServer(
    action_node,
    ExecuteNavigation,
    '/action_test/navigation/execute',
    execute_goal,
    goal_callback=accept_goal,
    cancel_callback=lambda _handle: CancelResponse.ACCEPT,
    callback_group=ReentrantCallbackGroup(),
)
temporary = tempfile.TemporaryDirectory()
agent = Agent({
    'robot_id': 'action_test',
    'coordinator': 'http://127.0.0.1:1',
    'key': 'test',
    'enable_control': True,
    'allow_fleet_motion': True,
    'localization_source': 'odom',
    'length': .5,
    'width': .37,
    'padding': .05,
    'radius': .39,
    'state_file': temporary.name + '/state.json',
})
agent.goal_safe = lambda _target: True
executor = MultiThreadedExecutor(num_threads=6)
for node in (driver, local_navigation, action_node, agent):
    executor.add_node(node)
thread = threading.Thread(target=executor.spin, daemon=True)
thread.start()


def publish_status():
    local_status_pub.publish(String(data='''{"robot_id":"action_test","nav_ready":true,"control_gate_ready":true,
      "velocity":[0.0,0.0],"stopped_seconds":2.0,"nav_active":false,"pending_goal":false,
      "estop":false,"estop_input":false,"contact":false,"pose_stationary":true,"pose_age":0.02}'''))
    catalog = {'robot_id': 'action_test', 'local_ready': True, 'fleet_enabled': True,
               'local_override': False, 'manual_active': False, 'control_mode': 'LOCAL',
               'agent_boot': agent.boot, 'control_epoch': '', 'applied_command_seq': 0,
               'agent_command_kind': 'observe', 'agent_command_task_id': '',
               'agent_fleet_hold': True, 'agent_estop': False,
               'agent_status_age': .01, 'coordinator_age': .05}
    if echo_command[0]:
        catalog.update(control_epoch=agent.command_epoch,
                       applied_command_seq=agent.applied_seq,
                       agent_command_kind=agent.command_kind,
                       agent_command_task_id=agent.command_task_id,
                       agent_fleet_hold=agent.fleet_hold)
    catalog_pub.publish(String(data=json.dumps(catalog)))
    odom.publish(Odometry())
    safe_output.publish(Twist())
    with agent.lock:
        agent.last_response = time.monotonic()
        agent.response = {'peers': [], 'alignment': None, 'safety_clearance': .2,
                          'command': {'seq': agent.applied_seq, 'epoch': agent.command_epoch,
                                      'kind': agent.command_kind}}


def spin(seconds):
    end = time.monotonic() + seconds
    while time.monotonic() < end:
        publish_status()
        time.sleep(.02)


try:
    spin(2.0)
    assert agent.startup_done and agent.control_gate_ready
    assert agent.client.server_is_ready()
    publishers = agent.get_publishers_info_by_topic('/msc/cmd_vel_safe')
    assert len(publishers) == 1 and publishers[0].node_name == 'local_navigation_action_test'

    echo_command[0] = False
    agent.apply({'seq': 1, 'epoch': 'epoch-1', 'kind': 'navigate', 'task_id': 'task-1', 'target': [1., 0., 0.]})
    spin(.15)
    assert agent.pending_dispatch and not received_goals, 'agent must wait for local command-lease acknowledgement'
    echo_command[0] = True
    spin(.3)
    assert agent.pending_goal_requests, 'FLEET command must be sent through ExecuteNavigation'
    agent.apply({'seq': 2, 'epoch': 'epoch-1', 'kind': 'hold', 'task_id': 'task-1'})
    spin(1.6)
    assert cancelled_goals, 'late action acceptance must be canceled after the coordinator hold'
    assert not agent.pending_goal_requests and agent.cancel_pending == 0
    assert agent.completed_seq == 0, 'a canceled late acceptance must not count as completion'

    goal_delay[0] = 0.
    agent.apply({'seq': 3, 'epoch': 'epoch-1', 'kind': 'navigate', 'task_id': 'task-2', 'target': [2., 0., 0.]})
    spin(2.5)
    assert received_goals and received_goals[-1].source == ExecuteNavigation.Goal.FLEET
    assert received_goals[-1].task_id == 'task-2'
    assert received_goals[-1].command_epoch == 'epoch-1' and received_goals[-1].revision == 3
    assert agent.navigation_feedback == {
        'task_id': 'task-2', 'command_epoch': 'epoch-1', 'revision': 3,
        'phase': 'tracking', 'message': 'test route active', 'passed_points': 0,
        'tf_age_ms': 12.0,
    }
    assert agent.own_snapshot.get('navigation_feedback') == agent.navigation_feedback
    assert agent.completed_seq == 3
    assert all(v == 0. and w == 0. for v, w in observed_commands), 'MSC agent must not publish chassis velocity'
    assert agent.count_publishers('/msc/peer_map') == 1, 'MSC agent is the sole peer-map owner in the isolated graph'
    print('PASS: FLEET goals use ExecuteNavigation; hold cancels late acceptance; MSC agent emits no Twist output')
finally:
    agent.closed = True
    agent.network.join(timeout=2)
    executor.shutdown()
    thread.join(timeout=2)
    action_server.destroy()
    for node in (agent, action_node, local_navigation, driver):
        node.destroy_node()
    temporary.cleanup()
    rclpy.try_shutdown()
