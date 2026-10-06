"""Isolated local safety-gate regression; no chassis or serial hardware is opened."""
import os
import json
import threading
import time
import tempfile
from types import SimpleNamespace

assert os.environ.get('ROS_DOMAIN_ID') == '179' and os.environ.get('ROS_LOCALHOST_ONLY') == '1'
os.environ['OMNIFLEET_ROBOT_ID'] = 'isolated'

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import Bool, String

from omnifleet_local_navigation.control import LocalControl
from omnifleet_local_navigation.task_policy import Ticket


class IsolatedLocalControl(LocalControl):
    def __init__(self, config):
        self.policy = SimpleNamespace(local_override=False, canceling=False)
        self.fleet_enabled = True
        self.inhibited = False
        self.armed = True
        self.current_job = None
        self.navigation_active = False
        self.fleet_permission_epoch = 0
        self.stop_calls = []
        self.execution = SimpleNamespace(active=False, stop=self.stop_calls.append)
        super().__init__(config)

    def output_permitted(self):
        return self.armed and not self.inhibited


rclpy.init()
driver = Node('omnifleet_t2_driver')
nav = Node('controller_server')
manual = Node('operator_test')
received = []


def observe_safe_command(msg):
    received.append((msg.linear.x, msg.angular.z))


driver.create_subscription(Twist, '/isolated/msc/cmd_vel_safe', observe_safe_command, 20)
odom = nav.create_publisher(Odometry, '/isolated/odom', 10)
nav_pub = nav.create_publisher(Twist, '/isolated/msc/nav_cmd_vel', 10)
manual_pub = manual.create_publisher(Twist, '/isolated/cmd_vel', 10)
stop_pub = manual.create_publisher(Bool, '/isolated/msc/estop', 10)
temporary = tempfile.TemporaryDirectory()
control = IsolatedLocalControl({
    'robot_id': 'isolated', 'enable_control': True, 'length': .5, 'width': .37,
    'padding': .05, 'radius': .37, 'safety_clearance': .2,
    'state_file': temporary.name + '/state.json',
})
executor = MultiThreadedExecutor(num_threads=5)
for node in (driver, nav, manual, control):
    executor.add_node(node)
worker = threading.Thread(target=executor.spin, daemon=True)
worker.start()


def spin(seconds, nav_command=None, manual_command=None):
    end = time.monotonic() + seconds
    while time.monotonic() < end:
        odom.publish(Odometry())
        if nav_command is not None:
            msg = Twist(); msg.linear.x = nav_command; nav_pub.publish(msg)
        if manual_command is not None:
            msg = Twist(); msg.linear.x = manual_command; manual_pub.publish(msg)
        time.sleep(.02)


def last_command():
    assert received, 'no safe-output command received'
    return received[-1]


try:
    for _ in range(40):
        spin(.1)
        if control.control_gate_ready and control.startup_done:
            break
    assert control.control_gate_ready and control.startup_done, getattr(control, 'last_gate_state', None)
    assert len(control.get_publishers_info_by_topic('/isolated/msc/cmd_vel_safe')) == 1

    spin(.2, nav_command=.2)
    assert last_command() == (.2, 0.)
    spin(.2, nav_command=.2, manual_command=.1)
    assert last_command() == (.1, 0.), 'manual command must take priority over Nav2'
    spin(.5, nav_command=.2)
    assert last_command() == (0., 0.), 'manual input timeout must stop the output'

    status = {
        'robot_id': 'isolated', 'agent_boot': 'agent-1', 'agent_sequence': 10,
        'coordinator_age': .05, 'control_gate_ready': True, 'local_ready': True,
        'allow_fleet_motion': True, 'fleet_enabled': True, 'fleet_hold': False,
        'estop': False, 'manual_active': False, 'local_override': False,
        'control_mode': 'FLEET', 'command_kind': 'navigate', 'command_task_id': 'task-1',
        'control_epoch': 'epoch-1', 'applied_command_seq': 1,
        'fleet_pose': [0., 0., 0.], 'pose_age': .02, 'safety_clearance': .2,
        'peers': [{'pose': [3., 0., 0.], 'velocity': [0., 0.], 'radius': .37, 'age': .02}],
    }
    control.on_msc_status(String(data=json.dumps(status)))
    control.mode = 'FLEET'; control.fleet_hold = False; control.inhibited = False; control.armed = True
    control.policy.local_override = False; control.fleet_enabled = True
    control.nav = [.2, 0.]; control.nav_time = time.monotonic()
    control.local_pose = [0., 0., 0.]; control.pose_age = .02
    control.current_job = SimpleNamespace(source='FLEET', ticket=Ticket('task-1', 'FLEET', 1, 'epoch-1'))
    control.execution.active = True
    control.control_tick()
    spin(.1)
    assert last_command() == (.2, 0.)

    status['peers'] = [{'pose': [.7, 0., 0.], 'velocity': [0., 0.], 'radius': .37, 'age': .02}]
    control.on_msc_status(String(data=json.dumps(status)))
    control.control_tick()
    spin(.1)
    assert last_command() == (0., 0.), 'peer collision check must stop the fleet command'

    status.update({'agent_boot': 'agent-2', 'agent_sequence': 0, 'control_epoch': 'epoch-2',
                   'applied_command_seq': 0, 'command_epoch': 'epoch-2',
                   'command_task_id': 'task-2'})
    control.current_job = SimpleNamespace(source='FLEET', ticket=Ticket('task-2', 'FLEET', 0, 'epoch-2'))
    control.fleet_enabled = True; control.fleet_hold = False; control.mode = 'FLEET'
    stopped_before_restart = len(control.stop_calls)
    control.on_msc_status(String(data=json.dumps(status)))
    assert not control.fleet_enabled and control.fleet_hold
    assert len(control.stop_calls) > stopped_before_restart, 'agent restart must stop an active fleet route'

    status['peers'] = []
    control.on_msc_status(String(data=json.dumps(status)))
    control.agent_status_time = time.monotonic() - 1.
    control.control_tick()
    spin(.1)
    assert last_command() == (0., 0.) and control.fleet_hold, 'expired agent lease must stop FLEET output'

    stop_pub.publish(Bool(data=True));spin(.1)
    assert control.estop
    print('PASS: sole safe output, manual priority, peer collision stop, expired fleet lease, estop latch')
finally:
    executor.shutdown(); worker.join(timeout=2)
    for node in (control, driver, nav, manual):
        node.destroy_node()
    temporary.cleanup()
    rclpy.try_shutdown()
