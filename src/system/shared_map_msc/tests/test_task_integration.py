import unittest
import test_core
from omnifleet_msc.planning import FleetGrid
class TaskIntegrationTests(unittest.TestCase):
 def setUp(self):
  self.fixture=test_core.FleetTests();self.fixture.setUp();self.core=self.fixture.core
  self.core.grid=FleetGrid(100,100,.1,[-5,-5,0],[0]*10000)
  for state in self.fixture.states.values():state['stopped_seconds']=2.
  self.fixture.feed()
 def tearDown(self):self.fixture.tearDown()
 def test_claim_run_confirmed_stop_complete(self):
  self.core.task_operator({'op':'submit','frame_id':'fleet_map','task_id':'one','goal':[2,0,0]})
  self.fixture.feed(.6);task=self.core.task_board.tasks['one'];robot=task['owner']
  self.assertEqual(task['state'],'CLAIMED');self.assertEqual(self.core.commands[robot]['kind'],'claim')
  self.fixture.states[robot]['control_mode']='FLEET';self.fixture.feed(.6)
  self.assertEqual(task['state'],'RUNNING');command=self.core.commands[robot]
  self.fixture.states[robot]['completed_seq']=command['seq'];self.fixture.feed(.6)
  self.assertEqual(task['state'],'FINISHING');self.fixture.feed(.6)
  self.assertEqual(task['state'],'COMPLETED')
 def test_static_mode_only_queues_and_never_claims(self):
  self.core.config['allow_motion']=False
  self.core.task_operator({'op':'submit','frame_id':'fleet_map','task_id':'one','goal':[2,0,0]})
  self.fixture.feed(2)
  self.assertEqual(self.core.task_board.tasks['one']['state'],'PENDING');self.assertFalse(self.core.commands)
 def test_independent_tasks_obey_priority_and_single_moving_owner(self):
  self.core.task_operator({'op':'submit','frame_id':'fleet_map','task_id':'low','goal':[-3,0,0],'priority':1})
  self.core.task_operator({'op':'submit','frame_id':'fleet_map','task_id':'high','goal':[2,0,0],'priority':9})
  self.fixture.feed(.6)
  for state in self.fixture.states.values():state['control_mode']='FLEET'
  self.fixture.feed(.6)
  running=[t for t in self.core.task_board.tasks.values() if t['state']=='RUNNING']
  self.assertEqual([t['task_id'] for t in running],['high'])
  self.assertEqual(sum(c['kind']=='navigate' for c in self.core.commands.values()),1)
 def test_pause_stops_independent_task_until_explicit_resume(self):
  self.core.task_operator({'op':'submit','frame_id':'fleet_map','task_id':'one','goal':[2,0,0]})
  self.fixture.feed(.6);task=self.core.task_board.tasks['one'];robot=task['owner']
  self.fixture.states[robot]['control_mode']='FLEET';self.fixture.feed(.6)
  self.assertEqual(task['state'],'RUNNING')
  self.core.operator({'op':'pause'});self.fixture.feed(1.2)
  self.assertEqual(task['state'],'PENDING');self.assertFalse(any(c['kind']=='navigate' for c in self.core.commands.values()))
  self.core.operator({'op':'resume'});self.fixture.feed(.6)
  self.assertEqual(task['state'],'CLAIMED')
