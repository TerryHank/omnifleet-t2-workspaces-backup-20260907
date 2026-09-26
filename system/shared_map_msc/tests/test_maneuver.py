import unittest
import test_core
from omnifleet_msc.planning import FleetGrid
from omnifleet_msc.core import Coordinator
class ManeuverTests(unittest.TestCase):
 def setUp(self):
  self.f=test_core.FleetTests();self.f.setUp();self.f.execute();self.c=self.f.core
  self.c.grid=FleetGrid(150,100,.1,[-5,-5,0],[0]*15000)
 def tearDown(self):self.f.tearDown()
 def begin(self):
  states={r:self.c.robot(r) for r in self.c.members()}
  self.assertTrue(self.c.begin_passage(states,[1,0,0],'test conflict'))
  self.assertEqual(self.c.mission['state'],'RECOVERING')
 def test_passage_rejoins_without_repeating_leader_waypoint(self):
  self.begin()
  for _ in range(35):
   for robot,command in self.c.commands.items():
    if command['kind']=='navigate':
     self.f.states[robot]['local_pose']=command['target'];self.f.states[robot]['completed_seq']=command['seq']
   self.f.feed(.2)
   if self.c.mission['current_waypoint']==1:break
  self.assertEqual(self.c.mission['current_waypoint'],1)
  self.assertEqual(self.c.mission['completed_waypoints'],[0])
  self.assertNotIn('maneuver',self.c.mission)
 def test_cancel_interrupts_passage(self):
  self.begin();self.c.operator({'op':'cancel'});self.f.feed(.8)
  self.assertEqual(self.c.mission['state'],'FAILED')
  self.assertTrue(all(c['kind']!='navigate' for c in self.c.commands.values()))
 def test_restart_keeps_completed_waypoints_but_does_not_resume(self):
  self.c.mission['completed_waypoints']=[0];self.c.mission['current_waypoint']=1;self.c.persist()
  restored=Coordinator(self.c.config,self.c.path,lambda:self.f.now)
  self.assertEqual(restored.mission['current_waypoint'],1);self.assertEqual(restored.mission['state'],'DEGRADED')
  self.assertTrue(all(c['kind']=='hold' for c in restored.commands.values()))
