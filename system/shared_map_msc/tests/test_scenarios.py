import unittest
import test_core
from omnifleet_msc.planning import FleetGrid
class ScenarioTests(unittest.TestCase):
 def setUp(self):self.f=test_core.FleetTests();self.f.setUp();self.f.execute();self.c=self.f.core
 def tearDown(self):self.f.tearDown()
 def test_dynamic_follower_updates_are_rate_limited(self):
  sequences=set()
  for i in range(20):
   for robot,offset in [('robot_113',0),('robot_104',-1.4)]:
    self.f.states[robot]['local_pose']=[i*.08+offset,0,0];self.f.states[robot]['velocity']=[.4,0]
   self.f.feed(.2);sequences.add(self.c.commands['robot_104']['seq'])
  self.assertGreaterEqual(len(sequences),3);self.assertLessEqual(len(sequences),5)
 def test_narrow_corridor_reforms_row_to_column(self):
  cells=[0]*15000
  for y in range(100):
   for x in range(150):
    px=-5+(x+.5)*.1;py=-5+(y+.5)*.1
    if 2<px<5 and abs(py)>.65:cells[y*150+x]=100
  self.c.grid=FleetGrid(150,100,.1,[-5,-5,0],cells)
  self.c.mission['waypoints'][0]=[6,0,0];self.c.mission['formation']='row';self.c.mission['requested_formation']='row'
  self.f.states['robot_104']['local_pose']=[0,1.4,0]
  self.f.feed(.2)
  self.assertEqual(self.c.mission['formation'],'column',self.c.mission);self.assertEqual(self.c.mission['state'],'FORMING')
 def test_dynamic_blocked_goal_stops_team(self):
  self.c.grid=FleetGrid(150,100,.1,[-5,-5,0],[0]*15000)
  self.f.states['robot_113'].update(obstacles=[[1,0]],obstacle_age=0.)
  self.f.feed(.2)
  self.assertEqual(self.c.mission['state'],'DEGRADED');self.assertTrue(all(c['kind']=='hold' for c in self.c.commands.values()))
 def test_follower_lag_waits_and_recovers_without_repeating_waypoint(self):
  self.f.states['robot_104']['local_pose']=[-4,0,0];self.f.feed(.2)
  self.assertEqual(self.c.mission['state'],'RECOVERING');self.assertEqual(self.c.commands['robot_113']['kind'],'hold')
  self.f.states['robot_104']['local_pose']=[-1.4,0,0];self.f.feed(1.2)
  self.assertEqual(self.c.mission['state'],'EXECUTING');self.assertEqual(self.c.mission['current_waypoint'],0)
 def test_report_never_invents_unmeasured_collision_count(self):
  self.c.operator({'op':'cancel'});self.f.feed(.8)
  report=self.c.report();self.assertEqual(report['mission_count'],1)
  self.assertIsNone(report['missions']['one']['metrics']['physical_collision_count'])
