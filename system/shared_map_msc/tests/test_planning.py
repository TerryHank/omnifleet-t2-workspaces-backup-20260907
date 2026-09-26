import unittest
from omnifleet_msc.planning import FleetGrid,distance_to_path
class PlanningTests(unittest.TestCase):
 def grid(self):return FleetGrid(80,80,.1,[0.,0.,0.],[0]*6400)
 def test_unknown_and_narrow_passage_fail_closed(self):
  data=[0]*6400
  for y in range(80):
   for x in range(38,43):data[y*80+x]=100
  for y in range(38,42):
   for x in range(38,43):data[y*80+x]=0
  g=FleetGrid(80,80,.1,[0,0,0],data)
  self.assertFalse(g.route([2,4,0],[6,4,0],.39))
 def test_formation_swept_width_and_waiting_point(self):
  g=self.grid();path=[[2.,2.,0.],[6.,2.,0.]]
  self.assertTrue(g.formation_fits(path,'row',1.4,[.39,.39],.1))
  self.assertFalse(g.formation_fits(path,'row',6,[.39,.39],.1))
  waiting=g.waiting_point([4,2,0],path,.5)
  self.assertIsNotNone(waiting);self.assertTrue(g.route([4,2,0],waiting,.5))
  self.assertGreaterEqual(distance_to_path(waiting,path),1.15)
 def test_peer_blocks_goal_and_transform_is_respected(self):
  g=FleetGrid(80,80,.1,[10.,-2.,1.57],[0]*6400)
  p=g.point((20,20));self.assertEqual(g.cell(p),(20,20))
  self.assertFalse(g.safe(p,.4,[(p,.4)]))
