import unittest
from omnifleet_msc.geometry import safe_velocity
class ArcSafetyTests(unittest.TestCase):
 def test_turning_arc_is_checked_not_only_straight_line(self):
  own={'pose':[0,0,0],'radius':.1}
  peer={'pose':[.65,.35,0],'radius':.1,'age':0,'velocity':[0,0]}
  self.assertEqual(safe_velocity(own,[1,1],[peer],clearance=.05)[0],[0,0])
  self.assertEqual(safe_velocity(own,[1,0],[peer],clearance=.05)[0],[1,0])
