import unittest
from omnifleet_msc.metrics import Metrics
class MetricsTests(unittest.TestCase):
 def test_unknown_contact_and_drop_recovery_are_explicit(self):
  m=Metrics();values={};states={'a':{'online':True,'fleet_pose':[0,0,0],'velocity':[0,0]},'b':{'online':True,'fleet_pose':[2,0,0],'velocity':[0,0]}}
  m.observe(values,states,{'b':[3,0]},0);m.observe(values,states,{'b':[3,0]},1)
  m.observe(values,states,{'b':[0,0]},2)
  self.assertIsNone(values['physical_collision_count']);self.assertEqual(values['dropouts']['b']['count'],1)
  self.assertEqual(values['recovery_durations'][0]['seconds'],2)
  self.assertEqual(values['minimum_robot_distance'],2)
  report=m.report({'state':'COMPLETED','members':['a','b'],'waypoints':[[1,0,0]]},values,states)
  self.assertEqual(report['mission_completion_rate'],1.);self.assertFalse(any(report['residual_goals'].values()))
