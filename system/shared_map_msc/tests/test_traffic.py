import unittest
from omnifleet_msc.traffic import Traffic,sample_path

class TrafficTests(unittest.TestCase):
    def setUp(self):
        self.t=Traffic();self.config={r:{'radius':.37} for r in ('a','b')}
        self.align={r:{'transform':[0,0,0]} for r in self.config}
    def test_path_intersection_sets_priority_and_deadlock(self):
        states={'a':{'fleet_pose':[0,0,0],'nav_active':True,'planned_path':[[0,0],[2,0]]},
                'b':{'fleet_pose':[1.3,0,3.14],'nav_active':True,'planned_path':[[1.3,0],[-1,0]]}}
        held,dead=self.t.evaluate(states,self.align,self.config,'a',0,[])
        self.assertEqual(set(held),{'a','b'});self.assertFalse(dead)
        _,dead=self.t.evaluate(states,self.align,self.config,'a',9,[]);self.assertTrue(dead)
    def test_nonconflicting_paths_keep_both_running(self):
        states={'a':{'fleet_pose':[0,0,0],'nav_active':True,'planned_path':[[0,0],[2,0]]},
                'b':{'fleet_pose':[0,2,0],'nav_active':True,'planned_path':[[0,2],[2,2]]}}
        self.assertEqual(self.t.evaluate(states,self.align,self.config,'a',0,[])[0],{})
    def test_region_owner_not_released_while_inside(self):
        zone={'id':'door','bounds':[0,0,1,1]}
        states={'a':{'fleet_pose':[.5,.5,0],'nav_active':False},
                'b':{'fleet_pose':[-1,.5,0],'nav_active':True,'planned_path':[[-1,.5],[1,.5]]}}
        held,_=self.t.evaluate(states,self.align,self.config,'a',0,[zone])
        self.assertEqual(self.t.owners['door'],'a');self.assertIn('b',held)
        states['a']['fleet_pose']=[2.5,2.5,0]
        self.t.evaluate(states,self.align,self.config,'a',1,[zone]);self.assertEqual(self.t.owners['door'],'b')
    def test_predictions_stop_at_path_end(self):
        self.assertEqual(sample_path([0,0,0],[[0,0],[1,0]],1)[-1],[1,0])

if __name__=='__main__':unittest.main()
