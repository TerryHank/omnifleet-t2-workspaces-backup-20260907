import copy,tempfile,unittest
from pathlib import Path
from omnifleet_msc.core import Coordinator
from omnifleet_msc.geometry import compose,inverse,offsets,safe_velocity

class FleetTests(unittest.TestCase):
    def setUp(self):
        self.tmp=tempfile.TemporaryDirectory();self.now=10.;self.seq={r:0 for r in ('robot_113','robot_104')}
        self.core=Coordinator({'robots':{r:{'radius':.37} for r in self.seq},'reference_robot':'robot_113','allow_motion':True},Path(self.tmp.name)/'journal.json',lambda:self.now)
        self.states={r:{'local_pose':[0.,0.,0.] if r=='robot_113' else [-1.4,0.,0.],
            'pose_age':0.,'velocity':[0.,0.],'velocity_age':0.,'localization_epoch':r+'-epoch',
            'nav_ready':True,'control_gate_ready':True,'control_mode':'LOCAL','nav_active':False,
            'pending_goal':False,'goal_error':'','completed_seq':0,'pose_stationary':True} for r in self.seq}
        self.feed()
        for robot in self.seq:self.core.operator({'op':'alignment','robot_id':robot,'verified':True,'transform':[0,0,0]})
    def tearDown(self):self.tmp.cleanup()
    def feed(self,dt=.1):
        while dt>1e-8:
            step=min(dt,.2);dt-=step;self.now+=step
            for r in self.seq:
                command=self.core.commands.get(r,{})
                if command.get('kind') in ('claim','hold','safety_stop','release'):
                    self.states[r]['control_epoch']=command['epoch'];self.states[r]['applied_command_seq']=command['seq']
                self.seq[r]+=1;self.core.heartbeat(r,'boot-'+r,self.seq[r],copy.deepcopy(self.states[r]))
    def start(self):
        return self.core.operator({'op':'start','mission_id':'one','leader_id':'robot_113',
            'frame_id':'fleet_map',
            'members':list(self.seq),'formation':'column','spacing':1.4,'waypoints':[[1.,0.,0.],[2.,0.,0.],[3.,0.,0.]]})
    def execute(self):
        self.start()
        for s in self.states.values():s['control_mode']='FLEET'
        self.feed();self.feed(.6);self.feed(.2);self.feed(.2)
        self.assertEqual(self.core.mission['state'],'EXECUTING')
    def test_duplicate_mission_does_not_start_again(self):
        self.start();seq=self.core.seq;reply=self.start();self.assertTrue(reply['duplicate']);self.assertEqual(seq,self.core.seq)
    def test_no_alignment_no_motion_command(self):
        self.core.history['alignments'].pop('robot_104')
        with self.assertRaisesRegex(ValueError,'alignment'):self.start()
        self.assertFalse(self.core.commands)
    def test_static_mode_refuses_all_motion(self):
        self.core.config['allow_motion']=False
        with self.assertRaisesRegex(ValueError,'locked'):self.start()
        self.assertFalse(self.core.commands)
    def test_zero_wheel_feedback_does_not_replace_stationary_pose(self):
        self.execute();self.core.operator({'op':'pause'})
        self.states['robot_104']['pose_stationary']=False;self.feed(.8)
        self.assertFalse(self.core.mission.get('stopped_confirmed',False))
    def test_missing_mission_frame_is_rejected(self):
        with self.assertRaisesRegex(ValueError,'frame_id'):
            self.core.operator({'op':'start','mission_id':'bad-frame','waypoints':[[0.,0.,0.]]})
    def test_clearance_sets_minimum_formation_spacing(self):
        self.core.config['safety_clearance']=.5
        with self.assertRaisesRegex(ValueError,'spacing'):self.start()
    def test_uninstalled_gate_blocks_start(self):
        self.states['robot_104']['control_gate_ready']=False;self.feed()
        with self.assertRaisesRegex(ValueError,'gate'):self.start()
        self.assertFalse(self.core.commands)
    def test_follower_waits_for_pending_action_ack(self):
        self.execute();previous=self.core.commands['robot_104']['seq']
        self.states['robot_104']['pending_goal']=True
        self.states['robot_113']['local_pose']=[.2,0.,0.];self.feed(1.2)
        self.assertEqual(self.core.commands['robot_104']['seq'],previous)
        self.states['robot_104']['pending_goal']=False;self.feed(.2)
        self.assertGreater(self.core.commands['robot_104']['seq'],previous)
    def test_stale_and_duplicate_identity_rejected(self):
        with self.assertRaises(ValueError):self.core.heartbeat('robot_104','other-boot',1,self.states['robot_104'])
        with self.assertRaises(ValueError):self.core.heartbeat('robot_104','boot-robot_104',self.seq['robot_104'],self.states['robot_104'])
        self.now+=2;self.assertFalse(self.core.robot('robot_104')['online'])
    def test_pause_requires_measured_stop(self):
        self.execute();self.core.operator({'op':'pause'});self.states['robot_104']['velocity']=[.1,0.];self.feed(.6)
        self.assertFalse(self.core.mission.get('stopped_confirmed',False))
        self.states['robot_104']['velocity']=[0.,0.];self.feed();self.feed(.6)
        self.assertTrue(self.core.mission['stopped_confirmed'])
    def test_cancel_never_dispatches_old_targets(self):
        self.execute();self.core.operator({'op':'cancel'});self.feed();self.feed(.6)
        self.assertEqual(self.core.mission['state'],'FAILED')
        self.assertTrue(all(c['kind']=='release' for c in self.core.commands.values()))
    def test_offline_causes_degraded_hold(self):
        self.execute();self.now+=2.;self.core.tick()
        self.assertEqual(self.core.mission['state'],'DEGRADED')
        self.assertTrue(all(c['kind']=='hold' for c in self.core.commands.values()))
    def test_three_waypoints_require_three_successes(self):
        self.execute()
        for i in range(3):
            self.feed();command=self.core.commands['robot_113'];self.assertEqual(command['kind'],'navigate')
            self.states['robot_113']['local_pose']=[i+1.,0.,0.];self.states['robot_104']['local_pose']=[i+1.-1.4,0.,0.]
            self.states['robot_113']['completed_seq']=command['seq'];self.feed()
            self.assertEqual(self.core.mission['current_waypoint'],i+1)
        self.feed();self.feed();self.feed(.6)
        self.assertEqual(self.core.mission['state'],'COMPLETED')
        self.assertEqual(self.core.mission['completed_waypoints'],[0,1,2])
    def test_localization_restart_invalidates_alignment(self):
        self.execute();self.states['robot_113']['localization_epoch']='new-map';self.feed()
        self.assertEqual(self.core.mission['state'],'DEGRADED')
        self.assertIsNone(self.core.robot('robot_104')['fleet_pose'])
    def test_leader_change_recomputes_offsets(self):
        self.execute();self.core.operator({'op':'pause'});self.feed(.6)
        self.core.operator({'op':'leader','robot_id':'robot_104'})
        self.core.operator({'op':'resume'});self.feed(.9)
        self.assertEqual(self.core.mission['leader_id'],'robot_104')
        self.assertAlmostEqual(self.core.commands['robot_113']['target'][0],-2.8)
    def test_offline_removal_requires_verified_stationary_pose(self):
        self.execute();self.now+=2;self.core.tick()
        with self.assertRaisesRegex(ValueError,'stationary'):
            self.core.operator({'op':'remove_member','robot_id':'robot_113'})
        self.core.operator({'op':'remove_member','robot_id':'robot_113','stationary_verified':True,'verified_pose':[0,0,0]})
        self.assertEqual(self.core.mission['leader_id'],'robot_104')
        self.assertEqual(self.core.retired['robot_113'],[0.,0.,0.])
    def test_bad_member_state_cannot_replace_valid_state(self):
        before=self.core.robot('robot_104')
        with self.assertRaises(ValueError):self.core.heartbeat('robot_104','boot-robot_104',999,{'velocity':['bad',0]})
        self.assertEqual(self.core.robot('robot_104'),before)
    def test_geometry_and_collision_filter(self):
        a=[2,3,.7];b=[1,-2,.5];result=compose(inverse(a),compose(a,b))
        for actual,expected in zip(result,b):self.assertAlmostEqual(actual,expected)
        self.assertEqual(offsets('column',1.4,2)[1],[-1.4,0.,0.])
        command,reason=safe_velocity({'pose':[0,0,0],'radius':.37},[.4,0],[{'pose':[1.1,0,3.14],'velocity':[.4,0],'radius':.37,'age':0.}])
        self.assertEqual(command,[0.,0.]);self.assertTrue(reason)

if __name__=='__main__':unittest.main()
