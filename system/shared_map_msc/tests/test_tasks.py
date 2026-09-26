import unittest
from omnifleet_msc.task_board import TaskBoard
class TaskTests(unittest.TestCase):
 def setUp(self):
  self.board=TaskBoard({},lambda:None)
  self.robots={r:{'fleet_ready':True} for r in ('a','b')}
  self.config={'a':{'capabilities':['carry'],'payload_kg':3},'b':{'capabilities':[],'payload_kg':0}}
 def test_priority_capability_and_cost(self):
  for id_,priority in [('low',1),('high',9)]:self.board.submit({'task_id':id_,'goal':[0,0,0],'priority':priority})
  assignments=self.board.assign(self.robots,self.config,lambda r,p:1 if r=='b' else 2)
  self.assertEqual([(t['task_id'],t['owner']) for t in assignments],[('high','b'),('low','a')])
 def test_capability_payload_and_duplicate(self):
  request={'task_id':'carry','goal':[0,0,0],'capabilities':['carry'],'payload_kg':2}
  self.board.submit(request);self.assertTrue(self.board.submit(request)['duplicate'])
  self.assertEqual(self.board.assign(self.robots,self.config,lambda r,p:1)[0]['owner'],'a')
  self.assertFalse(self.board.assign(self.robots,self.config,lambda r,p:1))
 def test_fault_cannot_duplicate_claim(self):
  self.board.submit({'task_id':'one','goal':[0,0,0]});task=self.board.assign(self.robots,self.config,lambda r,p:1)[0]
  self.board.update('one',task['claim'],'RECOVERY_REQUIRED','offline')
  with self.assertRaises(ValueError):self.board.release('one')
  self.assertFalse(self.board.assign(self.robots,self.config,lambda r,p:1))
  self.board.release('one',True);next_=self.board.assign(self.robots,self.config,lambda r,p:1)[0]
  self.assertNotEqual(next_['claim'],task['claim'])
 def test_restart_quarantines_unfinished_claim(self):
  self.board.submit({'task_id':'one','goal':[0,0,0]});self.board.assign(self.robots,self.config,lambda r,p:1)
  restored=TaskBoard(self.board.tasks,lambda:None)
  self.assertEqual(restored.tasks['one']['state'],'RECOVERY_REQUIRED')
