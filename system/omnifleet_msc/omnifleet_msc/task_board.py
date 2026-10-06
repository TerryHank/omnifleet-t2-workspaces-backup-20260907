"""Durable, exclusive task claims. Offline owners require confirmed release before reassignment."""
import copy,math,time,uuid
from .geometry import pose

class TaskBoard:
    def __init__(self,tasks,save,clock=time.monotonic):
        self.tasks=tasks;self.save=save;self.clock=clock
        for task in tasks.values():
            if task['state']=='PENDING' and task['task_id'].startswith('foxglove-'):
                task.update(state='CANCELED',reason='coordinator restarted; submit panel route again explicitly')
            if task['state'] in ('CLAIMED','RUNNING','FINISHING','CANCELING'):
                task.update(state='RECOVERY_REQUIRED',reason='coordinator restarted; confirm previous owner stopped')
    def submit(self,request):
        id_=str(request['task_id'])
        if not id_ or len(id_)>120:raise ValueError('invalid task ID')
        if id_ in self.tasks:return {'duplicate':True,'task':copy.deepcopy(self.tasks[id_])}
        if len(self.tasks)>=1000:raise ValueError('task queue full')
        required=request.get('capabilities',[])
        if not isinstance(required,list) or not all(isinstance(v,str) and len(v)<80 for v in required):raise ValueError('invalid capabilities')
        payload=float(request.get('payload_kg',0.));priority=int(request.get('priority',0))
        if not math.isfinite(payload) or payload<0:raise ValueError('invalid payload')
        route=[pose(v) for v in request.get('waypoints',[request['goal']])]
        if not 1<=len(route)<=100:raise ValueError('route requires 1..100 waypoints')
        robot=request.get('robot_id')
        task={'robot_id':robot,'waypoints':route,'task_id':id_,'goal':pose(request['goal']),'priority':priority,'capabilities':required,'payload_kg':payload,
            'state':'PENDING','owner':None,'claim':None,'created_at':time.time(),'attempts':0,'completed_at':None,'reason':''}
        self.tasks[id_]=task;self.save();return {'task':copy.deepcopy(task),'duplicate':False}
    def assign(self,robots,config,cost):
        busy={t['owner'] for t in self.tasks.values() if t['owner'] and t['state'] not in ('COMPLETED','CANCELED','FAILED')}
        assignments=[]
        for task in sorted(self.tasks.values(),key=lambda t:(-t['priority'],t['created_at'],t['task_id'])):
            if task['state']!='PENDING':continue
            candidates=[]
            for robot,state in robots.items():
                if task.get('robot_id') and task['robot_id']!=robot:continue
                if len(task.get('waypoints',[]))>1 and not state.get('supports_route'):continue
                capabilities=config[robot].get('capabilities',[])
                if robot in busy or not state.get('fleet_ready') or state.get('manual_active') or state.get('nav_active'):continue
                if not set(task['capabilities']).issubset(capabilities) or task['payload_kg']>config[robot].get('payload_kg',0.):continue
                value=cost(robot,task['goal'])
                if value is not None and math.isfinite(value):candidates.append((value,robot))
            if candidates:
                value,robot=min(candidates);task.update(owner=robot,claim=uuid.uuid4().hex,state='CLAIMED',cost=value,reason='')
                task['attempts']+=1;busy.add(robot);assignments.append(copy.deepcopy(task))
        if assignments:self.save()
        return assignments
    def update(self,id_,claim,state,reason=''):
        task=self.tasks[id_]
        if claim!=task['claim']:raise ValueError('stale claim')
        allowed={'CLAIMED':{'RUNNING','RECOVERY_REQUIRED','CANCELING'},'RUNNING':{'FINISHING','COMPLETED','RECOVERY_REQUIRED','CANCELING'},
                 'FINISHING':{'COMPLETED','RECOVERY_REQUIRED','CANCELING'},
                 'CANCELING':{'CANCELED','RECOVERY_REQUIRED'}}
        if state not in allowed.get(task['state'],set()):raise ValueError('invalid task transition')
        task.update(state=state,reason=reason)
        if state=='COMPLETED':task['completed_at']=time.time()
        self.save()
    def release(self,id_,stopped_verified=False,cancel=False):
        task=self.tasks[id_]
        if task['state'] not in ('RECOVERY_REQUIRED','CANCELING'):raise ValueError('task is not awaiting release')
        if not stopped_verified:raise ValueError('previous owner stop confirmation required')
        task.update(state='CANCELED' if cancel else 'PENDING',owner=None,claim=None,reason='');self.save()
    def status(self):return copy.deepcopy(self.tasks)
