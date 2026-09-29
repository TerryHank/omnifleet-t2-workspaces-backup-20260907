import copy,math,time,uuid
from .geometry import angle,pose,compose,inverse,offsets,closest_approach,world_velocity
from .storage import read,save
from .traffic import Traffic
from .task_board import TaskBoard
from .trail import LeaderTrail
from .maneuver import serial_plan,step as maneuver_step
from .metrics import Metrics

TERMINAL={'IDLE','COMPLETED','FAILED'}

class Coordinator:
    def __init__(self,config,journal,clock=time.monotonic):
        self.config=config;self.path=journal;self.clock=clock;self.robots={};self.commands={};self.seq=0
        clearance=config.get('safety_clearance',.2)
        if not math.isfinite(clearance) or clearance<.2:raise ValueError('safety clearance must be finite and at least 0.2 m')
        self.boot_epoch=uuid.uuid4().hex;self.stop_request_active=False;self.stop_confirmed=False
        self.online_states={}
        self.history=read(journal,{'missions':{},'alignments':{},'events':[]})
        self.event_sequence=max((e.get('sequence',0) for e in self.history['events']),default=0)
        self.trail=None
        self.mission=None;self.events=[];self.stopped_since=None
        self.last_targets={};self.errors={};self.started=0.;self.phase_since=clock()
        self.metrics={};self.resume_phase='FORMING'
        self.traffic=Traffic();self.retired=copy.deepcopy(self.history.get('retired',{}));self.joining=None
        self.shared_map_state={}
        self.grid=None;self.task_tick_at=-10.
        self.measurements=Metrics()
        self.task_board=TaskBoard(self.history.setdefault('tasks',{}),lambda:save(self.path,self.history),clock)
        unfinished=[m for m in self.history['missions'].values() if m['state'] not in TERMINAL]
        if unfinished:
            self.mission=copy.deepcopy(unfinished[-1]);self.mission.pop('maneuver',None)
            self.mission.update(state='DEGRADED',epoch=uuid.uuid4().hex,failure_reason='coordinator restarted; verify stopped state and resume explicitly')
            self.metrics=copy.deepcopy(self.mission.get('metrics',{}))
            self.started=clock()-max(0.,time.time()-self.metrics.get('started_at',time.time()))
            self.stop_all('coordinator restarted')
            self.history['missions'][self.mission['mission_id']]=copy.deepcopy(self.mission)
        save(self.path,self.history)

    def persist(self):
        self.history['retired']=copy.deepcopy(self.retired)
        if self.mission:
            self.mission['metrics']=copy.deepcopy(self.metrics)
            self.history['missions'][self.mission['mission_id']]=copy.deepcopy(self.mission)
        save(self.path,self.history)

    def event(self,kind,**detail):
        self.event_sequence+=1
        event={'sequence':self.event_sequence,'time':time.time(),'event':kind,**detail};self.events.append(event)
        self.events=self.events[-200:];self.history['events']=(self.history['events']+[event])[-1000:]

    def transition(self,state,reason=''):
        if self.mission is None:return
        if self.mission['state']==state and self.mission.get('failure_reason','')==reason:return
        self.mission.update(state=state,failure_reason=reason);self.phase_since=self.clock();self.stopped_since=None
        if state in ('COMPLETED','FAILED'):
            self.metrics=self.measurements.report(self.mission,self.metrics,{r:self.robot(r) for r in self.members()})
        self.event('mission_state',mission_id=self.mission['mission_id'],state=state,reason=reason)
        self.persist()

    def issue(self,robot,kind,**kwargs):
        previous=self.commands.get(robot,{})
        body={'kind':kind,'mission_id':self.mission['mission_id'] if self.mission else '',
              'epoch':self.mission['epoch'] if self.mission else self.boot_epoch,**kwargs}
        if all(previous.get(k)==v for k,v in body.items()) and set(previous)-{'seq'}==set(body):return
        self.seq+=1;self.commands[robot]={'seq':self.seq,**body}

    def heartbeat(self,robot,boot,seq,state):
        if robot not in self.config['robots']:raise ValueError('unknown robot ID')
        if not isinstance(state,dict) or not isinstance(boot,str) or not 0<len(boot)<128 or not isinstance(seq,int) or seq<1:
            raise ValueError('invalid heartbeat schema')
        if state.get('local_pose') is not None:pose(state['local_pose'])
        velocity=state.get('velocity',[0.,0.])
        if not isinstance(velocity,list) or len(velocity)!=2 or not all(isinstance(v,(int,float)) and math.isfinite(v) for v in velocity):
            raise ValueError('invalid velocity')
        for key in ('pose_age','velocity_age','obstacle_age'):
            value=state.get(key,99.)
            if not isinstance(value,(int,float)) or not math.isfinite(value) or value<0:raise ValueError('invalid observation age')
        for key in ('nav_ready','control_gate_ready','nav_active','pending_goal','estop','manual_active','pose_stationary','local_override','fleet_enabled'):
            if key in state and not isinstance(state[key],bool):raise ValueError('invalid state flag')
        path=state.get('planned_path',[])
        if not isinstance(path,list) or len(path)>200:raise ValueError('invalid path')
        for point in path:
            if not isinstance(point,list) or len(point)!=2:raise ValueError('invalid path point')
            pose([*point,0.])
        obstacles=state.get('obstacles',[])
        if not isinstance(obstacles,list) or len(obstacles)>10000:raise ValueError('invalid obstacle observations')
        for point in obstacles:
            if not isinstance(point,list) or len(point)!=2:raise ValueError('invalid obstacle point')
            pose([*point,0.])
        now=self.clock();previous=self.robots.get(robot)
        if previous:
            if previous['boot']!=boot and now-previous['received']<1.5:
                raise ValueError('duplicate live robot ID')
            if previous['boot']==boot and seq<=previous['seq']:raise ValueError('stale heartbeat sequence')
        self.robots[robot]={'boot':boot,'seq':seq,'received':now,'state':state}
        if previous is None:self.event('member_joined',robot=robot)
        elif previous['boot']!=boot:
            self.event('member_restarted',robot=robot)
            if self.mission and robot in self.mission['members'] and self.mission['state'] not in TERMINAL:
                self.transition('DEGRADED',robot+' restarted; explicit resume required')
        self.consume_navigation_result(robot,state.get('navigation_result'))
        self.tick()
        peers=[]
        current_command=self.commands.get(robot,{})
        ui_command=str(current_command.get('task_id','')).startswith('foxglove-') or bool(self.mission and self.mission['state'] not in TERMINAL and self.mission['mission_id'].startswith('foxglove-'))
        for other in self.config['robots']:
            if other!=robot:
                data=self.robot(other)
                if ui_command and other not in getattr(self,'ui_participants',self.config['robots']) and other not in self.retired and (not data['online'] or data['fleet_pose'] is None):
                    continue
                if other in self.retired and (not data['online'] or data['fleet_pose'] is None):
                    peers.append({'robot_id':other,'pose':self.retired[other],'age':0.,'velocity':[0.,0.],
                                  'radius':self.config['robots'][other]['radius']})
                else:
                    peers.append({'robot_id':other,'pose':data['fleet_pose'],'age':data['pose_age'],
                        'velocity':data.get('velocity',[0.,0.]),'radius':self.config['robots'][other]['radius']})
        alignment=self.history['alignments'].get(robot)
        return {'coordinator_boot':self.boot_epoch,'command':self.commands.get(robot,{'seq':0,'kind':'observe','epoch':self.boot_epoch}),'lease_seconds':.6,
                'safety_clearance':self.config.get('safety_clearance',.2),
                'peers':peers,'alignment':alignment,'leader_id':self.mission['leader_id'] if self.mission else None,
                'mission_state':'TASKS' if self.tasks_active() and (not self.mission or self.mission['state'] in TERMINAL) else self.mission['state'] if self.mission else 'IDLE'}

    def consume_navigation_result(self,robot,result):
        if not isinstance(result,dict) or result.get('stopped_verified') is not True:return
        if result.get('code') not in ('CANCELED','PREEMPTED_BY_LOCAL'):return
        task=self.task_board.tasks.get(result.get('task_id'))
        if not task or task.get('owner')!=robot:return
        if (not task.get('navigation_epoch') or result.get('command_epoch')!=task['navigation_epoch'] or
            result.get('revision')!=task.get('navigation_seq')):return
        if task['state'] not in ('CLAIMED','RUNNING','FINISHING','RECOVERY_REQUIRED','CANCELING'):return
        if task['state'] not in ('RECOVERY_REQUIRED','CANCELING'):
            self.task_board.update(task['task_id'],task['claim'],'CANCELING','onboard navigation canceled')
        self.task_board.release(task['task_id'],True,True)
        self.issue(robot,'observe',reason='onboard cancellation confirmed; explicit new task required')
        self.event('task_canceled_onboard',robot=robot,task_id=result['task_id'])

    def robot(self,robot):
        item=self.robots.get(robot);now=self.clock()
        if not item:return {'robot_id':robot,'online':False,'pose_age':99.,'fleet_pose':None,'health':'OFFLINE'}
        state=copy.deepcopy(item['state']);age=max(0.,now-item['received'])
        state.update(robot_id=robot,online=age<1.5,heartbeat_age=age,agent_boot=item['boot'])
        state['pose_age']=state.get('pose_age',99.)+age
        alignment=self.history['alignments'].get(robot)
        ref=self.robots.get(self.config['reference_robot'],{}).get('state',{}).get('localization_epoch')
        aligned=alignment and alignment.get('epoch')==state.get('localization_epoch') and alignment.get('reference_epoch')==ref
        state['fleet_pose']=compose(alignment['transform'],state['local_pose']) if aligned and state.get('local_pose') and state['pose_age']<.8 else None
        state['alignment_verified']=bool(aligned)
        state['local_health']=state.get('health')
        if state.get('estop'):state['health']='ESTOP'
        elif state.get('manual_active'):state['health']='MANUAL'
        elif state.get('local_override'):state['health']='LOCAL_CONTROL'
        elif state.get('goal_error'):state['health']='FAULT'
        elif state.get('nav_ready') and state.get('local_pose') and not aligned:state['health']='WAITING_ALIGNMENT'
        elif state.get('nav_ready') and not state.get('control_gate_ready'):state['health']='WAITING_CONTROL_GATE'
        if age>=1.5:state['health']='OFFLINE'
        state['fleet_ready']=bool(not state.get('local_override',False) and state.get('fleet_enabled',True) and state['online'] and state['fleet_pose'] is not None and state.get('nav_ready') and state.get('control_gate_ready') and not state.get('estop') and not state.get('manual_active') and not state.get('goal_error') and
            (not self.config.get('require_shared_map') or state.get('shared_map_ready')) and
            (not self.config.get('require_obstacles') or state.get('obstacle_observations_complete') and state.get('obstacle_age',99)+age<=2.5))
        return state

    def members(self):
        return self.mission['members'] if self.mission else list(self.config['robots'])

    def stop_all(self,reason=''):
        for robot in self.members():self.issue(robot,'hold',reason=reason)

    def all_stopped(self,robots=None):
        for robot in (robots if robots is not None else self.members()):
            s=self.robot(robot);v=s.get('velocity',[99.,99.])
            command=self.commands.get(robot,{})
            if command.get('kind') in ('claim','hold','safety_stop') and (s.get('control_epoch')!=command.get('epoch') or s.get('applied_command_seq')!=command.get('seq')):
                self.stopped_since=None;return False
            if not s['online'] or not s.get('pose_stationary',False) or s.get('velocity_age',99.)+s.get('heartbeat_age',99.)>.5 or abs(v[0])>.02 or abs(v[1])>.05:
                self.stopped_since=None;return False
            if s.get('nav_active') or s.get('pending_goal'):
                self.stopped_since=None;return False
        if self.stopped_since is None:self.stopped_since=self.clock()
        return self.clock()-self.stopped_since>=.5

    def validate_start(self,request):
        if not self.config.get('allow_motion',False):raise ValueError('fleet motion is locked: static acceptance mode')
        if any(t['state'] in ('CLAIMED','RUNNING','FINISHING','CANCELING','RECOVERY_REQUIRED') for t in self.task_board.tasks.values()):
            raise ValueError('finish or cancel independently assigned tasks before formation navigation')
        if request.get('frame_id')!='fleet_map':raise ValueError('mission frame_id must be fleet_map')
        members=request.get('members',list(self.config['robots']));leader=request.get('leader_id',members[0])
        if not 1<=len(members)<=2 or len(set(members))!=len(members) or leader not in members:
            raise ValueError('invalid two-robot membership/leader')
        if any(r not in self.config['robots'] for r in members):raise ValueError('unknown member')
        points=[pose(p) for p in request['waypoints']]
        if not 1<=len(points)<=100:raise ValueError('mission requires 1..100 waypoints')
        spacing=float(request.get('spacing',1.4));kind=request.get('formation','column')
        if request.get('follow_model','offset') not in ('offset','trail'):raise ValueError('invalid follow model')
        if request.get('follow_model')=='trail' and any(not self.robot(r).get('supports_route') for r in members):raise ValueError('route-capable agent required for trail following')
        offsets(kind,spacing,len(members))
        if not math.isfinite(spacing) or spacing<1.4 or spacing>5:raise ValueError('spacing must be 1.4..5 m for this stop envelope')
        for robot in members:
            s=self.robot(robot)
            if not s['online']:raise ValueError(robot+' is offline')
            if s['fleet_pose'] is None:raise ValueError(robot+' map alignment/pose not verified')
            if not s.get('nav_ready'):raise ValueError(robot+' Nav2 not ready')
            if not s.get('control_gate_ready'):raise ValueError(robot+' exclusive velocity gate not ready')
            if self.config.get('require_shared_map') and not s.get('shared_map_ready'):raise ValueError(robot+' shared map not ready')
            if self.config.get('require_obstacles') and (not s.get('obstacle_observations_complete') or s.get('obstacle_age',99)+s.get('heartbeat_age',0)>2.5):raise ValueError(robot+' obstacle observations unavailable')
            if s.get('estop') or s.get('manual_active'):raise ValueError(robot+' under estop/manual control')
        if len(members)==2:
            a,b=(self.robot(r) for r in members)
            minimum=sum(self.config['robots'][r]['radius'] for r in members)+self.config.get('safety_clearance',.2)
            if spacing<minimum+.34:raise ValueError('spacing is too small for configured clearance and stopping envelope')
            if math.dist(a['fleet_pose'][:2],b['fleet_pose'][:2])<minimum:raise ValueError('robots already below minimum separation')
        return members,leader,points,spacing,kind

    def operator(self,request):
        op=request['op']
        if op in ('stop','cancel','pause','resume') and (not self.mission or self.mission['state'] in TERMINAL):
            tasks=self.task_board.tasks
            if op=='resume':
                if self.stop_request_active or not all(self.robot(r).get('fleet_ready') for r in self.config['robots']):raise ValueError('fleet not ready to resume tasks')
                self.history['tasks_paused']=False
            elif op=='pause':self.history['tasks_paused']=True
            else:self.history['tasks_paused']=True
            if op!='resume':
                for task in tasks.values():
                    if task['state']=='PENDING' and op in ('stop','cancel'):task['state']='CANCELED'
                    if task['state'] in ('CLAIMED','RUNNING','FINISHING','CANCELING','RECOVERY_REQUIRED'):
                        task.update(state='RECOVERY_REQUIRED',cancel_requested=op in ('stop','cancel'),reason=op+' requested')
                        self.issue(task['owner'],'hold',reason=op+' requested')
            save(self.path,self.history)
            if op in ('pause','resume','cancel'):return self.status()
        if op=='stop' and (not self.mission or self.mission['state'] in TERMINAL):
            self.stop_request_active=True;self.stop_confirmed=False;self.stopped_since=None
            for robot in self.config['robots']:self.issue(robot,'safety_stop',reason='fleet safety stop')
            self.event('fleet_safety_stop');save(self.path,self.history);return self.status()
        if op=='release_stop':
            if self.mission and self.mission['state'] not in TERMINAL|{'PAUSED','DEGRADED'}:raise ValueError('stop/pause mission first')
            for robot in self.config['robots']:
                state=self.robot(robot)
                if not state['online'] or state.get('stopped_seconds',0)<.5 or state.get('nav_active') or state.get('pending_goal'):
                    raise ValueError(robot+' has no stable stopped confirmation')
            for robot in self.config['robots']:self.issue(robot,'release_stop')
            self.stop_request_active=False;self.stop_confirmed=False
            return self.status()
        if op=='alignment':
            if self.tasks_active():raise ValueError('cancel independently assigned tasks before realignment')
            robot=request['robot_id']
            if robot not in self.config['robots']:raise ValueError('unknown robot')
            if self.mission and self.mission['state'] not in TERMINAL:
                if self.mission['state'] not in ('PAUSED','DEGRADED') or robot==self.config['reference_robot']:
                    raise ValueError('cancel mission before changing reference alignment')
                if not self.all_stopped():raise ValueError('all members must be confirmed stopped before realignment')
            if request.get('verified') is not True:raise ValueError('alignment requires explicit verification')
            current=self.robots.get(robot,{}).get('state',{})
            ref=self.robots.get(self.config['reference_robot'],{}).get('state',{})
            if not current.get('localization_epoch') or not ref.get('localization_epoch'):raise ValueError('localization must be running before alignment')
            self.history['alignments'][robot]={'transform':pose(request['transform']),
                'revision':uuid.uuid4().hex,'reference':request.get('reference','fleet_map'),'verified_at':time.time(),
                'epoch':current['localization_epoch'],'reference_epoch':ref['localization_epoch']}
            self.event('alignment_verified',robot=robot)
            save(self.path,self.history);return self.status()
        if op=='start':
            mission_id=str(request['mission_id'])
            if not mission_id or len(mission_id)>120:raise ValueError('invalid mission ID')
            if mission_id in self.history['missions']:return {'duplicate':True,'mission':self.history['missions'][mission_id]}
            if self.mission and self.mission['state'] not in TERMINAL:raise ValueError('another mission is active')
            members,leader,points,spacing,kind=self.validate_start(request)
            self.mission={'mission_id':mission_id,'epoch':uuid.uuid4().hex,'members':members,'leader_id':leader,
                'frame_id':'fleet_map',
                'waypoints':points,'current_waypoint':0,'completed_waypoints':[], 'formation':kind,
                'follow_model':request.get('follow_model','offset'),'requested_formation':kind,'spacing':spacing,'state':'IDLE','failure_reason':''}
            self.metrics={'minimum_robot_distance':None,'formation_error_sum':0.,'formation_error_samples':0,
                'maximum_formation_error':0.,'communication_losses':0,'navigation_failures':0,
                'physical_collision_count':None,'started_at':time.time(),
                'waypoint_checkpoints':{r:[] for r in members},'pending_checkpoints':{r:{} for r in members}}
            self.measurements=Metrics()
            self.started=self.clock();self.last_targets={};self.errors={};self.transition('PRECHECK')
            for robot in members:self.issue(robot,'claim')
        elif op in ('pause','stop','cancel'):
            if not self.mission or self.mission['state'] in TERMINAL:return self.status()
            self.resume_phase=self.mission['state'];self.stop_all(op)
            self.transition('PAUSED' if op=='pause' else 'ABORTING',op+' requested')
            if op=='stop':
                self.stop_request_active=True;self.stop_confirmed=False;self.stopped_since=None
                for robot in self.members():self.issue(robot,'safety_stop',reason='fleet safety stop')
        elif op=='resume':
            if not self.mission or self.mission['state'] not in ('PAUSED','DEGRADED'):raise ValueError('mission not paused/degraded')
            self.validate_start({**self.mission,'formation':self.mission['formation']})
            self.mission.pop('maneuver',None)
            self.mission['epoch']=uuid.uuid4().hex;self.last_targets={};self.transition('PRECHECK')
            for robot in self.members():self.issue(robot,'claim')
        elif op=='leader':
            if not self.mission or self.mission['state'] not in ('PAUSED','DEGRADED'):raise ValueError('pause before leader change')
            if request['robot_id'] not in self.members() or not self.robot(request['robot_id'])['online']:raise ValueError('leader unavailable')
            self.mission['leader_id']=request['robot_id'];self.event('leader_changed',robot=request['robot_id'])
            self.history['missions'][self.mission['mission_id']]=copy.deepcopy(self.mission);save(self.path,self.history)
        elif op=='formation':
            if not self.mission or self.mission['state'] not in ('PAUSED','DEGRADED'):raise ValueError('pause before formation change')
            spacing=float(request.get('spacing',self.mission['spacing']))
            if not math.isfinite(spacing) or not 1.4<=spacing<=5:raise ValueError('spacing must be 1.4..5 m')
            minimum=sum(self.config['robots'][r]['radius'] for r in self.members())+self.config.get('safety_clearance',.2)+.34
            if len(self.members())>1 and spacing<minimum:raise ValueError('spacing below stopping clearance')
            self.mission['spacing']=spacing
            kind=request['formation'];offsets(kind,spacing,len(self.members()))
            self.mission['formation']=kind;self.mission['requested_formation']=kind
            self.history['missions'][self.mission['mission_id']]=copy.deepcopy(self.mission);save(self.path,self.history)
        elif op=='remove_member':
            if not self.mission or self.mission['state'] not in ('PAUSED','DEGRADED'):raise ValueError('pause before removing a member')
            robot=request['robot_id']
            if robot not in self.members() or len(self.members())<2:raise ValueError('cannot remove this member')
            state=self.robot(robot)
            if state['online'] and state.get('stopped_seconds',0)>=.5 and not state.get('nav_active'):
                parked=state['fleet_pose']
            elif request.get('stationary_verified') is True and 'verified_pose' in request:
                parked=pose(request['verified_pose'])
            else:raise ValueError('faulted robot must have a verified stationary pose before others can continue')
            if parked is None:raise ValueError('stationary pose unavailable')
            self.retired[robot]=parked;self.issue(robot,'safety_stop',reason='removed robot must remain parked',scope='fleet_only')
            self.mission['members'].remove(robot)
            if self.mission['leader_id']==robot:self.mission['leader_id']=self.members()[0]
            self.mission['formation']='column';self.event('member_removed',robot=robot)
            self.persist()
        elif op=='add_member':
            if not self.mission or self.mission['state'] not in ('PAUSED','DEGRADED'):raise ValueError('pause before rejoining')
            robot=request['robot_id'];state=self.robot(robot)
            if robot not in self.config['robots'] or robot in self.members() or len(self.members())>=2:raise ValueError('invalid rejoining member')
            if not state['online'] or state['fleet_pose'] is None or not state.get('control_gate_ready'):raise ValueError('rejoining member not ready')
            if 'rendezvous' in request:meeting=pose(request['rendezvous'])
            else:
                leader=self.mission['leader_id'];leader_pose=self.robot(leader)['fleet_pose']
                if self.grid is None or leader_pose is None:raise ValueError('shared map and leader pose required for rendezvous')
                meeting=compose(leader_pose,[-self.mission['spacing'],0.,0.])
                radius=self.config['robots'][robot]['radius']+.1
                peer=(leader_pose,self.config['robots'][leader]['radius']+.1)
                if not self.grid.route(state['fleet_pose'],meeting,radius,[peer]):raise ValueError('no safe automatic rendezvous; provide a verified reachable point')
            self.joining={'robot':robot,'rendezvous':meeting}
            self.mission['members'].append(robot);self.retired.pop(robot,None)
            self.event('member_rejoin_requested',robot=robot)
            self.persist()
        elif op=='zones':
            if self.mission and self.mission['state'] not in TERMINAL:raise ValueError('cancel before changing traffic zones')
            zones=request['zones'];ids=set()
            for zone in zones:
                if zone['id'] in ids:raise ValueError('duplicate zone ID')
                ids.add(zone['id']);bounds=zone['bounds']
                if len(bounds)!=4 or not all(math.isfinite(float(v)) for v in bounds) or bounds[0]>=bounds[2] or bounds[1]>=bounds[3]:raise ValueError('invalid zone bounds')
            self.config['zones']=zones;self.history['zones']=zones;save(self.path,self.history)
        else:raise ValueError('unsupported operation')
        return self.status()

    def target(self,robot,target,role,route=None):
        now=self.clock();old=self.last_targets.get(robot)
        if self.robot(robot).get('pending_goal'):return
        if old and now-old[0]<1.:return
        if old and math.dist(target[:2],old[1][:2])<.12 and abs(angle(target[2]-old[1][2]))<.15:
            return
        alignment=self.history['alignments'][robot]['transform']
        extra={'route':[compose(inverse(alignment),p) for p in route]} if route is not None else {}
        self.issue(robot,'navigate',target=compose(inverse(alignment),target),role=role,
                   waypoint=self.mission['current_waypoint'],**extra)
        self.last_targets[robot]=(now,list(target))

    def tick(self):
        changed=False
        for robot in self.config['robots']:
            current=robot in self.robots and self.clock()-self.robots[robot]['received']<1.5
            if current!=self.online_states.get(robot,False):
                self.event('member_online' if current else 'member_offline',robot=robot);changed=True
                if not current and robot in self.robots and self.mission:
                    self.metrics.setdefault('offline_detection_seconds',[]).append({'robot':robot,'seconds':self.clock()-self.robots[robot]['received']})
            self.online_states[robot]=current
        if changed:save(self.path,self.history)
        if self.stop_request_active:self.stop_confirmed=self.all_stopped(self.config['robots'])
        if not self.mission or self.mission['state'] in TERMINAL:
            self.tick_tasks();return
        phase=self.mission['state'];states={r:self.robot(r) for r in self.members()}
        self.errors={};lead=self.mission['leader_id'];leader_position=states[lead].get('fleet_pose')
        if leader_position:
            members=[lead]+[r for r in self.members() if r!=lead]
            off=offsets(self.mission['formation'],self.mission['spacing'],len(members))
            for robot,offset in zip(members[1:],off[1:]):
                actual=states[robot].get('fleet_pose')
                if actual:
                    target=compose(leader_position,offset)
                    self.errors[robot]=[math.dist(actual[:2],target[:2]),abs(angle(actual[2]-target[2]))]
        self.measurements.observe(self.metrics,states,self.errors,self.clock(),phase in ('EXECUTING','RECOVERING') and not self.mission.get('maneuver'))
        bad=[r for r,s in states.items() if not s['online'] or s['fleet_pose'] is None or not s.get('nav_ready') or s.get('velocity_age',99.)+s.get('heartbeat_age',99.)>.5 or s.get('estop') or s.get('manual_active') or s.get('local_override')]
        if self.config.get('require_shared_map'):bad+= [r for r,s in states.items() if not s.get('shared_map_ready') and r not in bad]
        if self.config.get('require_obstacles'):bad += [r for r,s in states.items() if (not s.get('obstacle_observations_complete') or s.get('obstacle_age',99)+s.get('heartbeat_age',0)>2.5) and r not in bad]
        if bad and phase not in ('ABORTING','PAUSED','DEGRADED'):
            self.metrics['communication_losses']+=sum(not states[r]['online'] for r in bad)
            self.stop_all('member not ready');self.transition('DEGRADED',','.join(bad)+' unavailable/manual');return
        if phase in ('PAUSED','DEGRADED'):
            self.stop_all(self.mission['failure_reason']);self.mission['stopped_confirmed']=self.all_stopped();return
        if phase=='RECOVERING' and self.mission.get('maneuver'):
            errors=[s.get('goal_error') for s in states.values() if s.get('goal_error')]
            if errors:self.stop_all(errors[0]);self.transition('DEGRADED',errors[0]);return
            if maneuver_step(self,states):return
        if phase=='ABORTING':
            if self.mission.get('failure_reason')!='stop requested':self.stop_all('cancel')
            if self.all_stopped():
                self.transition('FAILED',self.mission.get('failure_reason','canceled')+'; all members stopped')
                for robot in self.members():self.issue(robot,'release')
            return
        if phase=='PRECHECK':
            if all(s.get('control_mode')=='FLEET' for s in states.values()) and self.all_stopped():
                self.last_targets={};self.transition('RECOVERING' if self.joining else 'FORMING')
            elif self.clock()-self.phase_since>10:
                self.stop_all('takeover not confirmed');self.transition('DEGRADED','control takeover timeout')
            return
        if phase=='STOPPING':
            self.stop_all('mission completed')
            if self.all_stopped():
                self.metrics['duration_seconds']=self.clock()-self.started
                self.metrics['final_velocities']={r:self.robot(r)['velocity'] for r in self.members()}
                self.metrics['residual_goals']={r:self.robot(r).get('nav_active',False) or self.robot(r).get('pending_goal',False) for r in self.members()}
                self.mission['metrics']=copy.deepcopy(self.metrics);self.transition('COMPLETED')
                for robot in self.members():self.issue(robot,'release')
            return
        leader=self.mission['leader_id'];followers=[r for r in self.members() if r!=leader]
        leader_pose=states[leader]['fleet_pose'];kind=self.mission['formation']
        if phase=='REGROUPING' and self.grid is not None and kind!=self.mission['requested_formation']:
            radii=[self.config['robots'][r]['radius'] for r in [leader]+followers]
            if self.observed_grid(states).formation_fits([leader_pose],self.mission['requested_formation'],self.mission['spacing'],radii,.1):
                self.mission['formation']=kind=self.mission['requested_formation'];self.last_targets.clear()
            else:self.stop_all('requested final formation has insufficient room');self.transition('ABORTING','requested final formation has insufficient room');return
        if self.grid is not None and phase=='EXECUTING' and followers and self.clock()-getattr(self,'envelope_check_at',-10.)>=1.:
            self.envelope_check_at=self.clock();goal=self.mission['waypoints'][self.mission['current_waypoint']]
            planning_grid=self.observed_grid(states)
            route=planning_grid.route(leader_pose,goal,self.config['robots'][leader]['radius']+.1)
            radii=[self.config['robots'][r]['radius'] for r in [leader]+followers]
            if not route:self.stop_all('leader route blocked');self.transition('DEGRADED','shared map has no footprint-safe leader route');return
            if self.mission.get('follow_model')!='trail' and not planning_grid.formation_fits(route,kind,self.mission['spacing'],radii,.1):
                if kind!='column' and planning_grid.formation_fits(route,'column',self.mission['spacing'],radii,.1):
                    self.mission['formation']='column';self.last_targets.clear();self.stop_all('reforming column')
                    self.transition('FORMING','formation narrowed for passage');return
                if self.begin_passage(states,goal,'formation envelope blocked'):return
            elif kind!=self.mission['requested_formation'] and planning_grid.formation_fits(route,self.mission['requested_formation'],self.mission['spacing'],radii,.1):
                self.mission['formation']=self.mission['requested_formation'];self.last_targets.clear();self.stop_all('restoring requested formation')
                self.transition('FORMING','clearance restored');return
        if phase=='RECOVERING' and self.joining:
            robot=self.joining['robot'];self.issue(leader,'hold',reason='waiting for rejoin')
            self.target(robot,self.joining['rendezvous'],'rejoin')
            command=self.commands[robot]
            if states[robot].get('goal_error'):
                self.stop_all('rejoin failed');self.transition('DEGRADED',states[robot]['goal_error'])
            elif states[robot].get('completed_seq')==command['seq']:
                self.joining=None;self.last_targets={};self.transition('FORMING');self.event('rendezvous_reached',robot=robot)
            return
        held,deadlock=self.traffic.evaluate(states,self.history['alignments'],self.config['robots'],leader,self.clock(),self.history.get('zones',self.config.get('zones',[])),self.config.get('safety_clearance',.2))
        self.metrics['path_conflicts']=self.traffic.conflict_count;self.metrics['deadlocks']=self.traffic.deadlock_count
        if deadlock:
            goal=self.mission['waypoints'][self.mission['current_waypoint']]
            if self.grid is not None and self.begin_passage(states,goal,'traffic deadlock'):return
            self.stop_all('traffic deadlock');self.transition('DEGRADED','traffic deadlock: no safe automatic waiting route');return
        off=offsets(kind,self.mission['spacing'],len(self.members()))
        for robot in self.members():
            actual=states[robot]['fleet_pose']
            pending=self.metrics.get('pending_checkpoints',{}).get(robot,{})
            for index,target in list(pending.items()):
                if math.dist(actual[:2],target[:2])<.25 and abs(angle(actual[2]-target[2]))<.25:
                    self.metrics['waypoint_checkpoints'].setdefault(robot,[]).append(int(index));pending.pop(index)
        trail_mode=self.mission.get('follow_model')=='trail' and phase in ('EXECUTING','REGROUPING')
        if trail_mode:
            if self.trail is None:self.stop_all('leader trail unavailable');self.transition('DEGRADED','regroup before rebuilding leader trail');return
            self.trail.append(leader_pose)
        formed=True
        for i,robot in enumerate(followers,1):
            target=compose(leader_pose,off[i]);actual=states[robot]['fleet_pose'];follow_route=None
            if trail_mode:
                try:target,follow_route=self.trail.follow(robot,actual,i*self.mission['spacing'])
                except ValueError as error:self.stop_all(str(error));self.transition('DEGRADED',str(error));return
            distance=math.dist(target[:2],actual[:2]);heading=abs(angle(target[2]-actual[2]));self.errors[robot]=[distance,heading]
            self.metrics['formation_error_sum']+=distance;self.metrics['formation_error_samples']+=1
            self.metrics['maximum_formation_error']=max(self.metrics['maximum_formation_error'],distance)
            formed=formed and distance<.25 and heading<.25
            separation=math.dist(leader_pose[:2],actual[:2]);minimum=self.metrics['minimum_robot_distance']
            self.metrics['minimum_robot_distance']=separation if minimum is None else min(minimum,separation)
            if robot in held:self.issue(robot,'hold',reason=held[robot]);self.last_targets.pop(robot,None)
            else:self.target(robot,target,'follower',follow_route)
            if distance>2. and phase=='EXECUTING' and robot not in held:
                self.issue(leader,'hold',reason='follower lagging');self.transition('RECOVERING','follower lagging');return
            error=states[robot].get('goal_error')
            if error:
                if 'unsafe target' in error and kind=='row':
                    self.mission['formation']='column';self.mission['epoch']=uuid.uuid4().hex;self.last_targets={}
                    self.event('formation_narrowed',reason=error)
                    self.transition('PRECHECK','row blocked; reforming column')
                    for member in self.members():self.issue(member,'claim')
                    return
                self.metrics['navigation_failures']+=1;self.stop_all(error)
                self.transition('DEGRADED',robot+': '+error);return
        if phase in ('FORMING','RECOVERING','REGROUPING'):
            self.issue(leader,'hold',reason=phase)
            if formed:
                if phase=='REGROUPING':self.transition('STOPPING')
                else:
                    if self.mission.get('follow_model')=='trail':
                        self.trail=LeaderTrail([compose(leader_pose,off[i]) for i in reversed(range(len(off)))])
                    self.last_targets.pop(leader,None);self.transition('EXECUTING')
            elif self.clock()-self.phase_since>90:
                self.stop_all('formation timeout');self.transition('DEGRADED','formation/rejoin timeout')
            return
        if phase=='EXECUTING':
            if leader in held:self.issue(leader,'hold',reason=held[leader]);self.last_targets.pop(leader,None);return
            s=states[leader];command=self.commands.get(leader,{})
            if s.get('goal_error'):
                self.stop_all('leader navigation failed');self.transition('DEGRADED',s['goal_error']);return
            if command.get('kind')=='navigate' and s.get('completed_seq')==command['seq']:
                index=self.mission['current_waypoint'];self.mission['completed_waypoints'].append(index)
                self.metrics['waypoint_checkpoints'].setdefault(leader,[]).append(index)
                for i,robot in enumerate(followers,1):
                    self.metrics['pending_checkpoints'].setdefault(robot,{})[str(index)]=compose(leader_pose,off[i])
                self.mission['current_waypoint']+=1;self.last_targets.pop(leader,None)
                self.persist()
                if self.mission['current_waypoint']>=len(self.mission['waypoints']):
                    self.transition('REGROUPING');return
            self.target(leader,self.mission['waypoints'][self.mission['current_waypoint']],'leader')

    def status(self):
        robots={r:self.robot(r) for r in self.config['robots']}
        positions=[r['fleet_pose'] for r in robots.values() if r['fleet_pose'] is not None]
        distance=math.dist(positions[0][:2],positions[1][:2]) if len(positions)==2 else None
        mission=copy.deepcopy(self.mission) if self.mission else {'state':'IDLE','mission_id':None,'leader_id':None,'current_waypoint':None}
        if mission['state'] in TERMINAL and self.tasks_active():mission['state']='TASKS'
        return {'mission_id':mission['mission_id'],'mission_state':mission['state'],'leader_id':mission['leader_id'],
            'frame_id':'fleet_map','reference_robot':self.config['reference_robot'],
            'motion_enabled':bool(self.config.get('allow_motion',False)),
            'shared_map':copy.deepcopy(self.shared_map_state),
            'task_allocation':{'supported':True,'endpoint':'/v1/tasks','tasks':self.task_board.status()},
            'stop_requested':self.stop_request_active,'stop_confirmed':self.stop_confirmed,
            'active_robot_ids':[r for r in self.members() if robots[r]['online']],
            'online_robot_ids':[r for r,s in robots.items() if s['online']],
            'offline_robot_ids':[r for r,s in robots.items() if not s['online']],
            'failed_robot_ids':[r for r,s in robots.items() if s.get('goal_error')],
            'formation_type':mission.get('formation'),'formation_error':copy.deepcopy(self.errors),
            'minimum_robot_distance':self.metrics.get('minimum_robot_distance',distance),'current_robot_distance':distance,
            'current_stage':mission['state'],'current_waypoint':mission.get('current_waypoint'),
            'region_owners':copy.deepcopy(self.traffic.owners),
            'failure_reason':mission.get('failure_reason',''),'mission':mission,'robots':robots,'metrics':copy.deepcopy(self.metrics)}

    def task_operator(self,request):
        op=request.get('op','list')
        if op=='list':return self.task_board.status()
        if op=='submit':
            if request.get('frame_id')!='fleet_map':raise ValueError('task frame_id must be fleet_map')
            if request.get('robot_id') and request['robot_id'] not in self.config['robots']:raise ValueError('unknown task robot')
            return self.task_board.submit(request)
        task=self.task_board.tasks[request['task_id']]
        if op=='cancel':
            if task['state']=='PENDING':task['state']='CANCELED';save(self.path,self.history)
            elif task['state'] not in ('COMPLETED','CANCELED','FAILED','RECOVERY_REQUIRED','CANCELING'):
                self.task_board.update(task['task_id'],task['claim'],'CANCELING','operator canceled')
                self.issue(task['owner'],'hold',reason='independent task canceled')
            elif task['state']=='RECOVERY_REQUIRED':task['cancel_requested']=True;save(self.path,self.history)
        elif op=='confirm_stopped':
            if task['state'] not in ('RECOVERY_REQUIRED','CANCELING'):raise ValueError('task is not awaiting previous-owner confirmation')
            if request.get('stationary_verified') is not True:raise ValueError('explicit stationary confirmation required')
            state=self.robot(task['owner'])
            if state['online'] and (not state.get('pose_stationary') or state.get('nav_active') or state.get('pending_goal')):raise ValueError('live state contradicts stationary confirmation')
            self.retired[task['owner']]=pose(request['verified_pose'])
            self.task_board.release(task['task_id'],True,task.get('cancel_requested',False))
            self.persist()
        else:raise ValueError('unsupported task operation')
        return self.task_board.status()

    def report(self):
        missions=self.history['missions'];terminal=[m for m in missions.values() if m['state'] in ('COMPLETED','FAILED')]
        tasks=self.task_board.status()
        return {'schema':'MSC-V1','generated_at':time.time(),'motion_enabled':bool(self.config.get('allow_motion')),
            'measurement_source':self.config.get('measurement_source','unspecified'),
            'mission_count':len(terminal),'mission_completion_rate':sum(m['state']=='COMPLETED' for m in terminal)/len(terminal) if terminal else None,
            'missions':{k:{'state':m['state'],'failure_reason':m.get('failure_reason',''),'current_waypoint':m.get('current_waypoint'),
                           'metrics':m.get('metrics',{})} for k,m in missions.items()},
            'tasks':tasks,'static_only':not self.config.get('allow_motion',False),
            'physical_motion_acceptance':'not performed in static-only deployment' if not self.config.get('allow_motion') else 'requires independently reviewed measurement evidence'}

    def tasks_active(self):
        return any(t['state'] in ('CLAIMED','RUNNING','FINISHING','CANCELING','RECOVERY_REQUIRED') for t in self.task_board.tasks.values())

    def begin_passage(self,states,goal,reason):
        leader=self.mission['leader_id'];followers=[r for r in self.members() if r!=leader]
        if len(followers)!=1:return False
        plan,error=serial_plan(self.observed_grid(states),leader,followers[0],states,self.config['robots'],goal,self.mission['spacing'])
        self.stop_all(reason)
        if plan is None:self.transition('DEGRADED',reason+': '+error);return True
        plan['stage_since']=self.clock();self.mission['maneuver']=plan;self.last_targets.clear()
        self.event('coordinated_passage_started',reason=reason,waiting=plan['waiting'])
        self.transition('RECOVERING',reason);return True

    def complete_passage_waypoint(self):
        index=self.mission['current_waypoint'];leader=self.mission['leader_id']
        if index not in self.mission['completed_waypoints']:
            self.mission['completed_waypoints'].append(index)
            for robot in self.members():self.metrics['waypoint_checkpoints'].setdefault(robot,[]).append(index)
            self.mission['current_waypoint']+=1
        self.persist()
        self.transition('REGROUPING' if self.mission['current_waypoint']>=len(self.mission['waypoints']) else 'EXECUTING')

    def observed_grid(self,states):
        points=[]
        for robot,state in states.items():
            if state.get('obstacle_age',99)+state.get('heartbeat_age',0)>2.5 or not state.get('alignment_verified'):continue
            transform=self.history['alignments'][robot]['transform']
            for point in state.get('obstacles',[]):
                p=compose(transform,[*point,0.])
                # Known robot bodies are scheduled separately as moving actors.
                # Do not remove any occupied cell of the authoritative base grid.
                inside_robot=False
                for other,s in states.items():
                    if s.get('fleet_pose') is None:continue
                    q=compose(inverse(s['fleet_pose']),p);body=self.config['robots'][other]
                    if abs(q[0])<=body.get('length',.5)/2 and abs(q[1])<=body.get('width',.37)/2:inside_robot=True;break
                if not inside_robot:points.append(p[:2])
        return self.grid.with_obstacles(points) if points else self.grid

    def tick_tasks(self):
        now=self.clock()
        if self.stop_request_active or not self.config.get('allow_motion') or now-self.task_tick_at<.5:return
        self.task_tick_at=now
        states={r:self.robot(r) for r in self.config['robots']}
        live=[t for t in self.task_board.tasks.values() if t['state'] in ('CLAIMED','RUNNING','FINISHING')]
        running=[t for t in live if t['state'] in ('RUNNING','FINISHING')]
        selected=min(running or live,key=lambda t:(-t['priority'],t['created_at'],t['task_id']))['task_id'] if live else None
        def stopped(robot):
            s=states[robot]
            command=self.commands.get(robot,{})
            acknowledged=(s.get('control_epoch')==command.get('epoch') and s.get('applied_command_seq')==command.get('seq'))
            return s['online'] and s.get('pose_stationary') and s.get('stopped_seconds',0)>.5 and s.get('velocity_age',99)+s.get('heartbeat_age',0)<.5 and not s.get('nav_active') and not s.get('pending_goal') and acknowledged
        for task in list(self.task_board.tasks.values()):
            robot=task['owner']
            if not robot or task['state'] in ('COMPLETED','CANCELED','FAILED','PENDING'):continue
            state=states[robot];command=self.commands.get(robot,{})
            if task['state'] in ('RECOVERY_REQUIRED','CANCELING'):
                self.issue(robot,'hold',reason='waiting for previous task owner to stop')
                if stopped(robot):self.task_board.release(task['task_id'],True,task['state']=='CANCELING' or task.get('cancel_requested',False));self.issue(robot,'release')
                continue
            if not state.get('fleet_ready') or state.get('goal_error'):
                self.task_board.update(task['task_id'],task['claim'],'RECOVERY_REQUIRED',state.get('goal_error') or state.get('health','not ready'))
                self.issue(robot,'hold',reason='task owner unavailable');continue
            if task['state']=='CLAIMED':
                if task['task_id']!=selected:continue
                if state.get('control_mode')=='FLEET' and stopped(robot) and state.get('control_epoch')==command.get('epoch') and state.get('applied_command_seq')==command.get('seq'):
                    target=compose(inverse(self.history['alignments'][robot]['transform']),task['goal'])
                    extra={'route':[compose(inverse(self.history['alignments'][robot]['transform']),p) for p in task['waypoints']]} if len(task.get('waypoints',[]))>1 else {}
                    self.issue(robot,'navigate',target=target,role='independent_task',task_id=task['task_id'],claim=task['claim'],**extra)
                    task.update(navigation_epoch=self.commands[robot]['epoch'],navigation_seq=self.commands[robot]['seq'])
                    self.task_board.update(task['task_id'],task['claim'],'RUNNING')
            elif task['state']=='RUNNING' and command.get('kind')=='navigate' and state.get('completed_seq')==command['seq']:
                self.issue(robot,'hold',reason='independent task finished; verify stop')
                self.task_board.update(task['task_id'],task['claim'],'FINISHING')
            elif task['state']=='FINISHING' and stopped(robot):
                self.task_board.update(task['task_id'],task['claim'],'COMPLETED');self.issue(robot,'release')
        if self.grid is None or self.history.get('tasks_paused',False) or getattr(self,'ui_start_in_progress',False):return
        def cost(robot,goal):
            radius=self.config['robots'][robot]['radius']+.1
            peers=[(s['fleet_pose'],self.config['robots'][r]['radius']+.1) for r,s in states.items() if r!=robot and s.get('fleet_pose')]
            route=self.grid.route(states[robot]['fleet_pose'],goal,radius,peers)
            return sum(math.dist(a[:2],b[:2]) for a,b in zip(route,route[1:])) if route else None
        for task in self.task_board.assign(states,self.config['robots'],cost):
            self.issue(task['owner'],'claim',task_id=task['task_id'],claim=task['claim'])
