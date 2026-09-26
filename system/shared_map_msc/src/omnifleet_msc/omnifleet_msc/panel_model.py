"""Operator-selected fleet vehicles and routes. Configuration never starts motion."""
import copy,math,uuid
from .geometry import pose,compose
from .storage import read,save

class FleetPanelModel:
    def __init__(self,core,path):
        self.core=core;self.path=path;self.revision=0;self.previews={};self.last_error=''
        defaults={'active':[],'selected':None,'mode':'independent','leader':None,'spacing':1.4,
                  'names':{r:r for r in core.config['robots']},'routes':{r:[] for r in core.config['robots']}}
        self.config=read(path,defaults)
        self.validate(self.config)
        self.core.ui_participants=list(self.config['active'])

    def busy(self):
        return bool(self.core.tasks_active() or (self.core.mission and self.core.mission['state'] not in ('IDLE','COMPLETED','FAILED')))

    def robot_busy(self,robot):
        if self.core.mission and self.core.mission['state'] not in ('IDLE','COMPLETED','FAILED') and robot in self.core.mission['members']:return True
        return any((t.get('owner') or t.get('robot_id'))==robot and t['state'] not in ('COMPLETED','CANCELED','FAILED') for t in self.core.task_board.tasks.values())

    def validate(self,c):
        known=set(self.core.config['robots']);active=c['active']
        if not isinstance(active,list) or len(active)>2 or len(set(active))!=len(active) or not set(active)<=known:
            raise ValueError('本部署支持已登记的两辆车，车辆编号不能重复')
        if c['mode'] not in ('independent','leader'):raise ValueError('未知协同模式')
        if c['selected'] is not None and c['selected'] not in active:raise ValueError('请选择已激活车辆')
        if active and c['leader'] not in active:raise ValueError('领航车必须已激活')
        if not math.isfinite(c['spacing']) or not 1.4<=c['spacing']<=5:raise ValueError('跟随距离范围为1.4～5米')
        for robot in known:
            name=c['names'].get(robot,robot)
            if not isinstance(name,str) or not name.strip() or len(name)>40 or any(ord(ch)<32 for ch in name):raise ValueError('车辆名称须为1～40个可见字符')
            route=c['routes'].get(robot,[])
            if not isinstance(route,list) or len(route)>100:raise ValueError('每车最多100个航点')
            for value in route:pose(value)

    def configure(self,request):
        changing=set(request)-{'id','op','revision','created_at'}
        if self.busy() and changing-{'selected','routes'}:raise ValueError('任务执行或取消中，请先停止后修改车辆编组')
        if 'routes' in request:
            for r,v in request['routes'].items():
                if v!=self.config['routes'].get(r) and self.robot_busy(r):raise ValueError(r+'任务执行中，不能改动其路线')
        if request.get('revision')!=self.revision:raise ValueError('配置已变化，请按最新状态重试')
        c=copy.deepcopy(self.config)
        for key in ('active','selected','mode','leader','spacing','names','routes'):
            if key in request:c[key]=copy.deepcopy(request[key])
        if 'active' in request:
            if c['selected'] not in c['active']:c['selected']=next(iter(c['active']),None)
            if c['leader'] not in c['active']:c['leader']=next(iter(c['active']),None)
        c['spacing']=float(c['spacing']);self.validate(c)
        self.config=c;self.core.ui_participants=list(c['active']);self.revision+=1;self.previews={};save(self.path,c)
        return '车辆和路线设置已保存；未发送运动目标'

    def add_point(self,value):
        robot=self.config['selected']
        if robot not in self.config['active']:raise ValueError('先激活并选中车辆')
        if self.config['mode']=='leader' and robot!=self.config['leader']:raise ValueError('领航模式请选中领航车编辑路线')
        routes=copy.deepcopy(self.config['routes']);routes[robot].append(pose(value))
        return self.configure({'revision':self.revision,'routes':routes})

    def preview(self,robot):
        revision=self.revision
        if robot not in self.config['active']:raise ValueError('车辆未激活')
        state=self.core.robot(robot);start=state.get('fleet_pose');grid=self.core.grid
        if start is None and robot==self.core.config['reference_robot'] and state.get('pose_age',99)<.8:
            start=state.get('local_pose')
        origin=copy.deepcopy(start)
        if start is None or grid is None:raise ValueError('等待车辆在共享地图中定位，以及共享地图数据')
        points=self.config['routes'][robot]
        if not points:raise ValueError('先添加航点')
        states={r:self.core.robot(r) for r in self.core.config['robots']}
        grid=self.core.observed_grid(states);result=[];radius=self.core.config['robots'][robot]['radius']+.1
        peers=[(s['fleet_pose'],self.core.config['robots'][r]['radius']+.1) for r,s in states.items() if r!=robot and s.get('fleet_pose')]
        for index,target in enumerate(points,1):
            segment=grid.route(start,target,radius,peers)
            if not segment:raise ValueError(f'航点{index}不可达或完整车体通道不足')
            result.extend(segment if not result else segment[1:]);start=target
        if revision!=self.revision:raise ValueError('路线已改变，丢弃过期预检结果')
        self.previews[robot]={'revision':revision,'points':result,'at':self.core.clock(),'start':origin}
        return f'{self.config["names"][robot]}：{len(points)}个航点已通过共享地图预检'

    def start(self,request):
        if request.get('revision')!=self.revision:raise ValueError('路线已变化，请重新确认')
        if not self.core.config.get('allow_motion'):raise ValueError('当前部署处于不运动验收模式，可编辑和预检，不能发车')
        if self.config['mode']=='leader' and self.busy():raise ValueError('已有协同任务，请先停止或等待完成')
        active=self.config['active']
        if not active:raise ValueError('未激活车辆')
        robots=[self.config['leader']] if self.config['mode']=='leader' else [request.get('robot_id',self.config['selected'])]
        required=active if self.config['mode']=='leader' else robots
        for robot in required:
            if robot not in active:raise ValueError('车辆未激活')
            if robot in robots and self.robot_busy(robot):raise ValueError(robot+'已有任务，请先停止或等待完成')
            state=self.core.robot(robot)
            if not state.get('fleet_ready'):raise ValueError(robot+' 尚未满足协同启动条件')
            if not state.get('supports_route'):raise ValueError(robot+' 的代理需要更新多点任务接口')
        for robot in robots:
            preview=self.previews.get(robot,{})
            if preview.get('revision')!=self.revision or self.core.clock()-preview.get('at',-1e9)>5:
                raise ValueError('请重新预检最新路线后开始')
            if math.dist(preview['start'][:2],self.core.robot(robot)['fleet_pose'][:2])>.25:
                raise ValueError('车辆已偏离预检起点，请重新预检')
        if self.config['mode']=='leader':
            members=[self.config['leader']]+[r for r in active if r!=self.config['leader']]
            return self.core.operator({'op':'start','mission_id':'foxglove-'+request['id'],'members':members,'leader_id':members[0],
                'formation':'column','follow_model':'trail','spacing':self.config['spacing'],'frame_id':'fleet_map','waypoints':self.config['routes'][members[0]]})
        robot=robots[0];points=self.config['routes'][robot]
        return self.core.task_operator({'op':'submit','task_id':'foxglove-'+request['id'],'frame_id':'fleet_map','robot_id':robot,'goal':points[-1],'waypoints':points})

    def command(self,request):
        op=request.get('op')
        if op=='configure':return self.configure(request)
        if op=='add':return self.add_point(request['pose'])
        if op=='preview':return self.preview(request.get('robot_id',self.config['selected']))
        if op=='start':return self.start(request)
        if op=='stop':
            # Existing coordinator stop semantics cancel owned actions and retain
            # recovery requirements; a lost vehicle is never silently made LOCAL.
            if self.config['mode']=='leader':return self.core.operator({'op':'cancel'})
            robot=self.config['selected'];results=[]
            for id_,task in list(self.core.task_board.tasks.items()):
                if (task.get('owner') or task.get('robot_id'))==robot and task['state'] not in ('COMPLETED','CANCELED','FAILED'):
                    results.append(self.core.task_operator({'op':'cancel','task_id':id_}))
            return results
        raise ValueError('未知面板操作')

    def state(self):
        robots=[];c=self.config;order=([c['leader']]+[r for r in c['active'] if r!=c['leader']]) if c['active'] else []
        for robot,body in self.core.config['robots'].items():
            s=self.core.robot(robot)
            display_pose=s.get('fleet_pose')
            # fleet_map is explicitly the reference robot's map frame. Showing
            # that robot does not assert that another robot is aligned to it.
            if display_pose is None and robot==self.core.config['reference_robot'] and s.get('pose_age',99)<.8:
                display_pose=s.get('local_pose')
            robots.append({'id':robot,'name':c['names'][robot],'active':robot in c['active'],
                'rank':order.index(robot)+1 if robot in order else None,'online':s['online'],
                'pose':display_pose,'local_pose_3d':s.get('local_pose_3d'),'pose_age':s.get('pose_age',99),'health':s.get('health','OFFLINE'),
                'ready':s.get('fleet_ready',False),'busy':self.robot_busy(robot),'supports_route':s.get('supports_route',False),
                'reason':('本地导航已接管，需显式交还协同' if s.get('local_override') else s.get('goal_error') or s.get('safety_stop_reason') or ''),
                'body':{k:body.get(k,v) for k,v in [('length',.5),('width',.37)]},
                'planned_path':s.get('planned_path',[]),'alignment':self.core.history['alignments'].get(robot,{}).get('transform')})
        return {'version':1,'revision':self.revision,'config':c,'robots':robots,'busy':self.busy(),
                'motion_enabled':bool(self.core.config.get('allow_motion')),'previews':self.previews,
                'mission':self.core.status()['mission'],'tasks':{k:{f:t.get(f) for f in ('state','owner','robot_id','reason')} for k,t in list(self.core.task_board.tasks.items())[-20:]}}
