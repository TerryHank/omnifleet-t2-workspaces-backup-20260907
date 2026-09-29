"""Two-robot yielding plan: stop, move one robot aside, pass, then regroup."""
import math
from .geometry import compose
from .planning import distance_to_path

def serial_plan(grid,leader,follower,states,config,goal,spacing,clearance=.1):
    a=states[leader]['fleet_pose'];b=states[follower]['fleet_pose']
    ar=config[leader]['radius']+clearance;br=config[follower]['radius']+clearance
    route=grid.route(a,goal,ar)
    if not route:return None,'leader has no footprint-safe route'
    regroup=compose(goal,[-spacing,0.,0.])
    if not grid.safe(regroup,br,[(goal,ar)]):return None,'no safe column regrouping point at destination'
    waiting=list(b)
    if distance_to_path(b,route)<ar+br+.1:
        waiting=grid.waiting_point(b,route,br,[(a,ar)])
        if waiting is None:return None,'no reachable waiting point outside passing route'
    if not grid.route(waiting,regroup,br,[(goal,ar)]):return None,'waiting robot cannot safely rejoin after passage'
    return {'stage':'STOP','leader':leader,'follower':follower,'waiting':waiting,'goal':list(goal),
            'regroup':regroup,'route':route,'move_to_wait':math.dist(waiting[:2],b[:2])>.2},''

def step(core,states):
    m=core.mission.get('maneuver')
    if not m:return False
    leader=m['leader'];follower=m['follower'];stage=m['stage']
    if core.clock()-m['stage_since']>120:
        core.stop_all('coordinated passage timeout');core.transition('DEGRADED','coordinated passage timeout');return True
    def advance(next_stage):
        m['stage']=next_stage;m['stage_since']=core.clock();core.last_targets.clear()
        core.event('maneuver_stage',stage=next_stage);core.persist()
    def finished(robot):
        c=core.commands.get(robot,{})
        return c.get('kind')=='navigate' and states[robot].get('completed_seq')==c.get('seq')
    if stage=='STOP':
        core.stop_all('preparing coordinated passage')
        if core.all_stopped():advance('YIELD' if m['move_to_wait'] else 'PASS')
    elif stage=='YIELD':
        core.issue(leader,'hold',reason='waiting for yielding robot')
        if finished(follower):core.issue(follower,'hold',reason='waiting position reached');advance('WAIT_STOP')
        else:core.target(follower,m['waiting'],'yield')
    elif stage=='WAIT_STOP':
        if core.all_stopped():advance('PASS')
    elif stage=='PASS':
        core.issue(follower,'hold',reason='exclusive passage for leader')
        if finished(leader):core.issue(leader,'hold',reason='passage complete');advance('PASS_STOP')
        else:core.target(leader,m['goal'],'passage')
    elif stage=='PASS_STOP':
        if core.all_stopped():advance('REJOIN')
    elif stage=='REJOIN':
        core.issue(leader,'hold',reason='waiting for follower to rejoin')
        if finished(follower):core.issue(follower,'hold',reason='rejoined');advance('FINAL_STOP')
        else:core.target(follower,m['regroup'],'rejoin')
    elif stage=='FINAL_STOP' and core.all_stopped():
        core.mission.pop('maneuver');core.mission['formation']='column';core.last_targets.clear()
        core.traffic.waiting.clear();core.traffic.owners.clear();core.metrics['automatic_recoveries']=core.metrics.get('automatic_recoveries',0)+1
        core.event('coordinated_passage_complete');core.complete_passage_waypoint()
    return True
