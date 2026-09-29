"""Conservative path conflict and configured narrow-region ownership."""
import math
from .geometry import compose

def sample_path(position,path,speed,horizon=3.,step=.2):
    points=[list(position[:2])]
    if path:
        nearest=min(range(len(path)),key=lambda i:math.dist(position[:2],path[i][:2]))
        points+=path[nearest:]
    if len(points)==1:return [points[0]]*(int(horizon/step)+1)
    result=[]
    for i in range(int(horizon/step)+1):
        remaining=speed*i*step;value=points[-1]
        for a,b in zip(points,points[1:]):
            length=math.dist(a,b)
            if remaining<=length and length>1e-8:
                value=[a[j]+(b[j]-a[j])*remaining/length for j in range(2)];break
            remaining-=length
        result.append(value)
    return result

def in_zone(position,zone,radius=0.):
    x0,y0,x1,y1=zone['bounds']
    return x0-radius<=position[0]<=x1+radius and y0-radius<=position[1]<=y1+radius

class Traffic:
    def __init__(self):
        self.owners={};self.waiting={};self.last_conflicts=set();self.conflict_count=0;self.deadlock_count=0
    def evaluate(self,states,alignments,config,leader,now,zones,clearance=.2):
        held={};predictions={}
        for robot,state in states.items():
            path=state.get('planned_path',[]) if state.get('nav_active') else []
            transform=alignments[robot]['transform']
            path=[compose(transform,[p[0],p[1],0])[:2] for p in path]
            predictions[robot]=sample_path(state['fleet_pose'],path,.4 if path else 0.)
        for zone in zones:
            zone_id=zone['id'];owner=self.owners.get(zone_id)
            inside=[r for r,s in states.items() if in_zone(s['fleet_pose'],zone,config[r]['radius'])]
            requesters=[r for r,path in predictions.items() if any(in_zone(p,zone,config[r]['radius']) for p in path)]
            if owner not in states or (owner not in inside and owner not in requesters):
                self.owners.pop(zone_id,None);owner=None
            if owner is None and (inside or requesters):
                candidates=inside or requesters
                owner=leader if leader in candidates else sorted(candidates)[0];self.owners[zone_id]=owner
            for robot in requesters:
                if robot!=owner:held[robot]='waiting for region '+zone_id
        robots=list(states);conflicts=set()
        if len(robots)==2:
            a,b=robots;minimum=config[a]['radius']+config[b]['radius']+clearance
            if any(math.dist(pa,pb)<minimum for pa,pb in zip(predictions[a],predictions[b])):
                pair=tuple(sorted(robots));conflicts.add(pair)
                follower=b if a==leader else a
                held[follower]='yielding to leader path'
                if any(math.dist(p,states[follower]['fleet_pose'][:2])<minimum for p in predictions[leader]):
                    held[leader]='leader path blocked by waiting robot'
        self.conflict_count+=len(conflicts-self.last_conflicts);self.last_conflicts=conflicts
        for robot in list(self.waiting):
            if robot not in held:self.waiting.pop(robot)
        for robot in held:self.waiting.setdefault(robot,now)
        deadlocked=len(held)==len(states) and held and all(now-self.waiting[r]>8 for r in held)
        if deadlocked:
            self.deadlock_count+=1
            for robot in held:self.waiting[robot]=now
        return held,bool(deadlocked)
