"""Arc-length targets on the observed leader trail; no extrapolated turns."""
import math
from .geometry import angle,pose

class LeaderTrail:
    def __init__(self,seed):
        self.points=[];self.progress={}
        for p in seed:self.append(p)

    def append(self,value):
        p=pose(value)
        if not self.points:self.points.append((0.,p));return
        s,previous=self.points[-1];d=math.dist(previous[:2],p[:2])
        if d<.02:return
        self.points.append((s+d,p))
        while len(self.points)>2 and self.points[-1][0]-self.points[1][0]>40:
            self.points.pop(0)

    def at(self,s):
        if not self.points or s<self.points[0][0]-1e-6 or s>self.points[-1][0]+1e-6:
            raise ValueError('leader trail does not cover requested following distance')
        for (a,p),(b,q) in zip(self.points,self.points[1:]):
            if s<=b:
                t=max(0.,min(1.,(s-a)/(b-a)))
                return [p[0]+t*(q[0]-p[0]),p[1]+t*(q[1]-p[1]),angle(p[2]+t*angle(q[2]-p[2]))]
        return list(self.points[-1][1])

    def follow(self,robot,current,distance):
        end=self.points[-1][0]-distance
        target=self.at(end)
        cursor=self.progress.get(robot,self.points[0][0])
        # Bounded forward projection prevents skipping a loop at a crossing.
        candidates=[cursor];limit=min(end,cursor+1.)
        for (a,p),(b,q) in zip(self.points,self.points[1:]):
            if b<cursor or a>limit:continue
            dx,dy=q[0]-p[0],q[1]-p[1]
            t=((current[0]-p[0])*dx+(current[1]-p[1])*dy)/(dx*dx+dy*dy)
            candidates.append(max(cursor,min(limit,a+max(0.,min(1.,t))*(b-a))))
        cursor=min(candidates,key=lambda s:math.dist(current[:2],self.at(s)[:2]))
        self.progress[robot]=cursor
        route=[self.at(cursor)]+[p for s,p in self.points if cursor<s<end]+[target]
        if len(route)>100:raise ValueError('follower lag exceeds bounded trail request')
        return target,route
