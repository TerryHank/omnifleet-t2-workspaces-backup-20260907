"""Conservative grid checks for formation envelopes and collision-free waiting routes."""
import heapq,math
import numpy as np
from .geometry import compose,inverse,angle,offsets

def distance_to_path(point,path):
    if not path:return float('inf')
    result=math.dist(point[:2],path[0][:2])
    for a,b in zip(path,path[1:]):
        dx,dy=b[0]-a[0],b[1]-a[1];length=dx*dx+dy*dy
        t=max(0.,min(1.,((point[0]-a[0])*dx+(point[1]-a[1])*dy)/length)) if length else 0.
        result=min(result,math.hypot(point[0]-a[0]-t*dx,point[1]-a[1]-t*dy))
    return result

class FleetGrid:
    def __init__(self,width,height,resolution,origin,data):
        if width*height!=len(data) or resolution<=0:raise ValueError('invalid planning grid')
        self.width=width;self.height=height;self.res=resolution;self.origin=origin
        cells=np.asarray(data).reshape(height,width);self.blocked=(cells<0)|(cells>=100)
        self.masks={}
    def cell(self,p):
        x,y,_=compose(inverse(self.origin),[*p[:2],0.]);return math.floor(x/self.res),math.floor(y/self.res)
    def with_obstacles(self,points):
        result=FleetGrid(self.width,self.height,self.res,self.origin,self.blocked.astype(np.int8).reshape(-1)*100)
        for p in points:
            x,y=result.cell(p)
            if 0<=x<result.width and 0<=y<result.height:result.blocked[y,x]=True
        return result
    def point(self,cell):return compose(self.origin,[(cell[0]+.5)*self.res,(cell[1]+.5)*self.res,0.])
    def mask(self,radius):
        key=math.ceil(radius/self.res*100)/100
        if key not in self.masks:
            margin=math.ceil(radius/self.res+.71);pad=np.pad(self.blocked,margin,constant_values=True);mask=np.zeros_like(self.blocked)
            for dy in range(-margin,margin+1):
                for dx in range(-margin,margin+1):
                    if math.hypot(dx,dy)*self.res<=radius+self.res*.71:
                        mask |= pad[margin+dy:margin+dy+self.height,margin+dx:margin+dx+self.width]
            self.masks[key]=mask
        return self.masks[key]
    def safe(self,p,radius,peers=()):
        x,y=self.cell(p)
        return bool(0<=x<self.width and 0<=y<self.height and not self.mask(radius)[y,x] and
                    all(math.dist(p[:2],other[:2])>=radius+r for other,r in peers))
    def segment(self,a,b,radius,peers=()):
        count=max(1,math.ceil(math.dist(a[:2],b[:2])/(self.res*.5)))
        return all(self.safe([a[j]+(b[j]-a[j])*i/count for j in range(2)],radius,peers) for i in range(count+1))
    def route(self,start,goal,radius,peers=(),limit=60000):
        if not self.safe(start,radius,peers) or not self.safe(goal,radius,peers):return []
        first,last=self.cell(start),self.cell(goal)
        frontier=[(0.,first)];cost={first:0.};parent={};expanded=0
        while frontier and expanded<limit:
            _,current=heapq.heappop(frontier);expanded+=1
            if current==last:
                cells=[current]
                while current!=first:current=parent[current];cells.append(current)
                points=[self.point(c) for c in reversed(cells)]
                if len(points)==1:points=[list(start),list(goal)]
                else:points[0]=list(start);points[-1]=list(goal)
                # Remove raster stair-steps only where the full disk swept segment is clear.
                simplified=[points[0]];index=0
                while index<len(points)-1:
                    next_index=len(points)-1
                    while next_index>index+1 and not self.segment(points[index],points[next_index],radius,peers):next_index-=1
                    simplified.append(points[next_index]);index=next_index
                points=simplified
                for i in range(len(points)-1):points[i][2]=math.atan2(points[i+1][1]-points[i][1],points[i+1][0]-points[i][0])
                return [list(start)]+points
            for dx,dy in ((1,0),(-1,0),(0,1),(0,-1),(1,1),(1,-1),(-1,1),(-1,-1)):
                nxt=(current[0]+dx,current[1]+dy);p=self.point(nxt)
                if not self.safe(p,radius,peers):continue
                if dx and dy and (not self.safe(self.point((current[0]+dx,current[1])),radius,peers) or not self.safe(self.point((current[0],current[1]+dy)),radius,peers)):continue
                new=cost[current]+math.hypot(dx,dy)
                if new<cost.get(nxt,float('inf')):
                    cost[nxt]=new;parent[nxt]=current;heapq.heappush(frontier,(new+math.dist(nxt,last),nxt))
        return []
    def formation_fits(self,path,kind,spacing,radii,clearance):
        off=offsets(kind,spacing,len(radii))
        previous=None
        for p in path:
            positions=[compose(p,d) for d in off]
            if not all(self.safe(pos,r+clearance) for pos,r in zip(positions,radii)):return False
            if previous:
                if not all(self.segment(a,b,r+clearance) for a,b,r in zip(previous,positions,radii)):return False
                # Sample heading sweep as well as translation, avoiding corner cuts during turns.
                delta=angle(p[2]-previous_pose[2]);count=max(1,math.ceil(abs(delta)/.08))
                for j in range(1,count):
                    mid=[previous_pose[0]+(p[0]-previous_pose[0])*j/count,previous_pose[1]+(p[1]-previous_pose[1])*j/count,previous_pose[2]+delta*j/count]
                    if not all(self.safe(compose(mid,d),r+clearance) for d,r in zip(off,radii)):return False
            previous=positions;previous_pose=p
        return bool(path)
    def waiting_point(self,start,passing_path,radius,peers=(),max_distance=3.):
        candidates=[];cx,cy=self.cell(start);span=math.ceil(max_distance/self.res)
        for y in range(max(0,cy-span),min(self.height,cy+span+1),2):
            for x in range(max(0,cx-span),min(self.width,cx+span+1),2):
                p=self.point((x,y));distance=math.dist(p[:2],start[:2])
                if .35<distance<=max_distance and self.safe(p,radius+.15,peers) and distance_to_path(p,passing_path)>=2*radius+.15:
                    candidates.append((distance,p))
        for _,point in sorted(candidates,key=lambda item:item[0])[:80]:
            if self.route(start,point,radius,peers):return point
        return None
