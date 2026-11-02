"""Global A* on an inflated, known-free SLAM grid; no simulator access."""
from dataclasses import dataclass
import heapq
import math

import cv2
import numpy as np

from polyline_core import Frame2D


@dataclass
class ReturnPlan:
    points: list
    expanded: int
    length: float
    reason: str


class ReturnGrid:
    def __init__(self, cells, resolution, origin, yaw=0., clearance=.20,
                 preferred_clearance=.30, wall_cost=4., observation=None):
        self.cells=np.array(cells,dtype=np.int16,copy=True)
        self.resolution=float(resolution)
        self.frame=Frame2D(np.asarray(origin,dtype=float),yaw)
        self.clearance=clearance;self.preferred_clearance=preferred_clearance
        self.wall_cost=wall_cost
        if observation is not None:self.observe(observation)
        known=(self.cells>=0)&(self.cells<65)
        # Unknown cell centres remain forbidden. Inflate measured obstacles,
        # rather than treating the SLAM crop boundary as a physical wall.
        # The entrance and just-observed exterior can lie at that boundary.
        self.distance=cv2.distanceTransform((self.cells<65).astype(np.uint8),
                                             cv2.DIST_L2,cv2.DIST_MASK_PRECISE)*resolution
        self.safe=known&(self.distance>=clearance)

    def observe(self, view):
        """Extend only the return-planning layer with current measured rays.

        SLAM may crop the exterior just beside the stopped robot. Unknown
        cells traversed by a current ray can become free; mapped obstacles
        are never erased and cells beyond a hit remain unknown.
        """
        scan=view.scan;local_range=min(1.5,scan.range_max)
        padding=int(math.ceil(local_range/self.resolution))+2
        self.cells=np.pad(self.cells,padding,constant_values=-1)
        self.frame=Frame2D(self.frame.apply([-padding*self.resolution]*2),self.frame.yaw)
        raw=np.asarray(scan.ranges,dtype=float)
        valid=np.isposinf(raw)|(np.isfinite(raw)&(raw>=scan.range_min)&(raw<=scan.range_max))
        angles=scan.angle_min+np.arange(len(raw))*scan.angle_increment+view.yaw
        unit=np.column_stack((np.cos(angles),np.sin(angles)))
        limits=np.minimum(np.where(np.isposinf(raw),scan.range_max,raw),local_range)
        steps=np.arange(0.,local_range,self.resolution*.45)
        free=valid[:,None]&(steps[None,:]<limits[:,None]-.04)
        points=view.origin+steps[None,:,None]*unit[:,None,:]
        def cells_for(points):
            c,s=math.cos(self.frame.yaw),math.sin(self.frame.yaw)
            delta=points-self.frame.translation
            xy=np.floor(np.column_stack((c*delta[:,0]+s*delta[:,1],-s*delta[:,0]+c*delta[:,1]))/self.resolution).astype(int)
            inside=(xy[:,0]>=0)&(xy[:,0]<self.cells.shape[1])&(xy[:,1]>=0)&(xy[:,1]<self.cells.shape[0])
            return xy[inside]
        xy=cells_for(points[free]);old=self.cells[xy[:,1],xy[:,0]]
        self.cells[xy[:,1],xy[:,0]]=np.where(old<0,0,old)
        hits=valid&np.isfinite(raw)&(raw<local_range)
        xy=cells_for(view.origin+raw[hits,None]*unit[hits])
        self.cells[xy[:,1],xy[:,0]]=100

    def cell(self, point):
        col,row=np.floor(self.frame.inverse(point)/self.resolution).astype(int)
        return int(row),int(col)

    def point(self, cell):
        row,col=cell
        return self.frame.apply((np.array([col,row])+.5)*self.resolution)

    def allowed(self, cell):
        row,col=cell
        return 0<=row<self.safe.shape[0] and 0<=col<self.safe.shape[1] and bool(self.safe[cell])

    def line_safe(self, start, goal):
        count=max(2,int(math.ceil(np.linalg.norm(np.asarray(goal)-start)/(self.resolution*.4)))+1)
        previous=None
        for point in np.linspace(start,goal,count):
            cell=self.cell(point)
            if not self.allowed(cell):return False
            if previous is not None and cell[0]!=previous[0] and cell[1]!=previous[1]:
                if not self.allowed((previous[0],cell[1])) or not self.allowed((cell[0],previous[1])):
                    return False
            previous=cell
        return True

    def plan(self, start, goal, max_expansions=250000, extra_cost=None):
        if extra_cost is not None and (extra_cost.shape!=self.safe.shape or np.any(extra_cost<0)):
            raise ValueError('A* extra_cost must match the grid and be nonnegative')
        start,goal=np.asarray(start,dtype=float),np.asarray(goal,dtype=float)
        source,target=self.cell(start),self.cell(goal)
        if not self.allowed(source):return ReturnPlan([],0,0.,'start is outside inflated known-free map')
        if not self.allowed(target):return ReturnPlan([],0,0.,'home is outside inflated known-free map')
        def heuristic(cell):
            dr,dc=abs(target[0]-cell[0]),abs(target[1]-cell[1])
            return max(dr,dc)+(math.sqrt(2)-1)*min(dr,dc)
        queue=[(heuristic(source),0.,source)];costs={source:0.};parent={};expanded=0
        offsets=[(dr,dc) for dr in (-1,0,1) for dc in (-1,0,1) if dr or dc]
        while queue and expanded<max_expansions:
            _,cost,current=heapq.heappop(queue)
            if cost>costs[current]+1e-9:continue
            if current==target:
                cells=[current]
                while current!=source:
                    current=parent[current];cells.append(current)
                ordered=list(reversed(cells))
                # Do not force a detour through the source/goal cell centres;
                # their exact poses can lie at a cell edge. This also avoids
                # putting a tiny backwards segment after the last A* cell.
                points=[start]+[self.point(c) for c in ordered[1:-1]]+[goal]
                # Keep A*'s clearance-biased turns. Remove only collinear cells,
                # never string-pull a new shortcut against an inflated wall.
                path=[points[0]]
                for i in range(1,len(points)-1):
                    a,b=points[i]-path[-1],points[i+1]-points[i]
                    if np.linalg.norm(a)<1e-7:continue
                    if abs(float(a[0]*b[1]-a[1]*b[0]))<1e-9 and float(a@b)>=0:continue
                    path.append(points[i])
                if np.linalg.norm(goal-path[-1])>1e-7:path.append(goal)
                if len(path)==1:path.append(goal.copy())
                if not all(self.line_safe(a,b) for a,b in zip(path[:-1],path[1:])):
                    return ReturnPlan([],expanded,0.,'endpoint connector is not safe')
                length=sum(float(np.linalg.norm(b-a)) for a,b in zip(path[:-1],path[1:]))
                return ReturnPlan(path,expanded,length,'A* route through inflated known-free map')
            expanded+=1
            for dr,dc in offsets:
                nxt=current[0]+dr,current[1]+dc
                if not self.allowed(nxt):continue
                if dr and dc and (not self.allowed((current[0]+dr,current[1])) or
                                  not self.allowed((current[0],current[1]+dc))):continue
                penalty=self.wall_cost*max(0.,1.-float(self.distance[nxt])/self.preferred_clearance)**2
                if extra_cost is not None:penalty+=float(extra_cost[nxt])
                candidate=cost+math.hypot(dr,dc)*(1.+penalty)
                if candidate+1e-9<costs.get(nxt,float('inf')):
                    costs[nxt]=candidate;parent[nxt]=current
                    heapq.heappush(queue,(candidate+heuristic(nxt),candidate,nxt))
        reason='A* expansion limit reached' if queue else 'no known-free route connects exit and home'
        return ReturnPlan([],expanded,0.,reason)
