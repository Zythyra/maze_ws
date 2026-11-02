"""Topology-constrained A*, accepted cubic B-splines and dual-curvature speeds.

Algorithm reference: Zythyra/ifly2_2026, rk3588, my_planner C2/C5,
commit 4c876c9e551c4ad161344b9efa5a625f48a981e3 (BSD package).
This ROS-independent differential-drive adaptation omits active drift and vy.
"""
from dataclasses import dataclass
import math
import time

import cv2
import numpy as np
from scipy.interpolate import BSpline
from scipy.ndimage import median_filter, maximum_filter1d

from return_planner import ReturnGrid


def clean_path(points):
    points=np.asarray(points,dtype=float).reshape(-1,2)
    if not len(points):return points
    return points[np.r_[True,np.linalg.norm(np.diff(points,axis=0),axis=1)>.003]]


def arc_length(points):
    return np.r_[0.,np.cumsum(np.linalg.norm(np.diff(points,axis=0),axis=1))]


def interpolate(points, s, query):
    return np.stack([np.interp(query,s,points[:,i]) for i in range(2)],axis=-1)


def resample(points, step=.02):
    points=clean_path(points)
    if len(points)<2:return points
    s=arc_length(points)
    query=np.r_[np.arange(0.,s[-1],step),s[-1]]
    return clean_path(interpolate(points,s,query))


def project(points, position):
    points=np.asarray(points);a=points[:-1];delta=np.diff(points,axis=0)
    squared=np.sum(delta*delta,axis=1)
    fraction=np.clip(np.sum((position-a)*delta,axis=1)/np.maximum(squared,1e-12),0.,1.)
    projections=a+fraction[:,None]*delta
    index=int(np.argmin(np.linalg.norm(projections-position,axis=1)))
    s=arc_length(points)
    return float(s[index]+fraction[index]*math.sqrt(squared[index])),projections[index]


@dataclass
class MotionReference:
    xy: np.ndarray
    speed_max: float
    stop_at_end: bool
    lateral_accel: float = .9
    acceleration: float = .8
    deceleration: float = .9
    omega_limit: float = 3.5
    wheel_limit: float = .95

    def __post_init__(self):
        self.xy=resample(self.xy,.02);self.s=arc_length(self.xy)
        # C2: a physical chord, not adjacent cell headings.
        before=interpolate(self.xy,self.s,np.maximum(0.,self.s-.06))
        after=interpolate(self.xy,self.s,np.minimum(self.s[-1],self.s+.06))
        tangent=after-before
        self.yaw=np.unwrap(np.arctan2(tangent[:,1],tangent[:,0]))
        def curvature(distance):
            lo=np.maximum(0.,self.s-distance);hi=np.minimum(self.s[-1],self.s+distance)
            return (np.interp(hi,self.s,self.yaw)-np.interp(lo,self.s,self.yaw))/np.maximum(hi-lo,1e-6)
        self.k_track=curvature(.06)
        self.k_speed=median_filter(curvature(.12),size=5,mode='nearest')
        # Keep a short hold after a bend; avoid lifting a minimum speed above
        # the actual curvature or wheel feasibility limit.
        envelope=maximum_filter1d(np.abs(self.k_speed),size=9,mode='nearest')
        speed=np.minimum(self.speed_max,np.sqrt(self.lateral_accel/np.maximum(envelope,1e-6)))
        speed=np.minimum(speed,self.omega_limit/np.maximum(np.abs(self.k_track),1e-6))
        speed=np.minimum(speed,self.wheel_limit/(1.+.08*np.abs(self.k_track)))
        if self.stop_at_end:speed[-1]=0.
        for i in range(len(speed)-2,-1,-1):
            speed[i]=min(speed[i],math.sqrt(speed[i+1]**2+2*self.deceleration*(self.s[i+1]-self.s[i])))
        for i in range(1,len(speed)):
            speed[i]=min(speed[i],math.sqrt(speed[i-1]**2+2*self.acceleration*(self.s[i]-self.s[i-1])))
        self.speed=speed

    def sample(self, query):
        return (interpolate(self.xy,self.s,query),np.interp(query,self.s,self.yaw),
                np.interp(query,self.s,self.k_track),np.interp(query,self.s,self.speed))

    def horizon(self, position, measured_speed, count=20, dt=.05):
        current,_=project(self.xy,np.asarray(position))
        locations=[];velocities=[];velocity=max(0.,measured_speed)
        for _ in range(count+1):
            cap=float(np.interp(current,self.s,self.speed))
            velocity=min(cap,max(0.,velocity+self.acceleration*dt))
            locations.append(current);velocities.append(velocity)
            current=min(self.s[-1],current+velocity*dt)
        xy,heading,k,_=self.sample(np.array(locations))
        velocity=np.asarray(velocities)
        # No fabricated continuation beyond a measured/terminal path endpoint.
        if self.stop_at_end:
            velocity=np.where(np.array(locations)>=self.s[-1]-.002,0.,velocity)
        return np.column_stack((xy,heading,velocity,k*velocity))


class ReferenceBuilder:
    def __init__(self, clearance=.18, tube=.18, forward_window=1.6, period=.35,
                 lateral_accel=.9):
        self.clearance=clearance;self.tube=tube;self.forward_window=forward_window
        self.period=period;self.lateral_accel=lateral_accel
        self.map_data=None;self.reference=None;self.grid=None
        self.last_build=-float('inf');self.raw=None;self.dirty=True
        self.stop_request=None
        self.report=dict(status='WAIT_MAP',build_ms=0.,astar_expanded=0)

    def set_map(self,cells,resolution,origin,yaw=0.):
        self.map_data=(np.asarray(cells).copy(),resolution,np.asarray(origin).copy(),yaw)

    def invalidate(self):
        self.dirty=True

    def build(self, raw, position, speed, stop_at_end=False, observation=None):
        started=time.monotonic();raw=clean_path(raw);position=np.asarray(position)
        if len(raw)<2 or self.map_data is None:
            self.report.update(status='WAIT_MAP_OR_PATH');return None
        changed=(self.raw is None or self.raw.shape!=raw.shape or
                 float(np.max(np.linalg.norm(self.raw-raw,axis=1)))>.04)
        if (not changed and not self.dirty and started-self.last_build<self.period
                and self.reference is not None and self.stop_request==stop_at_end
                and abs(self.reference.speed_max-speed)<.015):
            return self.reference
        self.last_build=started;self.dirty=False;self.raw=raw.copy()
        self.stop_request=stop_at_end
        lengths=arc_length(raw);progress,_=project(raw,position)
        lo=max(0.,progress-.22);hi=min(lengths[-1],progress+self.forward_window)
        queries=np.r_[lo,lengths[(lengths>lo)&(lengths<hi)],hi]
        window=clean_path(interpolate(raw,lengths,queries))
        if len(window)<2 or hi-lo<.04:return self.reference
        cells,resolution,origin,yaw=self.map_data
        grid=ReturnGrid(cells,resolution,origin,yaw,clearance=self.clearance,
                        wall_cost=5.,observation=observation)
        # A* may improve geometry inside the selected road, never choose an
        # alternative maze branch. Rasterize its allowed topology corridor.
        allowed=np.zeros_like(grid.safe,dtype=np.uint8)
        centre=np.ones_like(allowed)
        for a,b in zip(window[:-1],window[1:]):
            cv2.line(allowed,grid.cell(a)[::-1],grid.cell(b)[::-1],1,
                     thickness=max(1,int(2*self.tube/resolution)))
            cv2.line(centre,grid.cell(a)[::-1],grid.cell(b)[::-1],0,thickness=1)
        grid.safe &= allowed.astype(bool)
        offset=cv2.distanceTransform(centre,cv2.DIST_L2,cv2.DIST_MASK_PRECISE)*resolution
        extra_cost=3.*(offset/self.tube)**2
        plan=grid.plan(window[0],window[-1],max_expansions=12000,extra_cost=extra_cost)
        clipped=False
        if not plan.points:
            # C5: an occluded/map-edge tail shortens the observable window;
            # it is not a wall, and does not invalidate the whole approach.
            dense=resample(window,.08);distance=arc_length(dense)
            for index in range(len(dense)-2,1,-1):
                if distance[index]<.35:break
                if not grid.allowed(grid.cell(dense[index])):continue
                candidate=grid.plan(window[0],dense[index],max_expansions=4000,extra_cost=extra_cost)
                if candidate.points:
                    plan=candidate;clipped=True;break
        if plan.points:seed=np.asarray(plan.points);status='ASTAR'
        elif all(grid.line_safe(a,b) for a,b in zip(window[:-1],window[1:])):
            seed=window;status='RAW_SAFE'
        else:
            # HOLD is allowed only for the same endpoint/direction, and only
            # if the complete previously accepted path is still valid now.
            old=self.reference
            if (not changed and old is not None and
                    all(grid.line_safe(a,b) for a,b in zip(old.xy[:-1],old.xy[1:]))):
                self.grid=grid;self.report.update(status='HOLD_VALID',build_ms=(time.monotonic()-started)*1000)
                return old
            self.reference=None;self.report.update(status='NO_KNOWN_CORRIDOR',
                reason=plan.reason,build_ms=(time.monotonic()-started)*1000)
            return None
        selected=resample(seed);smoothed=False
        # C5 candidate/active separation: a clamped cubic B-spline is accepted
        # only after checking all sampled chords against known inflated space.
        # Each attempt starts from the raw A* seed; smoothing is not cumulative.
        for spacing in (.34,.26,.18,.10):
            controls=resample(seed,spacing);n=len(controls)
            if n<4:continue
            knots=np.r_[np.zeros(4),np.arange(1,n-3)/(n-3),np.ones(4)]
            curve=BSpline(knots,controls,3)
            candidate=resample(curve(np.linspace(0.,1.,max(40,int(arc_length(seed)[-1]/.008)))))
            if all(grid.line_safe(a,b) for a,b in zip(candidate[:-1],candidate[1:])):
                selected=candidate;smoothed=True;break
        committed=False
        old=self.reference
        if old is not None and len(selected)>3:
            new_s=arc_length(selected);new_progress,new_point=project(selected,position)
            old_progress,old_point=project(old.xy,position)
            new_forward=interpolate(selected,new_s,min(new_s[-1],new_progress+.40))-new_point
            old_forward=interpolate(old.xy,old.s,min(old.s[-1],old_progress+.40))-old_point
            agreement=float(new_forward@old_forward/max(np.linalg.norm(new_forward)*np.linalg.norm(old_forward),1e-9))
            if agreement>.65 and np.linalg.norm(new_point-old_point)<.10:
                # C5 commitment/start blending prevents a raw segment switch
                # from moving the curve underneath the robot. Never retain an
                # old turn across a changed branch or a failed safety check.
                matched=old_progress+new_s-new_progress
                prior=interpolate(old.xy,old.s,matched)
                t=np.clip((new_s-new_progress)/.30,0.,1.)
                weight=(1.-3*t*t+2*t*t*t)
                weight[(matched<0.)|(matched>old.s[-1])]=0.
                blended=resample(weight[:,None]*prior+(1-weight[:,None])*selected)
                if all(grid.line_safe(a,b) for a,b in zip(blended[:-1],blended[1:])):
                    selected=blended;committed=True
        terminal=bool(clipped or (stop_at_end and hi>=lengths[-1]-.01))
        self.reference=MotionReference(selected,speed,terminal,self.lateral_accel)
        self.grid=grid
        self.report=dict(status=status+('_BSPLINE' if smoothed else '_VALIDATED_POLYLINE'),
                         build_ms=(time.monotonic()-started)*1000,astar_expanded=plan.expanded,
                         max_curvature=float(np.max(np.abs(self.reference.k_track))),
                         minimum_speed=float(np.min(self.reference.speed)),points=len(selected),
                         observable_window_clipped=clipped,committed_prefix=committed)
        return self.reference
