"""Persistent map road graph, left-first exploration, and pure-pursuit geometry."""
from dataclasses import dataclass, field
import math

import numpy as np


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def direction(index):
    return np.array([(1.,0.),(0.,1.),(-1.,0.),(0.,-1.)][index % 4])


@dataclass
class Frame2D:
    translation: np.ndarray
    yaw: float

    def apply(self, point):
        c,s=math.cos(self.yaw),math.sin(self.yaw)
        x,y=point
        return np.asarray(self.translation)+[c*x-s*y,s*x+c*y]

    def inverse(self, point):
        x,y=np.asarray(point)-self.translation;c,s=math.cos(self.yaw),math.sin(self.yaw)
        return np.array([c*x+s*y,-s*x+c*y])


class ExitDetector:
    """Latch completion after leaving a corridor for sustained open space.

    Uses only fresh, unique, full-circle scans and accumulated odometry travel.
    Positive infinity is a valid no-return; malformed/missing rays are not.
    This is a local open-space criterion, not a world-boundary oracle.
    """
    def __init__(self, clearance=1., confirm_time=.8, min_travel=1., max_scan_gap=.35,
                 required_open_sides=3, sector_half_angle_deg=10., min_open_arc_deg=180.):
        self.clearance=clearance;self.confirm_time=confirm_time
        self.min_travel=min_travel;self.max_scan_gap=max_scan_gap
        self.required_open_sides=required_open_sides
        self.sector_half_angle=math.radians(sector_half_angle_deg)
        self.min_open_arc_deg=min_open_arc_deg
        self.open_sides=0;self.open_arc_deg=0.
        self.corridor_distance=None;self.armed=False;self.reached=False
        self.clear_since=None;self.last_stamp=None;self.confirmed_at=None
        self.sectors={};self.scan_valid=False

    def reset_candidate(self):
        self.clear_since=None

    def update(self, scan, distance, stamp):
        if self.reached:return True
        if self.last_stamp is not None:
            if stamp==self.last_stamp:return False
            if stamp<self.last_stamp or stamp-self.last_stamp>self.max_scan_gap:
                self.reset_candidate()
        self.last_stamp=stamp
        raw=np.asarray(scan.ranges,dtype=float);n=len(raw)
        increment=scan.angle_increment
        self.sectors={};self.scan_valid=False;self.open_sides=0;self.open_arc_deg=0.
        if (n<32 or not math.isfinite(increment) or increment==0 or
                not math.isfinite(scan.angle_min) or
                not math.isfinite(scan.range_min) or scan.range_min<0 or
                not math.isfinite(scan.range_max) or scan.range_max<=self.clearance or
                scan.range_max<=scan.range_min or
                (n-1)*abs(increment)<2*math.pi-2*abs(increment)):
            self.reset_candidate();return False
        angles=scan.angle_min+np.arange(n)*increment
        valid=np.isposinf(raw)|(np.isfinite(raw)&(raw>=scan.range_min)&(raw<=scan.range_max))
        for index,name in enumerate(('front','left','back','right')):
            delta=angles-index*math.pi/2
            mask=np.abs(np.arctan2(np.sin(delta),np.cos(delta)))<=self.sector_half_angle+1e-9
            count=int(np.count_nonzero(mask))
            if count<3 or np.count_nonzero(valid&mask)<.9*count:
                self.reset_candidate();return False
            self.sectors[name]=float(np.min(np.minimum(raw[mask&valid],scan.range_max)))
        self.scan_valid=True
        self.open_sides=sum(value>=self.clearance for value in self.sectors.values())
        # Three corridor mouths at a T/cross junction can look open along the
        # cardinal rays. An exterior beside one wall also has an uninterrupted
        # free half-circle; junction corner walls break that arc into short runs.
        clear=valid&(np.minimum(raw,scan.range_max)>=self.clearance)
        longest=run=0
        for opened in np.concatenate((clear,clear)):
            run=run+1 if opened else 0
            longest=max(longest,run)
        self.open_arc_deg=min(360.,math.degrees(min(longest,n)*abs(increment)))
        # A stationary startup in an empty world must never count as an exit.
        if self.corridor_distance is None and (
                self.sectors['left']<min(.75,self.clearance*.8) and
                self.sectors['right']<min(.75,self.clearance*.8)):
            self.corridor_distance=distance
        self.armed=(self.corridor_distance is not None and
                    distance-self.corridor_distance>=self.min_travel)
        if (not self.armed or self.open_sides<self.required_open_sides or
                self.open_arc_deg<self.min_open_arc_deg):
            self.reset_candidate();return False
        if self.clear_since is None:self.clear_since=stamp
        if stamp-self.clear_since>=self.confirm_time-1e-9:
            self.reached=True;self.confirmed_at=stamp
        return self.reached

    def report(self):
        return dict(armed=self.armed,scan_valid=self.scan_valid,
                    candidate=self.clear_since is not None,
                    clear_duration_s=0. if self.clear_since is None else self.last_stamp-self.clear_since,
                    sector_min_m=self.sectors,open_sides=self.open_sides,
                    required_open_sides=self.required_open_sides,open_arc_deg=self.open_arc_deg)


class ScanView:
    def __init__(self, scan, origin, yaw, clearance=.30):
        self.origin, self.yaw, self.scan = np.asarray(origin), yaw, scan
        self.clearance = clearance
        raw = np.asarray(scan.ranges, dtype=float)
        self.ranges = np.where(np.isposinf(raw), scan.range_max, raw)
        angles = scan.angle_min + np.arange(len(raw))*scan.angle_increment + yaw
        valid = np.isfinite(raw) & (raw >= scan.range_min) & (raw < scan.range_max)
        self.points = self.origin + np.column_stack((np.cos(angles[valid]),np.sin(angles[valid])))*raw[valid,None]

    def clear(self, point, margin=.025):
        delta = np.asarray(point)-self.origin
        distance = float(np.linalg.norm(delta))
        if distance < self.scan.range_min:
            return True
        if distance > self.scan.range_max-.03:
            return None
        angle = wrap(math.atan2(delta[1],delta[0])-self.yaw)
        # Respect both 0..2pi and -pi..pi scan layouts, without assuming 360 rays.
        relative = (angle-self.scan.angle_min) % (2*math.pi)
        index = int(round(relative/self.scan.angle_increment))
        if index >= len(self.ranges):
            index = 0 if relative > 2*math.pi-2*self.scan.angle_increment else len(self.ranges)-1
        indices = [(index+i) % len(self.ranges) for i in (-1,0,1)]
        values = self.ranges[indices]
        values = values[np.isfinite(values) & (values >= self.scan.range_min)]
        if not len(values): return None
        return bool(np.percentile(values,35) >= distance+margin)

    def line_end(self, anchor, heading, position, clearance):
        axis = direction(heading); normal = direction(heading+1)
        anchor = np.asarray(anchor)
        progress = max(0.,float((position-anchor) @ axis))
        local = self.points-anchor
        along, lateral = local @ axis, local @ normal
        hits = np.sort(along[(along > progress+.10) & (np.abs(lateral)<.055)])
        if len(hits):
            group = hits[hits < hits[0]+.07]
            wall = float(np.median(group))
            before = anchor+axis*max(progress+.03,wall-.12)
            if self.clear(before,0.) is True:
                return anchor+axis*max(.035,wall-clearance), True
        # An unseen far wall cannot be fabricated. Extend only the observed ray.
        visible = progress
        for along in np.arange(progress+.05,progress+3.3,.05):
            if self.clear(anchor+axis*along,0.) is not True: break
            visible = along
        return anchor+axis*max(progress+.04,visible-clearance), False

    def opening(self, anchor, heading, position, endpoint, clearance, side=1):
        axis, normal = direction(heading), direction(heading+side)
        progress = float((position-anchor) @ axis)
        end = float((endpoint-anchor) @ axis)
        samples = np.arange(max(-.65,progress-.7),min(progress+1.45,end+.55),.025)
        values = [self.clear(anchor+axis*s+normal*(clearance+.025),.12) for s in samples]
        runs = []; start = None
        for i, opened in enumerate(values+[False]):
            if opened is True and start is None: start=i
            if opened is not True and start is not None:
                left_edge=start>0 and values[start-1] is False
                right_edge=i<len(values) and opened is False
                if left_edge or right_edge:
                    width = samples[i-1]-samples[start]+.025
                    full=left_edge and right_edge
                    if (full and .38<width<1.05) or (not full and width>.18):
                        guess=(samples[start]+samples[i-1])/2 if full else (
                            samples[start]+clearance if left_edge else samples[i-1]-clearance)
                        point=self.corridor_center(anchor+axis*guess,heading+side,clearance)
                        if point is not None:
                            center=float((point-anchor) @ axis)
                            # The previous corner is not another side opening.
                            if max(progress+.08,.32)<center<end-.10 and self.clear(point+normal*.52,.02) is True:
                                runs.append((center,point,width))
                start=None
        return min(runs,key=lambda item:item[0]) if runs else None

    def right_opening(self, anchor, heading, position, endpoint, clearance):
        return self.opening(anchor,heading,position,endpoint,clearance,-1)

    def corridor_center(self, anchor, heading, clearance):
        """Offset a measured side wall by clearance; never invent a far edge."""
        anchor=np.asarray(anchor);axis,normal=direction(heading),direction(heading+1)
        relative=self.points-anchor;along=relative @ axis;lateral=relative @ normal
        visible=(along>.16)&(along<1.25)&(np.abs(lateral)>.10)&(np.abs(lateral)<.62)
        indices=np.flatnonzero(visible)
        if len(indices)<4:return None
        bins=np.round(lateral[indices]/.025).astype(int)
        candidates=[]
        for bucket in np.unique(bins):
            group=indices[np.abs(lateral[indices]-bucket*.025)<.035]
            if len(group)<4 or np.ptp(along[group])<.16:continue
            wall=float(np.median(lateral[group]))
            shift=wall-math.copysign(clearance,wall)
            if abs(shift)>.24:continue
            point=anchor+normal*shift
            free=[self.clear(point+axis*d,.015) for d in (.12,.24,.36)]
            if any(value is False for value in free) or sum(value is True for value in free)<2:continue
            score=len(group)*min(float(np.ptp(along[group])),.8)
            candidates.append((score,point))
        return max(candidates,key=lambda item:item[0])[1] if candidates else None

    def closing_wall(self, point, heading):
        """Require a measured wall across the mouth, not an occluded ray.

        A corner can block rays to a perfectly open branch. Only a continuous
        wall perpendicular to that branch, spanning its centre on both sides,
        proves closure. Missing coverage remains unknown.
        """
        relative=self.points-np.asarray(point)
        along=relative @ direction(heading);lateral=relative @ direction(heading+1)
        width=min(.16,self.clearance*.55)
        selected=(along>.10)&(along<self.clearance+.15)&(np.abs(lateral)<width+.05)
        x,y=along[selected],lateral[selected]
        if len(x)<6:return False
        for bucket in np.unique(np.round(x/.025).astype(int)):
            group=np.abs(x-bucket*.025)<.045
            xx,yy=x[group],y[group]
            # All three parts must be seen. Two jambs around an unseen/open
            # middle are not a closing wall either.
            if any(np.count_nonzero((yy>=low)&(yy<high))<2
                   for low,high in ((-width,-.05),(-.05,.05),(.05,width))):continue
            slope,offset=np.polyfit(yy,xx,1)
            if abs(slope)<.35 and np.std(xx-(slope*yy+offset))<.025:return True
        return False

    def ports(self, point):
        result = {}
        for index in range(4):
            samples = [self.clear(point+direction(index)*d,.015) for d in (.18,.34,.52)]
            if all(value is True for value in samples): result[index]='open'
            elif any(value is False for value in samples) and self.closing_wall(point,index):
                result[index]='blocked'
            else: result[index]='unknown'
        return result


@dataclass
class GraphAnchor:
    first: int
    second: int | None
    local: np.ndarray


class PoseGraphAnchors:
    """Local rigid attachments to stable SLAM keyframe IDs, including loop updates."""
    def __init__(self):
        self.poses = {}
        self._pose_cache=None

    def frame(self, first, second):
        origin = self.poses[first]
        angle = 0.
        if second is not None:
            delta = self.poses[second]-origin
            angle = math.atan2(delta[1], delta[0])
        c,s = math.cos(angle),math.sin(angle)
        return origin,np.array([[c,-s],[s,c]])

    def attach(self, point):
        if not self.poses:return None
        if self._pose_cache is not self.poses:
            self._pose_cache=self.poses
            self._keys=np.array(list(self.poses),dtype=int)
            self._positions=np.array(list(self.poses.values()))
        delta=self._positions-point
        distances=delta[:,0]**2+delta[:,1]**2
        index=int(np.argmin(distances));first=int(self._keys[index])
        delta=self._positions-self._positions[index]
        squared=delta[:,0]**2+delta[:,1]**2
        nearby=np.flatnonzero((np.abs(self._keys-first)<=12)&(squared>.20**2)&(squared<.85**2))
        second=int(self._keys[nearby[np.argmin(np.abs(self._keys[nearby]-first))]]) if len(nearby) else None
        origin,rotation = self.frame(first,second)
        return GraphAnchor(first,second,rotation.T @ (np.asarray(point)-origin))

    def resolve(self, anchor, fallback):
        if anchor is None or anchor.first not in self.poses:return fallback
        if anchor.second is not None and anchor.second not in self.poses:return fallback
        origin,rotation=self.frame(anchor.first,anchor.second)
        return origin+rotation @ anchor.local


@dataclass
class Junction:
    point: np.ndarray
    parent: int | None
    ports: dict = field(default_factory=dict)
    neighbors: dict = field(default_factory=dict)
    branches: dict = field(default_factory=dict)
    anchor: GraphAnchor | None = None


@dataclass
class Leg:
    source: int
    heading: int
    end: np.ndarray
    terminal: bool
    destination: int | None = None
    returning: bool = False
    split: bool = False
    edge_id: int = -1
    visited: bool = False
    anchor: GraphAnchor | None = None
    retired: bool = False
    map_dead_end: bool = False
    map_dead_end_cells: int = 0
    map_check_reason: str = ''


class Explorer:
    """Persistent two-sided road graph with left-first exploration."""
    def __init__(self, clearance=.30, preview=.48, tolerance=.065, merge_radius=.22):
        self.clearance,self.preview,self.tolerance=clearance,preview,tolerance
        self.merge_radius=merge_radius
        self.nodes=[];self.edges=[];self.leg=self.next_leg=None
        self.return_route=[];self.return_target=None;self.last_return_edges=[]
        self.backtracks=self.turns=self.openings=0
        self.return_discoveries=0
        self.state='WAIT_SCAN';self.reason=''
        self.gap_candidate=None;self.gap_count=0
        self.right_gap_candidate=None;self.right_gap_count=0
        self.side_junctions=0
        self.right_openings=0
        self.slam=PoseGraphAnchors();self.map_revisions=0;self.wall_revisions=0
        self.branch_map=None;self.branch_map_cache={}
        self.forward_map_cache={};self.forward_dead_cuts={}
        self.map_forward_turnbacks=0;self.last_map_turnback=None

    def set_branch_map(self, branch_map):
        self.branch_map=branch_map;self.branch_map_cache={}
        self.forward_map_cache={}
        if branch_map is not None:
            for point,_,heading in self.forward_dead_cuts.values():
                branch_map.remember(branch_map.check(point,heading))
        for edge in self.edges:self.check_mapped_branch(edge)

    def return_from_mapped_dead_end(self, position, heading):
        """Reverse the active road now, without first chasing its unused tip."""
        leg=self.leg
        if self.branch_map is None or leg is None or leg.returning:return False
        if heading is not None and abs(wrap(heading-leg.heading*math.pi/2))>.35:return False
        start=self.nodes[leg.source].point;axis=direction(leg.heading)
        vector=leg.end-start
        fraction=float(np.clip((position-start) @ vector/max(float(vector @ vector),1e-9),0.,1.))
        projection=start+fraction*vector
        if np.linalg.norm(position-projection)>.16:return False
        key=(leg.edge_id,float(start[0]),float(start[1]),leg.heading)
        if key not in self.forward_map_cache:
            self.forward_map_cache[key]=self.branch_map.check(start,leg.heading)
        proof=self.forward_map_cache[key];cut=start.copy();mode='road entrance'
        if not proof.dead_end:
            # The already passed part of a long road can still border unknown
            # space. Prove just the component ahead of the current projection.
            cut=projection-axis*.38;mode='current projection'
            proof=self.branch_map.check(cut,leg.heading)
        if not proof.dead_end:return False
        self.forward_dead_cuts[leg.edge_id]=(cut.copy(),self.slam.attach(cut),leg.heading)
        self.branch_map.remember(proof);self.branch_map_cache={}
        # A prefetched turn is only reserved, not yet entered. Release it so
        # the region proof can exclude its junction from frontier selection.
        for pending in [self.next_leg]+self.return_route:
            if pending is not None and not pending.returning and pending.edge_id!=leg.edge_id:
                self.edges[pending.edge_id].visited=False
                self.nodes[pending.source].ports[pending.heading]='open'
        self.next_leg=None;self.return_route=[]
        if leg.destination is None:self.bind_destination()
        for edge in self.edges:self.check_mapped_branch(edge)
        reverse=Leg(leg.destination,(leg.heading+2)%4,start.copy(),True,leg.source,True,
                    edge_id=leg.edge_id,visited=True)
        if self.untried(leg.source):
            target=leg.source
            route=[self.select_branch(leg.source,reverse.heading)]
        else:
            target,route=self.frontier_route(leg.source)
            if target is None:target=leg.source
        self.backtracks+=1;self.map_forward_turnbacks+=1
        self.last_map_turnback=dict(edge_id=leg.edge_id,position=position.tolist(),
            unused_tip=leg.end.tolist(),distance_to_tip_m=float(np.linalg.norm(leg.end-position)),
            proof_cells=proof.cells,proof_origin=mode,return_target=target)
        self.leg=reverse;self.return_route=route
        self.next_leg=route[0] if route else None;self.return_target=target
        self.last_return_edges=[leg.edge_id]+[item.edge_id for item in route if item.returning]
        self.gap_candidate=self.right_gap_candidate=None;self.gap_count=self.right_gap_count=0
        self.state='BACKTRACK';self.reason='map proves road ahead is closed; return from current position'
        return True

    def check_mapped_branch(self, edge):
        # Keep graph IDs and actual visit flags. A map proof may be withdrawn
        # by a later SLAM correction or a newly observed opening.
        if self.branch_map is None or edge.visited or edge.retired or edge.destination is not None:
            edge.map_dead_end=False;edge.map_dead_end_cells=0;edge.map_check_reason=''
            self.branch_map_cache.pop(edge.edge_id,None)
            return
        point=self.nodes[edge.source].point
        key=(float(point[0]),float(point[1]),edge.heading)
        if self.branch_map_cache.get(edge.edge_id)==key:return
        proof=self.branch_map.check(point,edge.heading)
        edge.map_dead_end=proof.dead_end;edge.map_dead_end_cells=proof.cells
        edge.map_check_reason=proof.reason
        self.branch_map_cache[edge.edge_id]=key

    def align_to_walls(self, walls, is_free):
        """Refit stored geometry to observed map walls without changing IDs.

        Each straight wall contributes a normal/offset constraint to both road
        endpoints. Shared junctions solve all incident constraints together.
        This also allows small local wall rotations after SLAM correction.
        """
        walls=np.asarray(walls)
        if len(walls)<8:return
        constraints={i:[] for i in range(len(self.nodes))};tips={}
        for edge in self.edges:
            if edge.retired or not edge.visited:continue
            a=self.nodes[edge.source].point;b=edge.end;vector=b-a
            length=float(np.linalg.norm(vector))
            if length<.4:continue
            axis=vector/length;normal=np.array([-axis[1],axis[0]])
            local=walls-a
            along=local[:,0]*axis[0]+local[:,1]*axis[1]
            lateral=local[:,0]*normal[0]+local[:,1]*normal[1];fits=[]
            for side in (-1,1):
                selected=(along>.08)&(along<length-.08)&(side*lateral>.06)&(side*lateral<.60)
                x,y=along[selected],lateral[selected]
                medians=[]
                for low,high in zip(np.linspace(.08,length-.08,6)[:-1],np.linspace(.08,length-.08,6)[1:]):
                    mask=(x>=low)&(x<high)
                    if np.count_nonzero(mask)<2:continue
                    xx,yy=x[mask],y[mask]
                    # Use the visible face, rather than the back of a thick wall.
                    near=side*yy<np.min(side*yy)+.045
                    medians.append((float(np.median(xx[near])),float(np.median(yy[near]))))
                if len(medians)<3:continue
                mx,my=np.array(medians).T
                if np.ptp(mx)<.20:continue
                slope,intercept=np.polyfit(mx,my,1)
                if abs(slope)>.35 or np.std(my-(slope*mx+intercept))>.025:continue
                scale=math.sqrt(1+slope*slope);n=(normal-slope*axis)/scale
                offset=float(n @ a)+intercept/scale-side*self.clearance
                fits.append((len(medians),n,offset))
            if not fits:continue
            if len(fits)==2 and abs(fits[0][2]-fits[1][2])>.12:
                fits=[max(fits,key=lambda item:item[0])]
            for _,n,offset in fits:
                constraints[edge.source].append((n,offset))
                if edge.destination is not None:constraints[edge.destination].append((n,offset))
                else:tips.setdefault(edge.edge_id,[]).append((n,offset))

        def corrected(point,conditions):
            if not conditions:return point
            # The prior preserves unconstrained coordinates along a corridor.
            matrix=np.array([n for n,_ in conditions]+[[.15,0.],[0.,.15]])
            rhs=np.array([offset for _,offset in conditions]+list(.15*point))
            target=np.linalg.lstsq(matrix,rhs,rcond=None)[0]
            delta=target-point;length=float(np.linalg.norm(delta))
            if length>.04:delta*=.04/length
            proposed=point+delta
            return proposed if is_free(proposed) else point

        moves={};changed=False
        for i,node in enumerate(self.nodes):
            point=corrected(node.point,constraints[i]);moves[i]=point-node.point
            if np.linalg.norm(moves[i])>.001:
                node.point=point;node.anchor=self.slam.attach(point);changed=True
        for edge in self.edges:
            if edge.retired:continue
            if edge.destination is not None:edge.end=self.nodes[edge.destination].point.copy()
            else:
                if edge.visited:point=corrected(edge.end,tips.get(edge.edge_id,[]))
                else:point=edge.end+moves.get(edge.source,np.zeros(2))
                if np.linalg.norm(point-edge.end)>.001:
                    edge.end=point;edge.anchor=self.slam.attach(point);changed=True
        self.wall_revisions+=int(changed)
        self.sync_route()

    def update_pose_graph(self, poses):
        if not poses:return
        self.slam.poses={key:np.asarray(value,dtype=float) for key,value in poses.items()}
        for edge_id,(point,anchor,heading) in list(self.forward_dead_cuts.items()):
            point=self.slam.resolve(anchor,point).copy()
            if anchor is None or anchor.second is None:anchor=self.slam.attach(point)
            self.forward_dead_cuts[edge_id]=(point,anchor,heading)
        changed=False
        for node in self.nodes:
            new=self.slam.resolve(node.anchor,node.point)
            changed |= np.linalg.norm(new-node.point)>.001
            node.point=new.copy()
            if node.anchor is None or node.anchor.second is None:
                node.anchor=self.slam.attach(node.point)
        for edge in self.edges:
            if edge.retired:continue
            if edge.destination is not None:edge.end=self.nodes[edge.destination].point.copy()
            else:
                new=self.slam.resolve(edge.anchor,edge.end)
                changed |= np.linalg.norm(new-edge.end)>.001
                edge.end=new.copy()
                if edge.anchor is None or edge.anchor.second is None:
                    edge.anchor=self.slam.attach(edge.end)
        self.map_revisions+=int(changed)
        self.sync_route()

    def new_edge(self, source, heading, end, terminal):
        heading%=4
        existing=self.nodes[source].branches.get(heading)
        if existing is not None:return self.edges[existing]
        # Reaching the other end of a saved ray reuses its undirected road ID.
        for saved in self.edges:
            if saved.retired or saved.destination is not None or saved.source==source:continue
            if (saved.heading+2)%4!=heading:continue
            if np.linalg.norm(saved.end-self.nodes[source].point)>=self.merge_radius:continue
            saved.destination=source;saved.end=self.nodes[source].point.copy();saved.terminal=True
            self.nodes[saved.source].neighbors[saved.heading]=source
            self.nodes[source].neighbors[heading]=saved.source
            self.nodes[source].branches[heading]=saved.edge_id
            if saved.visited:self.nodes[source].ports[heading]='tried'
            return saved
        edge=Leg(source,heading,np.asarray(end).copy(),terminal,edge_id=len(self.edges))
        edge.anchor=self.slam.attach(edge.end)
        self.edges.append(edge);self.nodes[source].branches[heading]=edge.edge_id
        return edge

    def begin(self, view, position, heading=None):
        position=np.asarray(position).copy()
        initial=0 if heading is None else int(round(heading/(math.pi/2)))%4
        self.nodes=[Junction(position,None,{i:('tried' if i==initial else 'blocked') for i in range(4)},
                             anchor=self.slam.attach(position))]
        end,terminal=view.line_end(position,initial,position,self.clearance)
        self.leg=self.new_edge(0,initial,end,terminal);self.leg.visited=True
        self.state='TRACK_PATH'

    def vertex(self, point, parent):
        for i,node in enumerate(self.nodes):
            if i!=parent and np.linalg.norm(node.point-point)<self.merge_radius:return i
        self.nodes.append(Junction(np.asarray(point).copy(),parent,anchor=self.slam.attach(point)))
        return len(self.nodes)-1

    def sync_leg(self, leg):
        if leg is None:return
        edge=self.edges[leg.edge_id]
        if leg is edge:
            if edge.destination is not None:edge.end=self.nodes[edge.destination].point.copy()
            return
        target=edge.source if leg.source==edge.destination else edge.destination
        if target is not None:leg.end=self.nodes[target].point.copy();leg.destination=target

    def sync_route(self):
        for leg in [self.leg,self.next_leg]+self.return_route:self.sync_leg(leg)

    def bind_destination(self):
        leg=self.leg
        if leg.destination is not None:return
        leg.destination=self.vertex(leg.end,leg.source)
        leg.end=self.nodes[leg.destination].point.copy();leg.anchor=None
        source,node=self.nodes[leg.source],self.nodes[leg.destination]
        other_id=node.branches.get((leg.heading+2)%4)
        if other_id is not None and other_id!=leg.edge_id:
            other=self.edges[other_id]
            if not other.visited:
                other.retired=True
                for known in self.nodes:
                    for port,edge_id in list(known.branches.items()):
                        if edge_id==other_id:known.branches[port]=leg.edge_id
        source.neighbors[leg.heading]=leg.destination
        node.neighbors[(leg.heading+2)%4]=leg.source
        node.branches[(leg.heading+2)%4]=leg.edge_id
        node.ports[(leg.heading+2)%4]='tried'
        if leg.split:
            node.ports[leg.heading]='open';node.ports[(leg.heading+1)%4]='open'

    def observe_branches(self, view, node_id):
        node=self.nodes[node_id]
        for index,status in view.ports(node.point).items():
            if node.ports.get(index)=='tried':continue
            if index in node.branches:
                # A later occluded scan cannot erase a confirmed static road.
                node.ports[index]='tried' if self.edges[node.branches[index]].visited else 'open'
                continue
            # A cached blocked result is not current visibility evidence after
            # moving or SLAM corrections. Saved road IDs above remain intact.
            node.ports[index]=status
        # Generate every visible direction now, even if left is selected first.
        for index in range(4):
            if node.ports.get(index)!='open' or index in node.branches:continue
            end,terminal=view.line_end(node.point,index,node.point,self.clearance)
            if np.linalg.norm(end-node.point)>=.14:self.new_edge(node_id,index,end,terminal)

    def untried(self, node_id):
        node=self.nodes[node_id]
        for edge_id in node.branches.values():self.check_mapped_branch(self.edges[edge_id])
        # Preserve connected roads for transit, but do not use a node wholly
        # inside a proven pocket as a new exploration target.
        if self.branch_map is not None and self.branch_map.enclosing_proof(node.point) is not None:return []
        return [index for index,edge_id in node.branches.items()
                if node.ports.get(index)=='open' and not self.edges[edge_id].visited
                and not self.edges[edge_id].retired and not self.edges[edge_id].map_dead_end]

    def outgoing(self, node_id, heading):
        edge=self.edges[self.nodes[node_id].branches[heading]]
        if edge.source==node_id:return edge
        return Leg(node_id,heading,self.nodes[edge.source].point.copy(),True,
                   edge.source,False,edge_id=edge.edge_id,visited=edge.visited)

    def select_branch(self, node_id, incoming):
        node=self.nodes[node_id]
        for offset in (1,0,-1,2):
            index=(incoming+offset)%4
            if index not in self.untried(node_id):continue
            edge=self.outgoing(node_id,index)
            edge.visited=True;self.edges[edge.edge_id].visited=True
            node.ports[index]='tried';self.turns+=int(index!=incoming)
            return edge
        return None

    def frontier_route(self, start):
        """Shortest known route to a junction that still has a saved branch."""
        import heapq
        queue=[(0.,start,[])];best={start:0.}
        while queue:
            cost,node_id,path=heapq.heappop(queue)
            if cost>best[node_id]+1e-9:continue
            if node_id!=start and self.untried(node_id):return node_id,path
            for heading,target in self.nodes[node_id].neighbors.items():
                edge_id=self.nodes[node_id].branches[heading];edge=self.edges[edge_id]
                if not edge.visited:continue
                length=float(np.linalg.norm(self.nodes[target].point-self.nodes[node_id].point))
                distance=cost+length
                if distance>=best.get(target,float('inf'))-1e-9:continue
                best[target]=distance
                leg=Leg(node_id,heading,self.nodes[target].point.copy(),True,target,True,
                        edge_id=edge_id,visited=True)
                heapq.heappush(queue,(distance,target,path+[leg]))
        return None,[]

    def start_return(self, node_id):
        self.gap_candidate=None;self.gap_count=0
        self.right_gap_candidate=None;self.right_gap_count=0
        target,route=self.frontier_route(node_id)
        if not route:
            self.leg=self.next_leg=None;self.return_route=[]
            self.state='EXPLORATION_COMPLETE';self.reason='all saved branches explored';return
        self.backtracks+=1;self.return_target=target
        self.last_return_edges=[leg.edge_id for leg in route]
        self.leg=route[0];self.return_route=route[1:]
        self.next_leg=self.return_route[0] if self.return_route else None
        self.state='BACKTRACK';self.reason='one continuous saved route to nearest unexplored junction'

    def enter_return_left(self, node_id, selected):
        # A branch reserved at the old return target has not been entered yet.
        # Release it before replacing the remainder of that return route.
        for pending in self.return_route:
            if not pending.returning:
                self.edges[pending.edge_id].visited=False
                self.nodes[pending.source].ports[pending.heading]='open'
        selected.visited=True;self.edges[selected.edge_id].visited=True
        self.nodes[node_id].ports[selected.heading]='tried'
        self.return_route=[selected];self.next_leg=selected
        self.return_target=None;self.last_return_edges=[]
        self.gap_candidate=None;self.gap_count=0
        self.right_gap_candidate=None;self.right_gap_count=0
        self.return_discoveries+=1;self.turns+=1
        self.state='TRACK_PATH';self.reason='prefer unexplored left branch during return'

    def split_current_road(self, point):
        """Insert one crossing into an existing road, retaining its old ID.

        The old edge remains source -> crossing; one new edge represents the
        remaining crossing -> destination part. No parallel road is drawn.
        """
        leg=self.leg;edge=self.edges[leg.edge_id]
        a,b=edge.source,edge.destination
        old_end=edge.end.copy();old_terminal=edge.terminal
        node_id=self.vertex(point,a)
        if node_id in (a,b):return None
        node=self.nodes[node_id];forward=edge.heading;reverse=(forward+2)%4
        if forward in node.branches or reverse in node.branches:return None
        edge.destination=node_id;edge.end=node.point.copy();edge.anchor=None;edge.terminal=True
        self.nodes[a].neighbors[forward]=node_id
        node.neighbors[reverse]=a;node.branches[reverse]=edge.edge_id
        continuation=Leg(node_id,forward,old_end,old_terminal,b,
                         edge_id=len(self.edges),visited=leg.returning)
        self.edges.append(continuation);node.branches[forward]=continuation.edge_id
        if not leg.returning:
            edge.visited=leg.source==a
            continuation.visited=leg.source!=a
        node.ports[reverse]='tried' if edge.visited else 'open'
        self.nodes[a].ports[forward]='tried' if edge.visited else 'open'
        node.ports[forward]='tried' if continuation.visited else 'open'
        if b is not None:
            node.neighbors[forward]=b
            self.nodes[b].neighbors[reverse]=node_id
            self.nodes[b].branches[reverse]=continuation.edge_id
            self.nodes[b].ports[reverse]='tried' if continuation.visited else 'open'
        approach_id=edge.edge_id if leg.source==a else continuation.edge_id
        self.leg=Leg(leg.source,leg.heading,node.point.copy(),True,node_id,leg.returning,
                     edge_id=approach_id,visited=True)
        if leg.returning:
            remaining=self.outgoing(node_id,leg.heading)
            remaining=Leg(node_id,leg.heading,remaining.end.copy(),remaining.terminal,
                          leg.destination,True,edge_id=remaining.edge_id,visited=True)
            self.return_route.insert(0,remaining);self.next_leg=remaining
            self.last_return_edges=[self.leg.edge_id]+[r.edge_id for r in self.return_route if r.returning]
        else:
            if self.next_leg is not None:
                pending=self.next_leg
                self.edges[pending.edge_id].visited=False
                self.nodes[pending.source].ports[pending.heading]='open'
            self.next_leg=None
        self.side_junctions+=1
        return node_id

    def record_side_openings(self, view, position, heading):
        """Register both sides before deciding which branch to drive into."""
        leg=self.leg
        if heading is not None and abs(wrap(heading-leg.heading*math.pi/2))>=.30:
            self.gap_candidate=self.right_gap_candidate=None
            self.gap_count=self.right_gap_count=0
            return set()
        start=self.nodes[leg.source].point;endpoint=leg.end.copy()
        confirmed=[]
        for side,prefix,detector in ((1,'',view.opening),
                                    (-1,'right_',getattr(view,'right_opening',None))):
            gap=None if detector is None else detector(start,leg.heading,position,endpoint,self.clearance)
            point_key=prefix+'gap_candidate';count_key=prefix+'gap_count'
            old=getattr(self,point_key)
            if gap is None:
                setattr(self,point_key,None);setattr(self,count_key,0);continue
            count=getattr(self,count_key)+1 if old is not None and np.linalg.norm(gap[1]-old)<.08 else 1
            setattr(self,point_key,np.asarray(gap[1]).copy());setattr(self,count_key,count)
            if count>=2:confirmed.append((gap[0],np.asarray(gap[1]),side))
        fresh_left=set()
        # Split the far crossing first, then the near one. Both get recorded
        # even if driving into the nearer left branch will defer the far road.
        for _,measured,side in sorted(confirmed,key=lambda item:item[0],reverse=True):
            leg=self.leg;start=self.nodes[leg.source].point
            vector=leg.end-start
            fraction=float(np.clip((measured-start) @ vector/max(vector @ vector,1e-9),0.,1.))
            point=start+fraction*vector;port=(leg.heading+side)%4
            if np.linalg.norm(point-measured)>.18:continue
            if np.linalg.norm(point-start)<self.merge_radius:
                node_id=leg.source
            elif leg.destination is not None and np.linalg.norm(point-leg.end)<self.merge_radius:
                node_id=leg.destination
            else:
                if view.ports(point).get(port)!='open':continue
                end,_=view.line_end(point,port,point,self.clearance)
                if np.linalg.norm(end-point)<.14:continue
                node_id=self.split_current_road(point)
                if node_id is None:continue
            self.observe_branches(view,node_id)
            if port not in self.nodes[node_id].branches:continue
            if side==1:fresh_left.add(node_id)
            self.openings+=1
            if side==-1:self.right_openings+=1
            prefix='' if side==1 else 'right_'
            setattr(self,prefix+'gap_candidate',None);setattr(self,prefix+'gap_count',0)
        return fresh_left

    def return_left_branch(self, view, position, heading, fresh_left=()):
        leg=self.leg;left=(leg.heading+1)%4
        # Finish the already scheduled turn before searching for another gap.
        if self.return_target is None and self.return_route and not self.return_route[0].returning:
            return False
        # A crossing may become visible just after we pass its centre. Keep a
        # short observation window at the source too, instead of losing it as
        # soon as the continuous PP route commits the next road segment.
        source_point=self.nodes[leg.source].point
        progress=float((position-source_point) @ direction(leg.heading))
        if (leg.source!=0 and 0.<=progress<.18
                and np.linalg.norm(position-source_point)<.25):
            self.observe_branches(view,leg.source)
            if left in self.untried(leg.source):
                self.leg=Leg(leg.destination,(leg.heading+2)%4,source_point.copy(),True,
                             leg.source,True,edge_id=leg.edge_id,visited=True)
                self.enter_return_left(leg.source,self.outgoing(leg.source,left));return True
        node_id=leg.destination
        if np.linalg.norm(leg.end-position)<self.preview or node_id in fresh_left:
            self.observe_branches(view,node_id)
            if left in self.untried(node_id):
                self.enter_return_left(node_id,self.outgoing(node_id,left));return True
        return False

    def choose(self, view, position, arrived):
        leg=self.leg
        if leg.destination is None:
            # Set the crossing once, before creating branch IDs. Returning roads
            # never recenter or regenerate a previously recorded junction.
            ports=view.ports(leg.end)
            for offset in (1,-1):
                index=(leg.heading+offset)%4
                if ports.get(index)!='open':continue
                centered=view.corridor_center(leg.end,index,self.clearance)
                if centered is not None:
                    leg.end=np.asarray(centered).copy();leg.anchor=self.slam.attach(leg.end)
                break
            self.bind_destination()
        node_id=leg.destination
        self.observe_branches(view,node_id)
        left=(leg.heading+1)%4
        if not arrived and self.nodes[node_id].ports.get(left,'unknown')=='unknown':return
        self.next_leg=self.select_branch(node_id,leg.heading)
        if self.next_leg is not None:
            self.state='TRACK_PATH';self.reason='saved branches, left first';return
        if any(self.nodes[node_id].ports.get(i,'unknown')=='unknown' for i in range(4)):
            if arrived:
                self.state='OBSERVE_JUNCTION';self.reason='wait for branch visibility'
            return
        # The reverse leg includes our current projection on the incoming
        # road. PP can turn back here; reaching its unused far tip adds no
        # information once every port has been observed and exhausted.
        self.start_return(node_id)

    def advance(self):
        self.gap_candidate=None;self.gap_count=0
        self.right_gap_candidate=None;self.right_gap_count=0
        if self.leg.returning:
            if self.return_route:
                self.leg=self.return_route.pop(0)
                self.next_leg=self.return_route[0] if self.return_route else None
                if not self.leg.returning:
                    self.return_target=None;self.state='TRACK_PATH'
                    self.reason='resume saved unexplored branch'
            return
        self.leg=self.next_leg;self.next_leg=None
        self.gap_candidate=None;self.gap_count=0
        self.state='TRACK_PATH'

    def refresh_pending_path(self, view, position):
        # A branch first seen from the side may have only a short provisional
        # ray. Extend the chosen branch while approaching the corner, before
        # committing the segment switch, so PP need not chase a tiny endpoint.
        pending=[self.next_leg]+[r for r in self.return_route if not r.returning]
        seen=set()
        for leg in pending:
            if leg is None or leg.returning or leg.terminal or leg.edge_id in seen:continue
            seen.add(leg.edge_id);edge=self.edges[leg.edge_id]
            if edge.destination is not None:continue
            start=self.nodes[leg.source].point
            end,terminal=view.line_end(start,leg.heading,position,self.clearance)
            if terminal or np.linalg.norm(end-start)>np.linalg.norm(leg.end-start):
                edge.end=np.asarray(end).copy();edge.anchor=self.slam.attach(edge.end)
                edge.terminal=terminal
                leg.end=edge.end.copy();leg.terminal=terminal

    def update(self, view, position, heading=None):
        position=np.asarray(position)
        if not self.nodes:self.begin(view,position,heading)
        if self.leg is None:return
        self.sync_route();leg=self.leg;start=self.nodes[leg.source].point
        self.refresh_pending_path(view,position)
        fresh_left=self.record_side_openings(view,position,heading)
        leg=self.leg;start=self.nodes[leg.source].point
        distance=float(np.linalg.norm(leg.end-position))
        along=float((leg.end-position) @ direction(leg.heading))
        arrived=distance<self.tolerance or (along<.025 and distance<.16)
        if leg.returning:
            # Reuse stored roads; only a confirmed, unvisited left branch may
            # replace the remainder of this return route.
            if self.return_left_branch(view,position,heading,fresh_left):return
            leg=self.leg
            if not self.return_route:
                if distance<self.preview or arrived:
                    selected=self.select_branch(leg.destination,leg.heading)
                    if selected is not None:
                        self.return_route=[selected];self.next_leg=selected
                    elif arrived:self.start_return(leg.destination);return
                if not self.return_route:return
            upcoming=self.return_route[0]
            progress=float((position-leg.end) @ direction(upcoming.heading))
            lateral=abs(float((position-leg.end) @ direction(upcoming.heading+1)))
            if (progress>.10 and lateral<.14) or distance<.035:self.advance()
            return
        if self.return_from_mapped_dead_end(position,heading):return
        # A branch occluded on approach can become visible while traversing
        # the crossing. Fill its saved port before leaving the junction, using
        # the same node and port IDs. Root keeps the mandatory initial heading.
        for node_id in {leg.source,leg.destination}:
            if node_id is not None and node_id!=0 and np.linalg.norm(self.nodes[node_id].point-position)<.45:
                self.observe_branches(view,node_id)
        if not leg.terminal and self.next_leg is None:
            end,terminal=view.line_end(start,leg.heading,position,self.clearance)
            if terminal or np.linalg.norm(end-start)>np.linalg.norm(leg.end-start):
                leg.end=end;leg.anchor=self.slam.attach(end)
            leg.terminal=terminal
        distance=float(np.linalg.norm(leg.end-position))
        along=float((leg.end-position) @ direction(leg.heading))
        arrived=distance<self.tolerance or (along<.025 and distance<.16)
        if (leg.terminal and self.next_leg is None
                and self.gap_candidate is None and self.right_gap_candidate is None
                and (distance<self.preview or arrived)):
            self.choose(view,position,arrived)
        if self.leg is not leg or self.leg is None:return
        if self.next_leg is not None:
            progress=float((position-leg.end) @ direction(self.next_leg.heading))
            lateral=abs(float((position-leg.end) @ direction(self.next_leg.heading+1)))
            if (progress>.10 and lateral<.14) or np.linalg.norm(leg.end-position)<.035:self.advance()
        elif not leg.terminal and arrived:
            self.state='WAIT_FRONTIER';self.reason='extend the existing visible ray'

    def path(self):
        if self.leg is None:return []
        self.sync_route()
        points=[self.nodes[self.leg.source].point,self.leg.end]
        if self.leg.returning:points.extend(leg.end for leg in self.return_route)
        elif self.next_leg is not None:points.append(self.next_leg.end)
        return [np.asarray(point) for point in points]

    def global_path(self):
        return self.path()

    def graph_segments(self):
        self.sync_route()
        return [(edge.edge_id,self.nodes[edge.source].point,edge.end,edge.visited)
                for edge in self.edges if not edge.retired]

    def graph_report(self):
        return dict(nodes=[dict(id=i,point=node.point.tolist(),ports=node.ports,
                               branches=node.branches) for i,node in enumerate(self.nodes)],
                    edges=[dict(id=edge.edge_id,source=edge.source,destination=edge.destination,
                                heading=edge.heading,end=edge.end.tolist(),visited=edge.visited,
                                map_dead_end=edge.map_dead_end,map_dead_end_cells=edge.map_dead_end_cells,
                                map_check_reason=edge.map_check_reason)
                           for edge in self.edges if not edge.retired],
                    return_target=self.return_target,return_edge_ids=self.last_return_edges,
                    map_forward_turnbacks=self.map_forward_turnbacks,last_map_turnback=self.last_map_turnback,
                    active_edge_id=None if self.leg is None else self.leg.edge_id)

def pursuit_target(path,position,lookahead):
    """Forward intersection of the path and the metric lookahead circle.

    Arc-length lookahead gets shorter in Euclidean distance at a corner and
    jumps when the closest projection changes segments. Circle intersection
    keeps the PP denominator and target continuous through a right angle.
    """
    position=np.asarray(position)
    best=(float('inf'),0,0.)
    for i,(a,b) in enumerate(zip(path[:-1],path[1:])):
        vector=b-a;fraction=float(np.clip((position-a) @ vector/max(float(vector @ vector),1e-9),0.,1.))
        distance=float(np.linalg.norm(a+fraction*vector-position))
        if distance<best[0]: best=distance,i,fraction
    _,index,fraction=best
    for i in range(index,len(path)-1):
        a,b=path[i],path[i+1];vector=b-a
        squared=float(vector @ vector)
        if squared<1e-12:continue
        delta=a-position
        middle=float(delta @ vector)
        discriminant=middle*middle-squared*(float(delta @ delta)-lookahead*lookahead)
        if discriminant<0.:continue
        # The larger root is the forward exit from the lookahead circle.
        root=(-middle+math.sqrt(discriminant))/squared
        minimum=fraction if i==index else 0.
        if minimum-1e-9<=root<=1.+1e-9:
            return a+float(np.clip(root,0.,1.))*vector
    # If far off the path, aim ahead of its nearest projection, not at a
    # distant final corner. Normal endpoint approach simply returns the end.
    if best[0]>lookahead:
        point=path[index]+fraction*(path[index+1]-path[index]);remaining=lookahead
        for endpoint in path[index+1:]:
            length=float(np.linalg.norm(endpoint-point))
            if length>=remaining:return point+(endpoint-point)*remaining/max(length,1e-9)
            remaining-=length;point=endpoint
    return path[-1]


def predict_pose(position,yaw,linear,angular,duration):
    """Measured-twist prediction to compensate chassis yaw response delay."""
    angle=angular*duration
    if abs(angular)<1e-6:x,y=linear*duration,0.
    else:x,y=linear/angular*math.sin(angle),linear/angular*(1.-math.cos(angle))
    c,s=math.cos(yaw),math.sin(yaw)
    return np.asarray(position)+[c*x-s*y,s*x+c*y],wrap(yaw+angle)


def pure_pursuit(position,yaw,target,speed):
    delta=np.asarray(target)-position;c,s=math.cos(yaw),math.sin(yaw)
    x=c*delta[0]+s*delta[1];y=-s*delta[0]+c*delta[1]
    return speed,2*speed*y/max(x*x+y*y,1e-6),math.atan2(y,x)


def yaw_rate_command(desired,measured,gain,limit):
    """Inner IMU rate loop: brake chassis momentum before PP overcorrects."""
    return float(np.clip(desired+gain*(desired-measured),-limit,limit))
