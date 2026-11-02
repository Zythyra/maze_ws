import math
from pathlib import Path
import sys
import time
from types import SimpleNamespace as S
import unittest
import numpy as np

sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from polyline_core import ExitDetector,Explorer,Frame2D,Junction,Leg,PoseGraphAnchors,ScanView,direction,predict_pose,pure_pursuit,pursuit_target,yaw_rate_command
from maze_odometry import MazeOdometry
from polyline_follower import PolylineFollower
from nav_msgs.msg import Odometry


def scan_view(walls,position=(0.,0.),yaw=0.):
    origin=np.asarray(position)-.032*np.array([math.cos(yaw),math.sin(yaw)])
    ranges=[]
    def cross(a,b):return a[0]*b[1]-a[1]*b[0]
    for angle in np.arange(360)*math.pi/180+yaw:
        ray=np.array([math.cos(angle),math.sin(angle)]);distance=float('inf')
        for a,b in walls:
            a,b=np.asarray(a),np.asarray(b);segment=b-a;den=cross(ray,segment)
            if abs(den)<1e-9:continue
            t=cross(a-origin,segment)/den;u=cross(a-origin,ray)/den
            if .12<t<3.5 and 0<=u<=1:distance=min(distance,t)
        ranges.append(distance)
    scan=S(ranges=ranges,range_min=.12,range_max=3.5,angle_min=0.,angle_increment=math.pi/180)
    return ScanView(scan,origin,yaw)


class BranchView:
    def opening(self,*args):return None
    def corridor_center(self,anchor,*args):return np.asarray(anchor)
    def line_end(self,anchor,heading,position,clearance):
        return np.asarray(anchor)+direction(heading),True
    def ports(self,point):
        if np.linalg.norm(point-[1.,0.])<.1:return {0:'open',1:'open',2:'open',3:'blocked'}
        return {0:'blocked',1:'blocked',2:'blocked',3:'open'}


class ExitDetectionTests(unittest.TestCase):
    def corridor(self):
        return scan_view([([-2.,.3],[3.,.3]),([-2.,-.3],[3.,-.3])]).scan

    def open_scan(self, angle_min=0.):
        return S(ranges=[float('inf')]*360,range_min=.12,range_max=3.5,
                 angle_min=angle_min,angle_increment=math.pi/180)

    def armed_detector(self):
        detector=ExitDetector()
        detector.update(self.corridor(),0.,0.)
        detector.update(self.corridor(),1.,.1)
        self.assertTrue(detector.armed)
        return detector

    def test_empty_world_startup_cannot_be_an_exit(self):
        detector=ExitDetector()
        for i in range(30):self.assertFalse(detector.update(self.open_scan(),i*.1,i*.1))
        self.assertFalse(detector.armed)

    def test_corridor_observation_still_requires_minimum_travel(self):
        detector=ExitDetector();detector.update(self.corridor(),0.,0.)
        for i in range(1,20):self.assertFalse(detector.update(self.open_scan(),.5,i*.1))
        self.assertFalse(detector.armed)

    def test_sustained_open_space_finishes_and_latches(self):
        detector=self.armed_detector()
        for i in range(8):self.assertFalse(detector.update(self.open_scan(),1.2,.2+i*.1))
        self.assertTrue(detector.update(self.open_scan(),1.7,1.0))
        self.assertTrue(detector.update(self.corridor(),1.7,1.1))
        self.assertAlmostEqual(detector.confirmed_at,1.)

    def test_minus_pi_layout_and_range_max_no_returns_are_supported(self):
        detector=self.armed_detector();scan=self.open_scan(-math.pi)
        scan.ranges=[3.5]*360
        for i in range(9):detector.update(scan,1.2,.2+i*.1)
        self.assertTrue(detector.reached)

    def test_single_exterior_wall_does_not_prevent_three_side_completion(self):
        walls=([[-3.4,.3],[3.4,.3]],[[-3.4,-.3],[3.4,-.3]],
               [[.3,-3.4],[.3,3.4]],[[-.3,-3.4],[-.3,3.4]])
        for wall in walls:
            detector=self.armed_detector();scan=scan_view([wall]).scan
            for i in range(9):detector.update(scan,1.2,.2+i*.1)
            self.assertTrue(detector.reached)
            self.assertEqual(detector.open_sides,3)
            self.assertGreaterEqual(detector.open_arc_deg,180.)

    def test_two_blocked_directions_prevent_exit(self):
        for indices in ((0,90),(90,270),(180,270)):
            detector=self.armed_detector();scan=self.open_scan()
            for index in indices:scan.ranges[index]=.7
            for i in range(15):self.assertFalse(detector.update(scan,1.2,.2+i*.1))

    def test_three_clear_corridor_mouths_at_t_junction_do_not_finish(self):
        walls=[([-3.,-.3],[3.,-.3]),([-3.,.3],[-.3,.3]),([.3,.3],[3.,.3]),
               ([-.3,.3],[-.3,3.]),([.3,.3],[.3,3.])]
        detector=self.armed_detector();scan=scan_view(walls).scan
        for i in range(15):self.assertFalse(detector.update(scan,1.2,.2+i*.1))
        self.assertEqual(detector.open_sides,3)
        self.assertLess(detector.open_arc_deg,180.)

    def test_cross_junction_and_dead_end_are_not_open_exteriors(self):
        cross=[]
        for sign in (-1,1):
            cross.extend([([sign*.3,.3],[sign*.3,2.]),([sign*.3,-.3],[sign*.3,-2.]),
                          ([.3,sign*.3],[2.,sign*.3]),([-.3,sign*.3],[-2.,sign*.3])])
        dead_end=[([-2.,.3],[.3,.3]),([-2.,-.3],[.3,-.3]),([.3,-.3],[.3,.3])]
        for walls in (cross,dead_end):
            detector=self.armed_detector();scan=scan_view(walls).scan
            for i in range(15):self.assertFalse(detector.update(scan,1.2,.2+i*.1))

    def test_invalid_or_partial_scans_do_not_count_as_open_space(self):
        for value in (float('nan'),float('-inf'),0.,-1.):
            detector=self.armed_detector();scan=self.open_scan();scan.ranges=[value]*360
            for i in range(15):self.assertFalse(detector.update(scan,1.2,.2+i*.1))
            self.assertFalse(detector.scan_valid)
        detector=self.armed_detector();scan=self.open_scan();scan.ranges=scan.ranges[:180]
        for i in range(15):self.assertFalse(detector.update(scan,1.2,.2+i*.1))

    def test_duplicate_timestamps_cannot_complete_confirmation(self):
        detector=self.armed_detector()
        for _ in range(30):self.assertFalse(detector.update(self.open_scan(),1.2,.2))
        self.assertFalse(detector.reached)

    def test_scan_gap_or_returning_wall_resets_confirmation(self):
        detector=self.armed_detector()
        for i in range(5):detector.update(self.open_scan(),1.2,.2+i*.1)
        self.assertFalse(detector.update(self.open_scan(),1.2,1.5))
        self.assertEqual(detector.clear_since,1.5)
        detector.update(self.corridor(),1.2,1.6)
        self.assertIsNone(detector.clear_since)

    def test_finished_controller_keeps_zero_even_without_sensors(self):
        commands=[]
        state=S(enabled=True,scan=None,odom=None,mission_phase='HOME_REACHED',
                p=dict(scan_timeout=.7,exit_detection_enabled=True),
                exit_detector=S(reached=True),last_status=time.monotonic(),state='TRACK_PP',
                publish_command=lambda v,w:commands.append((v,w)))
        PolylineFollower.control(state)
        self.assertEqual(state.state,'HOME_REACHED');self.assertEqual(commands,[(0.,0.)])
        response=S();PolylineFollower.set_enabled(state,S(data=True),response)
        self.assertFalse(response.success);self.assertEqual(commands[-1],(0.,0.))


class PolylineTests(unittest.TestCase):
    def tracking_state(self, path, position, heading):
        odom=Odometry();odom.twist.twist.linear.x=.35
        return S(planner=S(path=lambda:[np.asarray(p) for p in path],state='TRACK_PATH',
                           next_leg=None,leg=S(returning=False)),
                 view=object(),position=np.asarray(position),heading=heading,odom=odom,
                 aligning=False,p=dict(goal_tolerance=.065,prediction_time=.10,lookahead=.32,
                     max_linear_speed=.35,max_angular_speed=2.4,max_alignment_speed=1.4,
                     yaw_rate_gain=.5,align_enter_angle=2.4,align_exit_angle=.45))

    def return_fixture(self):
        planner=Explorer()
        points=([0.,0.],[1.,0.],[1.,1.],[2.,1.],[2.,2.])
        planner.nodes=[Junction(np.array(p),i-1 if i else None,{j:'blocked' for j in range(4)})
                       for i,p in enumerate(points)]
        for source,target,heading in [(0,1,0),(1,2,1),(2,3,0),(3,4,1)]:
            edge=planner.new_edge(source,heading,planner.nodes[target].point,True)
            edge.destination=target;edge.visited=True
            planner.nodes[source].ports[heading]='tried'
            planner.nodes[source].neighbors[heading]=target
            planner.nodes[target].ports[(heading+2)%4]='tried'
            planner.nodes[target].neighbors[(heading+2)%4]=source
            planner.nodes[target].branches[(heading+2)%4]=edge.edge_id
        planner.new_edge(1,0,np.array([2.,0.]),True)
        planner.nodes[1].ports[0]='open'
        planner.leg=planner.edges[3]
        return planner

    def test_t_junction_records_both_unexplored_directions_before_selecting_left(self):
        planner=Explorer();view=BranchView()
        planner.update(view,np.zeros(2));planner.update(view,np.array([.8,0.]))
        node=planner.nodes[1]
        self.assertIn(0,node.branches);self.assertIn(1,node.branches)
        self.assertEqual(planner.next_leg.heading,1)
        self.assertFalse(planner.edges[node.branches[0]].visited)
        self.assertEqual(planner.untried(1),[0])

    def test_later_occlusion_does_not_erase_a_saved_unexplored_direction(self):
        class Occluded(BranchView):
            def ports(self,point):return {i:'blocked' for i in range(4)}
        planner=Explorer();view=BranchView()
        planner.update(view,np.zeros(2));planner.update(view,np.array([.8,0.]))
        roads=len(planner.edges)
        planner.observe_branches(Occluded(),1)
        self.assertEqual(planner.untried(1),[0])
        self.assertEqual(len(planner.edges),roads)

    def test_opposite_observation_of_saved_road_reuses_one_undirected_id(self):
        planner=Explorer()
        planner.nodes=[Junction(np.array([0.,0.]),None),Junction(np.array([1.,0.]),0)]
        original=planner.new_edge(0,0,np.array([1.,0.]),True);original.visited=True
        reverse=planner.new_edge(1,2,np.array([0.,0.]),True)
        self.assertEqual(original.edge_id,reverse.edge_id)
        self.assertEqual(len(planner.graph_segments()),1)
        self.assertEqual(planner.nodes[1].branches[2],original.edge_id)
        self.assertEqual(planner.nodes[1].ports[2],'tried')

    def test_reobserving_same_port_does_not_draw_a_shifted_duplicate(self):
        planner=Explorer()
        planner.nodes=[Junction(np.array([0.,0.]),None)]
        original=planner.new_edge(0,0,np.array([1.,0.]),True)
        again=planner.new_edge(0,0,np.array([1.12,.14]),True)
        self.assertIs(original,again)
        np.testing.assert_allclose(original.end,[1.,0.])
        self.assertEqual(len(planner.edges),1)

    def test_direction_occluded_on_approach_is_saved_while_passing_junction(self):
        class PartialView(BranchView):
            visible=False
            def ports(self,point):
                result=super().ports(point)
                if np.linalg.norm(point-[1.,0.])<.1 and not self.visible:result[0]='blocked'
                return result
        planner=Explorer();view=PartialView()
        for point in ([0.,0.],[.8,0.],[1.,.15]):planner.update(view,np.array(point))
        self.assertNotIn(0,planner.nodes[1].branches)
        view.visible=True
        planner.update(view,np.array([1.,.20]))
        self.assertIn(0,planner.nodes[1].branches)
        self.assertEqual(planner.untried(1),[0])

    def test_deadend_prepares_one_route_past_two_corners_to_actual_frontier(self):
        planner=self.return_fixture();planner.start_return(4)
        self.assertEqual(planner.return_target,1)
        self.assertEqual(planner.last_return_edges,[3,2,1])
        np.testing.assert_allclose(planner.path(),[[2.,2.],[2.,1.],[1.,1.],[1.,0.]])
        self.assertEqual(planner.backtracks,1)

    def test_confirmed_deadend_returns_before_reaching_tip(self):
        class DeadEnd(BranchView):
            def ports(self,point):return {i:'blocked' for i in range(4)}
        planner=self.return_fixture();position=np.array([2.,1.55])
        before=(len(planner.nodes),len(planner.edges))
        planner.update(DeadEnd(),position,math.pi/2)
        self.assertEqual(planner.backtracks,1)
        self.assertEqual(planner.return_target,1)
        self.assertEqual(planner.last_return_edges,[3,2,1])
        self.assertEqual((len(planner.nodes),len(planner.edges)),before)
        self.assertGreater(np.linalg.norm(position-planner.nodes[4].point),.4)
        target=pursuit_target(planner.path(),position,.32)
        self.assertLess(target[1],position[1])

    def test_unknown_side_does_not_trigger_early_deadend_return(self):
        class Occluded(BranchView):
            def ports(self,point):
                return {0:'unknown',1:'blocked',2:'blocked',3:'open'}
        planner=self.return_fixture();position=np.array([2.,1.55])
        planner.update(Occluded(),position,math.pi/2)
        self.assertEqual(planner.backtracks,0)
        self.assertFalse(planner.leg.returning)
        self.assertEqual(planner.nodes[4].ports[0],'unknown')

    def test_occluded_corner_is_unknown_then_opens_when_approached(self):
        for side in (-1,1):
            walls=[((-1.,side*.3),(.7,side*.3)),((.7,side*.3),(.7,side*1.6)),
                   ((1.3,-side*.3),(1.3,side*1.6)),((-1.,-side*.3),(1.3,-side*.3))]
            point=np.array([.88,0.]);port=1 if side==1 else 3
            far=scan_view(walls,(.4,0.));near=scan_view(walls,(.65,0.))
            # This was the false-deadend trigger: the long branch probe is
            # occluded by the incoming side wall, not a wall across the mouth.
            self.assertFalse(far.clear(point+direction(port)*.52,.015))
            self.assertEqual(far.ports(point)[port],'unknown')
            self.assertEqual(near.ports(point)[port],'open')

    def test_hidden_branch_is_not_backtracked_and_is_taken_after_visibility_improves(self):
        walls=[((-1.,.3),(.7,.3)),((.7,.3),(.7,1.6)),
               ((1.3,-.3),(1.3,1.6)),((-1.,-.3),(1.3,-.3))]
        planner=Explorer();planner.nodes=[Junction(np.zeros(2),None,{0:'tried'})]
        planner.leg=planner.new_edge(0,0,np.array([.88,0.]),True);planner.leg.visited=True
        planner.update(scan_view(walls,(.41,0.)),np.array([.41,0.]),0.)
        self.assertEqual(planner.backtracks,0);self.assertIsNone(planner.next_leg)
        self.assertIsNotNone(planner.leg);self.assertEqual(planner.nodes[1].ports[1],'unknown')
        planner.update(scan_view(walls,(.65,0.)),np.array([.65,0.]),0.)
        self.assertIsNotNone(planner.next_leg);self.assertEqual(planner.next_leg.heading,1)
        self.assertEqual(planner.backtracks,0)

    def test_jambs_without_a_visible_middle_do_not_prove_closure(self):
        walls=[((.3,-.22),(.3,-.07)),((.3,.07),(.3,.22))]
        view=scan_view(walls,(-.5,0.))
        self.assertFalse(view.closing_wall(np.zeros(2),0))

    def test_pending_left_opening_is_confirmed_before_endpoint_deadend_choice(self):
        class OpeningCandidate(BranchView):
            def opening(self,*args):return .8,np.array([.8,0.]),.6
            def ports(self,point):return {i:'blocked' for i in range(4)}
        planner=Explorer();view=OpeningCandidate()
        planner.begin(view,np.zeros(2))
        planner.update(view,np.array([.55,0.]))
        self.assertEqual(planner.gap_count,1)
        self.assertIsNone(planner.leg.destination)
        self.assertIsNotNone(planner.leg)
        self.assertEqual(planner.backtracks,0)

    def test_lidar_confirms_real_deadend_without_driving_to_wall_clearance_tip(self):
        walls=[((1.7,1.),(1.7,2.3)),((2.3,1.),(2.3,2.3)),((1.7,2.3),(2.3,2.3))]
        planner=self.return_fixture();position=np.array([2.,1.56])
        planner.update(scan_view(walls,position,math.pi/2),position,math.pi/2)
        self.assertTrue(planner.leg.returning)
        self.assertEqual(planner.return_target,1)
        self.assertGreater(np.linalg.norm(position-planner.nodes[4].point),.4)

    def test_drifted_return_without_new_openings_keeps_existing_roads_and_vertices(self):
        class NoNewRoads:
            def opening(self,*args):return None
            def ports(self,point):return {i:'unknown' for i in range(4)}
        planner=self.return_fixture();planner.start_return(4)
        before=(len(planner.nodes),len(planner.edges))
        original=[node.point.copy() for node in planner.nodes]
        for point in ([1.85,1.06],[1.05,.85],[1.03,.25],[1.16,.04]):
            planner.update(NoNewRoads(),np.array(point))
        self.assertEqual((len(planner.nodes),len(planner.edges)),before)
        np.testing.assert_allclose([node.point for node in planner.nodes],original)
        self.assertIsNone(planner.return_target)
        self.assertIs(planner.leg,planner.edges[4])
        self.assertFalse(planner.leg.returning)
        self.assertEqual(planner.backtracks,1)

    def test_return_takes_new_left_at_intermediate_junction_before_old_target(self):
        class NewLeft(BranchView):
            def ports(self,point):return {0:'open',1:'open',2:'blocked',3:'blocked'}
        planner=self.return_fixture();planner.start_return(4)
        before=len(planner.edges)
        planner.update(NewLeft(),np.array([2.,1.4]),-math.pi/2)
        self.assertIsNone(planner.return_target)
        self.assertEqual(planner.return_discoveries,1)
        self.assertEqual(len(planner.nodes),5);self.assertEqual(len(planner.edges),before+1)
        np.testing.assert_allclose(planner.path(),[[2.,2.],[2.,1.],[3.,1.]])
        planner.update(NewLeft(),np.array([2.15,1.]),0.)
        self.assertFalse(planner.leg.returning);self.assertEqual(planner.leg.heading,0)
        self.assertIn(0,planner.untried(1))  # The old return target remains available.

    def test_return_enters_saved_unvisited_left_without_creating_another_road(self):
        class Unknown(BranchView):
            def ports(self,point):return {i:'unknown' for i in range(4)}
        planner=self.return_fixture()
        edge=planner.new_edge(3,0,np.array([3.,1.]),True);planner.nodes[3].ports[0]='open'
        planner.start_return(4);before=(len(planner.nodes),len(planner.edges))
        planner.update(Unknown(),np.array([2.,1.4]),-math.pi/2)
        self.assertEqual((len(planner.nodes),len(planner.edges)),before)
        self.assertIs(planner.next_leg,edge);self.assertIsNone(planner.return_target)

    def test_left_that_becomes_visible_just_after_a_return_crossing_is_not_missed(self):
        class LateLeft(BranchView):
            def ports(self,point):
                if np.linalg.norm(point-[2.,2.])<.1:
                    return {0:'open',1:'blocked',2:'blocked',3:'open'}
                return {i:'blocked' for i in range(4)}
        planner=self.return_fixture();planner.start_return(4)
        planner.update(LateLeft(),np.array([2.,1.86]),-math.pi/2)
        self.assertEqual(planner.return_discoveries,1);self.assertIsNone(planner.return_target)
        self.assertEqual(planner.leg.destination,4);self.assertEqual(planner.next_leg.heading,0)
        self.assertEqual((len(planner.nodes),len(planner.edges)),(5,6))
        np.testing.assert_allclose(planner.path(),[[2.,1.],[2.,2.],[3.,2.]])

    def test_return_splits_one_existing_road_for_new_midroad_left_in_both_directions(self):
        for heading in (1,3):
            class NewGap(BranchView):
                def opening(self,*args):return .5,np.array([2.,1.5]),.6
                def ports(self,point):
                    if np.linalg.norm(point-[2.,1.5])>.1:return {i:'unknown' for i in range(4)}
                    return {i:('blocked' if i==(heading-1)%4 else 'open') for i in range(4)}
            planner=self.return_fixture()
            if heading==3:planner.start_return(4);source,target=4,3
            else:
                source,target=3,4;planner.return_target=1
                planner.leg=Leg(3,1,np.array([2.,2.]),True,4,True,edge_id=3,visited=True)
            position=np.array([2.,1.9 if heading==3 else 1.1]);view=NewGap()
            planner.update(view,position,heading*math.pi/2)
            self.assertEqual(len(planner.nodes),5)  # Single-scan candidate does not split.
            planner.update(view,position,heading*math.pi/2)
            self.assertEqual(len(planner.nodes),6);self.assertEqual(len(planner.edges),7)
            self.assertEqual(planner.edges[3].source,3);self.assertEqual(planner.edges[3].destination,5)
            self.assertEqual(planner.edges[5].source,5);self.assertEqual(planner.edges[5].destination,4)
            self.assertEqual(planner.nodes[3].neighbors[1],5)
            self.assertEqual(planner.nodes[4].neighbors[3],5)
            self.assertEqual(planner.nodes[5].neighbors,{3:3,1:4})
            self.assertEqual(planner.leg.source,source);self.assertEqual(planner.leg.destination,5)
            self.assertEqual(planner.next_leg.heading,(heading+1)%4)
            self.assertIsNone(planner.return_target)
            old_length=np.linalg.norm(planner.edges[3].end-planner.nodes[3].point)
            remainder=np.linalg.norm(planner.edges[5].end-planner.nodes[5].point)
            self.assertAlmostEqual(old_length+remainder,1.)
            # Re-observing the same gap while entering it does not add IDs.
            planner.update(view,position,heading*math.pi/2)
            self.assertEqual((len(planner.nodes),len(planner.edges)),(6,7))

    def test_return_does_not_reenter_explored_left(self):
        class OldLeft(BranchView):
            def ports(self,point):return {0:'open',1:'open',2:'blocked',3:'blocked'}
        planner=self.return_fixture()
        edge=planner.new_edge(3,0,np.array([3.,1.]),True);edge.visited=True
        planner.nodes[3].ports[0]='tried';planner.start_return(4)
        before=(len(planner.nodes),len(planner.edges))
        planner.update(OldLeft(),np.array([2.,1.4]),-math.pi/2)
        self.assertEqual(planner.return_target,1);self.assertEqual(planner.return_discoveries,0)
        self.assertEqual((len(planner.nodes),len(planner.edges)),before)

    def test_interrupting_return_releases_unentered_branch_at_old_target(self):
        class LeftAppears(BranchView):
            def ports(self,point):return {0:'open',1:'open',2:'blocked',3:'open'}
        planner=self.return_fixture();planner.start_return(4)
        reserved=planner.new_edge(3,3,np.array([2.,0.]),True)
        reserved.visited=True;planner.nodes[3].ports[3]='tried'
        planner.return_target=3;planner.return_route=[reserved];planner.next_leg=reserved
        planner.update(LeftAppears(),np.array([2.,1.4]),-math.pi/2)
        self.assertFalse(reserved.visited);self.assertIn(3,planner.untried(3))
        self.assertEqual(planner.next_leg.heading,0);self.assertIsNone(planner.return_target)

    def test_real_lidar_finds_outbound_right_gap_as_return_left_and_keeps_straight_road(self):
        walls=[((-.3,.3),(3.3,.3)),((-.3,-.3),(1.2,-.3)),((1.8,-.3),(3.3,-.3)),
               ((1.2,-.3),(1.2,-1.8)),((1.8,-.3),(1.8,-1.8)),((1.2,-1.8),(1.8,-1.8))]
        planner=Explorer()
        planner.nodes=[Junction(np.array([0.,0.]),None,{0:'tried',1:'open'}),
                       Junction(np.array([3.,0.]),0,{2:'tried'})]
        road=planner.new_edge(0,0,np.array([3.,0.]),True);road.destination=1;road.visited=True
        planner.nodes[0].neighbors[0]=1;planner.nodes[1].neighbors[2]=0;planner.nodes[1].branches[2]=0
        planner.new_edge(0,1,np.array([0.,1.]),True);planner.start_return(1)
        for x in (2.1,2.065):
            planner.update(scan_view(walls,(x,0.),math.pi),np.array([x,0.]),math.pi)
        self.assertEqual(planner.return_discoveries,1)
        self.assertEqual(planner.next_leg.heading,3);self.assertIsNone(planner.return_target)
        self.assertEqual(len(planner.nodes),3);self.assertEqual(len(planner.edges),4)
        self.assertAlmostEqual(planner.nodes[2].point[0],1.5,delta=.03)
        self.assertAlmostEqual(planner.nodes[2].point[1],0.,delta=1e-9)
        position=np.array([1.65,0.]);target=pursuit_target(planner.path(),position,.32)
        self.assertGreater(pure_pursuit(position,math.pi,target,.35)[1],0.)

    def test_forward_right_opening_creates_node_immediately_and_is_saved_while_driving_straight(self):
        walls=[((-.3,.3),(3.3,.3)),((-.3,-.3),(1.2,-.3)),((1.8,-.3),(3.3,-.3)),
               ((1.2,-.3),(1.2,-1.8)),((1.8,-.3),(1.8,-1.8)),((1.2,-1.8),(1.8,-1.8))]
        planner=Explorer();planner.begin(scan_view(walls),np.zeros(2))
        for x in (.9,.935):planner.update(scan_view(walls,(x,0.)),np.array([x,0.]),0.)
        self.assertEqual(len(planner.nodes),2);self.assertEqual(len(planner.edges),3)
        junction=planner.nodes[1]
        self.assertAlmostEqual(junction.point[0],1.5,delta=.03)
        self.assertGreater(np.linalg.norm(junction.point-[.935,0.]),planner.preview)
        right=planner.edges[junction.branches[3]]
        self.assertFalse(right.visited);self.assertEqual(junction.ports[3],'open')
        planner.update(scan_view(walls,(1.1,0.)),np.array([1.1,0.]),0.)
        self.assertEqual(planner.next_leg.heading,0);self.assertFalse(right.visited)
        for x in (1.65,1.7):planner.update(scan_view(walls,(x,0.)),np.array([x,0.]),0.)
        self.assertEqual((len(planner.nodes),len(planner.edges)),(2,3))
        self.assertIn(3,planner.untried(1))

    def test_opposite_side_openings_share_one_node_and_left_has_priority(self):
        walls=[]
        for side in (-1,1):
            walls.extend([((-.3,side*.3),(1.2,side*.3)),((1.8,side*.3),(3.3,side*.3)),
                          ((1.2,side*.3),(1.2,side*1.8)),((1.8,side*.3),(1.8,side*1.8)),
                          ((1.2,side*1.8),(1.8,side*1.8))])
        planner=Explorer();planner.begin(scan_view(walls),np.zeros(2))
        for x in (.9,.935,1.1):planner.update(scan_view(walls,(x,0.)),np.array([x,0.]),0.)
        self.assertEqual((len(planner.nodes),len(planner.edges)),(2,4))
        self.assertEqual(planner.side_junctions,1)
        self.assertEqual(set(planner.nodes[1].branches),{0,1,2,3})
        self.assertEqual(planner.next_leg.heading,1)
        self.assertIn(3,planner.untried(1));self.assertIn(0,planner.untried(1))

    def test_two_different_visible_crossings_are_both_registered_before_selecting_route(self):
        class TwoGaps(BranchView):
            def opening(self,*args):return 2.,np.array([2.,0.]),.6
            def right_opening(self,*args):return 1.,np.array([1.,0.]),.6
            def ports(self,point):
                return {0:'open',2:'open',1:'open' if point[0]>1.5 else 'blocked',
                        3:'open' if point[0]<1.5 else 'blocked'}
        planner=Explorer();planner.nodes=[Junction(np.zeros(2),None,{0:'tried'})]
        planner.leg=planner.new_edge(0,0,np.array([3.,0.]),True);planner.leg.visited=True
        for _ in range(2):planner.update(TwoGaps(),np.array([.4,0.]),0.)
        self.assertEqual((len(planner.nodes),len(planner.edges)),(3,5))
        far,near=planner.nodes[1],planner.nodes[2]
        np.testing.assert_allclose(far.point,[2.,0.]);np.testing.assert_allclose(near.point,[1.,0.])
        self.assertIn(1,planner.untried(1));self.assertIn(3,planner.untried(2))
        self.assertEqual(near.neighbors[0],1);self.assertEqual(far.neighbors[2],2)
        self.assertEqual(planner.leg.destination,2)

    def test_new_right_on_return_is_recorded_without_abandoning_original_target(self):
        class RightGap(BranchView):
            def right_opening(self,*args):return .5,np.array([2.,1.5]),.6
            def ports(self,point):
                if np.linalg.norm(point-[2.,1.5])>.1:return {i:'unknown' for i in range(4)}
                return {0:'blocked',1:'open',2:'open',3:'open'}
        planner=self.return_fixture();planner.start_return(4)
        for _ in range(2):planner.update(RightGap(),np.array([2.,1.9]),-math.pi/2)
        self.assertEqual((len(planner.nodes),len(planner.edges)),(6,7))
        self.assertEqual(planner.return_target,1);self.assertEqual(planner.return_discoveries,0)
        self.assertEqual(planner.last_return_edges,[5,3,2,1])
        self.assertIn(2,planner.untried(5))
        np.testing.assert_allclose(planner.path(),[[2.,2.],[2.,1.5],[2.,1.],[1.,1.],[1.,0.]])
        planner.update(RightGap(),np.array([2.,1.35]),-math.pi/2)
        self.assertTrue(planner.leg.returning);self.assertEqual(planner.return_target,1)

    def test_right_opening_has_the_same_measured_wall_offset_as_left(self):
        walls=[((-1.,-.3),(.6,-.3)),((1.2,-.3),(1.2,-1.4)),((-1.,.3),(4.,.3))]
        view=scan_view(walls,position=(.35,0.))
        opening=view.right_opening(np.zeros(2),0,np.array([.35,0.]),np.array([2.7,0.]),.3)
        self.assertIsNotNone(opening);self.assertAlmostEqual(opening[1][0],.9,delta=.025)

    def test_forward_split_from_other_end_marks_only_the_approached_part_tried(self):
        class Gap(BranchView):
            def opening(self,*args):return 1.5,np.array([1.5,0.]),.6
            def ports(self,point):return {0:'open',1:'blocked',2:'open',3:'open'}
        planner=Explorer()
        planner.nodes=[Junction(np.zeros(2),None,{0:'open'}),
                       Junction(np.array([3.,0.]),0,{2:'tried'},branches={2:0})]
        edge=planner.new_edge(0,0,np.array([3.,0.]),True);edge.destination=1;edge.visited=True
        planner.nodes[0].neighbors[0]=1;planner.nodes[1].neighbors[2]=0
        planner.leg=Leg(1,2,np.zeros(2),True,0,False,edge_id=0,visited=True)
        for _ in range(2):planner.update(Gap(),np.array([2.4,0.]),math.pi)
        self.assertFalse(planner.edges[0].visited);self.assertTrue(planner.edges[1].visited)
        self.assertEqual(planner.leg.edge_id,1)
        self.assertEqual(planner.nodes[2].ports[0],'tried')
        self.assertIn(2,planner.untried(2));self.assertIn(3,planner.untried(2))

    def test_map_to_odom_correction_preserves_pursuit_in_body_coordinates(self):
        transform=Frame2D(np.array([1.2,-.7]),.3)
        position=np.array([.4,.2]);target=np.array([.7,.4]);heading=.1
        a=pure_pursuit(position,heading,target,.2)
        b=pure_pursuit(transform.apply(position),heading+transform.yaw,transform.apply(target),.2)
        np.testing.assert_allclose(a,b,atol=1e-10)
        np.testing.assert_allclose(transform.inverse(transform.apply(position)),position)

    def test_loop_optimization_moves_existing_roads_without_changing_their_ids(self):
        planner=self.return_fixture()
        poses={i:np.array([.3*i,0.]) for i in range(9)}
        planner.update_pose_graph(poses);planner.start_return(4)
        original=[node.point.copy() for node in planner.nodes]
        edge_ids=[item[0] for item in planner.graph_segments()]
        transform=Frame2D(np.array([.45,-.2]),.17)
        planner.update_pose_graph({key:transform.apply(p) for key,p in poses.items()})
        np.testing.assert_allclose([node.point for node in planner.nodes],
                                   [transform.apply(p) for p in original],atol=1e-9)
        self.assertEqual([item[0] for item in planner.graph_segments()],edge_ids)
        self.assertEqual(planner.last_return_edges,[3,2,1])
        np.testing.assert_allclose(planner.path(),[transform.apply(p) for p in
                                   [[2.,2.],[2.,1.],[1.,1.],[1.,0.]]],atol=1e-9)
        self.assertEqual(planner.map_revisions,1)

    def test_map_wall_fit_corrects_rotated_corridor_without_drawing_another_road(self):
        planner=Explorer();planner.nodes=[Junction(np.array([0.,0.]),None),Junction(np.array([2.,0.]),0)]
        edge=planner.new_edge(0,0,np.array([2.,0.]),True);edge.destination=1;edge.visited=True
        x=np.linspace(.02,1.98,100);slope=.07;scale=math.sqrt(1+slope*slope)
        walls=np.vstack([np.column_stack((x,.08+slope*x+side*.3*scale)) for side in (-1,1)])
        for _ in range(14):planner.align_to_walls(walls,lambda _:True)
        normal=np.array([-slope,1.])/scale;offset=.08/scale
        for node in planner.nodes:self.assertAlmostEqual(float(node.point @ normal),offset,delta=.004)
        self.assertEqual(len(planner.nodes),2);self.assertEqual(len(planner.edges),1)
        self.assertEqual(planner.graph_segments()[0][0],0)
        self.assertGreater(planner.wall_revisions,0)

    def test_map_wall_fit_does_not_move_reference_into_unknown_or_occupied_cells(self):
        planner=Explorer();planner.nodes=[Junction(np.array([0.,0.]),None),Junction(np.array([1.,0.]),0)]
        edge=planner.new_edge(0,0,np.array([1.,0.]),True);edge.destination=1;edge.visited=True
        x=np.linspace(0.,1.,80);walls=np.column_stack((x,np.full_like(x,.40)))
        planner.align_to_walls(walls,lambda _:False)
        np.testing.assert_allclose([node.point for node in planner.nodes],[[0.,0.],[1.,0.]])
        self.assertEqual(planner.wall_revisions,0)

    def test_fusion_uses_signed_integrated_distance_not_instantaneous_wheel_noise(self):
        published=[]
        state=S(x=0.,y=0.,yaw=0.,rate=0.,imu_origin=0.,imu_stamp=1.,
                previous_time=1.,previous_yaw=0.,previous_wheel=(0.,0.,0.),
                publisher=S(publish=published.append),broadcaster=S(sendTransform=lambda _:None))
        message=Odometry();message.header.stamp.sec=1;message.header.stamp.nanosec=100000000
        message.pose.pose.orientation.w=1.;message.pose.pose.position.x=.02
        message.twist.twist.linear.x=.9  # Last physics-step speed disagrees with integrated travel.
        MazeOdometry.on_wheel(state,message)
        self.assertAlmostEqual(state.x,.02)
        self.assertAlmostEqual(published[-1].twist.twist.linear.x,.2)
        message.header.stamp.nanosec=200000000;message.pose.pose.position.x=-.01
        MazeOdometry.on_wheel(state,message)
        self.assertAlmostEqual(state.x,-.01)
        self.assertAlmostEqual(published[-1].twist.twist.linear.x,-.3)

    def test_initial_line_stops_exactly_point_three_before_observed_wall(self):
        walls=[((2.,-1.),(2.,1.)),((-1.,.3),(3.,.3)),((-1.,-.3),(3.,-.3))]
        view=scan_view(walls);end,terminal=view.line_end(np.zeros(2),0,np.zeros(2),.3)
        self.assertTrue(terminal);np.testing.assert_allclose(end,[1.7,0.],atol=.015)

    def test_unseen_wall_produces_extendable_same_direction_line(self):
        end,terminal=scan_view([]).line_end(np.zeros(2),0,np.zeros(2),.3)
        self.assertFalse(terminal);self.assertGreater(end[0],2.5)
        later,_=scan_view([],position=(1.,0.)).line_end(np.zeros(2),0,np.array([1.,0.]),.3)
        self.assertGreater(later[0],end[0]+.9);self.assertEqual(later[1],0.)

    def test_left_opening_center_is_measured_from_both_edges(self):
        walls=[((-1.,.3),(.6,.3)),((1.2,.3),(4.,.3)),((-1.,-.3),(4.,-.3)),
               ((.6,.3),(.6,.95)),((1.2,.3),(1.2,.95)),((.6,.95),(1.2,.95)),
               ((3.,-.3),(3.,.3))]
        view=scan_view(walls,position=(.35,0.))
        opening=view.opening(np.zeros(2),0,np.array([.35,0.]),np.array([2.7,0.]),.3)
        self.assertIsNotNone(opening)
        self.assertAlmostEqual(opening[1][0],.9,delta=.075)

    def test_one_visible_branch_wall_defines_line_exactly_point_three_from_wall(self):
        walls=[((-1.,.3),(.6,.3)),((1.2,.3),(1.2,1.4)),((-1.,-.3),(4.,-.3))]
        view=scan_view(walls,position=(.35,0.))
        center=view.corridor_center(np.array([1.05,0.]),1,.3)
        self.assertIsNotNone(center)
        self.assertAlmostEqual(center[0],.9,delta=.015)
        opening=view.opening(np.zeros(2),0,np.array([.35,0.]),np.array([2.7,0.]),.3)
        self.assertIsNotNone(opening)
        self.assertAlmostEqual(opening[1][0],.9,delta=.025)

    def test_side_wall_correction_preserves_orthogonal_corner_geometry(self):
        walls=[((1.2,-.3),(1.2,1.4)),((.6,.3),(.6,1.4)),((-1.,-.3),(1.2,-.3))]
        view=scan_view(walls,position=(.65,0.))
        centered=view.corridor_center(np.array([1.04,0.]),1,.3)
        self.assertIsNotNone(centered)
        np.testing.assert_allclose(centered,[.9,0.],atol=.015)

    def test_left_has_priority_and_produces_orthogonal_next_line(self):
        planner=Explorer();view=BranchView()
        planner.update(view,np.zeros(2));planner.update(view,np.array([.8,0.]))
        self.assertEqual(planner.next_leg.heading,1)
        path=planner.path();self.assertAlmostEqual(float((path[1]-path[0]) @ (path[2]-path[1])),0.)

    def test_deadend_returns_to_parent_and_does_not_reenter_tried_left_branch(self):
        planner=Explorer();view=BranchView()
        for point in ([0.,0.],[.8,0.],[1.,.15],[1.,.97]):planner.update(view,np.array(point))
        self.assertEqual(planner.backtracks,1)
        self.assertTrue(planner.leg.returning)
        np.testing.assert_allclose(planner.leg.end,[1.,0.])
        planner.update(view,np.array([1.,.20]))
        self.assertEqual(planner.next_leg.heading,0)
        self.assertEqual(planner.nodes[1].ports[1],'tried')

    def test_partial_or_nan_scan_never_claims_unknown_space_is_free(self):
        scan=S(ranges=[float('nan')]*360,range_min=.12,range_max=3.5,angle_min=-math.pi,angle_increment=math.pi/180)
        self.assertIsNone(ScanView(scan,[0.,0.],0.).clear([.5,0.]))

    def test_pp_looks_along_polyline_and_turns_to_correct_side(self):
        path=[np.array([0.,0.]),np.array([.4,0.]),np.array([.4,.6])]
        target=pursuit_target(path,np.array([.3,0.]),.25)
        np.testing.assert_allclose(target,[.4,math.sqrt(.25**2-.1**2)])
        self.assertGreater(pure_pursuit(np.array([.3,0.]),0.,target,.2)[1],0.)

    def test_corner_target_does_not_jump_when_nearest_segment_changes(self):
        path=[np.array([0.,0.]),np.array([1.,0.]),np.array([1.,1.])]
        a=pursuit_target(path,np.array([.9,.1-1e-6]),.3)
        b=pursuit_target(path,np.array([.9,.1+1e-6]),.3)
        self.assertLess(np.linalg.norm(a-b),1e-5)
        self.assertAlmostEqual(np.linalg.norm(a-[.9,.1-1e-6]),.3)

    def test_removing_incoming_segment_keeps_the_same_forward_target(self):
        path=[np.array([0.,0.]),np.array([1.,0.]),np.array([1.,1.])]
        position=np.array([.9,.12])
        np.testing.assert_allclose(pursuit_target(path,position,.3),pursuit_target(path[1:],position,.3))

    def test_prediction_matches_measured_straight_and_turning_motion(self):
        point,heading=predict_pose([0.,0.],math.pi/2,.2,0.,.35)
        np.testing.assert_allclose(point,[0.,.07],atol=1e-9)
        self.assertAlmostEqual(heading,math.pi/2)
        point,heading=predict_pose([0.,0.],0.,.2,1.,.35)
        np.testing.assert_allclose(point,[.2*math.sin(.35),.2*(1-math.cos(.35))])
        self.assertAlmostEqual(heading,.35)

    def test_reversed_path_requests_alignment_before_return(self):
        target=pursuit_target([np.array([1.,1.]),np.array([1.,0.])],np.array([1.,.95]),.2)
        angle=pure_pursuit(np.array([1.,.95]),math.pi/2,target,.2)[2]
        self.assertGreater(abs(angle),2.8)

    def test_imu_rate_feedback_brakes_overshoot_and_obeys_limit(self):
        self.assertLess(yaw_rate_command(.2,1.,1.2,1.5),0.)
        self.assertGreater(yaw_rate_command(.8,0.,1.2,1.5),.8)
        self.assertEqual(yaw_rate_command(1.5,-1.,1.2,1.5),1.5)

    def test_delayed_chassis_settles_without_repeated_large_sway(self):
        def run(gain):
            point=np.array([0.,.1]);heading=.2;rate=0.;errors=[]
            for _ in range(240):
                predicted,ph=predict_pose(point,heading,.2,rate,.1)
                target=pursuit_target([np.array([0.,0.]),np.array([10.,0.])],predicted,.28)
                linear,desired,_=pure_pursuit(predicted,ph,target,.2)
                command=yaw_rate_command(desired,rate,gain,1.5)
                rate+=(command-rate)*.05/.45
                point,heading=predict_pose(point,heading,linear,rate,.05)
                errors.append(abs(point[1]))
            return max(errors[80:]),errors[-1]
        raw,_=run(0.);damped,last=run(1.2)
        self.assertLess(damped,.65*raw)
        self.assertLess(last,.001)

    def test_ordinary_right_angle_corners_do_not_switch_to_stop_and_align(self):
        for side in (-1,1):
            state=self.tracking_state([[0.,0.],[1.,0.],[1.,side]], [.98,side*.02],0.)
            linear,angular=PolylineFollower.track_path(state)
            self.assertAlmostEqual(linear,.35);self.assertGreater(side*angular,0.)
            self.assertFalse(state.aligning);self.assertEqual(state.state,'TRACK_PP')

    def test_short_new_path_limits_turn_rate_without_collapsing_forward_speed(self):
        state=self.tracking_state([[0.,0.],[.04,.08]],[0.,0.],0.)
        state.odom.twist.twist.linear.x=0.
        linear,angular=PolylineFollower.track_path(state)
        self.assertAlmostEqual(linear,.35);self.assertAlmostEqual(angular,2.4)

    def test_true_reversal_still_aligns_in_place_then_resumes_forward_tracking(self):
        state=self.tracking_state([[1.,0.],[0.,0.]],[.9,0.],0.)
        self.assertEqual(PolylineFollower.track_path(state)[0],0.)
        self.assertTrue(state.aligning)
        state.heading=math.pi-.2
        self.assertAlmostEqual(PolylineFollower.track_path(state)[0],.35)
        self.assertFalse(state.aligning)

    def test_selected_short_branch_is_extended_before_switching_into_it(self):
        planner=Explorer();view=BranchView()
        planner.update(view,np.zeros(2));planner.update(view,np.array([.8,0.]))
        selected=planner.next_leg;selected.end=np.array([1.,.2]);selected.terminal=False
        before=(len(planner.nodes),len(planner.edges),selected.edge_id)
        planner.refresh_pending_path(view,np.array([.8,0.]))
        np.testing.assert_allclose(planner.path()[-1],[1.,1.])
        self.assertTrue(selected.terminal)
        self.assertEqual((len(planner.nodes),len(planner.edges),selected.edge_id),before)


if __name__=='__main__':unittest.main()
