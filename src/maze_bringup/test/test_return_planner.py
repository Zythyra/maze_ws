import json
import math
from pathlib import Path
import sys
from types import SimpleNamespace as S
import unittest

import numpy as np

sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from return_planner import ReturnGrid, ReturnPlan
from polyline_follower import PolylineFollower
from polyline_core import Frame2D


class ReturnPlannerTests(unittest.TestCase):
    def test_open_map_reaches_exact_endpoints(self):
        grid=ReturnGrid(np.zeros((30,50)),.1,[0.,0.],wall_cost=0.)
        plan=grid.plan([1.,1.],[3.,1.])
        self.assertTrue(plan.points);self.assertGreater(plan.expanded,0)
        np.testing.assert_allclose(plan.points[0],[1.,1.]);np.testing.assert_allclose(plan.points[-1],[3.,1.])
        self.assertLess(plan.length,2.1)

    def test_astar_uses_a_gap_instead_of_crossing_a_wall(self):
        cells=np.zeros((40,50));cells[:,25]=100;cells[25:34,25]=0
        grid=ReturnGrid(cells,.1,[0.,0.],wall_cost=0.)
        plan=grid.plan([1.,1.],[4.,1.])
        self.assertTrue(plan.points);self.assertGreater(plan.length,4.)
        self.assertTrue(all(grid.line_safe(a,b) for a,b in zip(plan.points[:-1],plan.points[1:])))
        self.assertTrue(any(p[1]>2.5 for p in plan.points))

    def test_unknown_strip_cannot_be_used_as_a_shortcut(self):
        cells=np.zeros((30,40));cells[:,20]=-1
        plan=ReturnGrid(cells,.1,[0.,0.]).plan([1.,1.],[3.,1.])
        self.assertEqual(plan.points,[])

    def test_diagonal_motion_cannot_cut_between_two_blocked_corners(self):
        grid=ReturnGrid([[0,100],[100,0]],1.,[0.,0.],clearance=0.)
        self.assertEqual(grid.plan([.5,.5],[1.5,1.5]).points,[])

    def test_inflation_rejects_a_passage_narrower_than_the_robot_margin(self):
        cells=np.zeros((20,40));cells[5,:]=100;cells[9,:]=100
        self.assertTrue(ReturnGrid(cells,.1,[0.,0.],clearance=.1).plan([1.,.75],[3.,.75]).points)
        self.assertFalse(ReturnGrid(cells,.1,[0.,0.],clearance=.22).plan([1.,.75],[3.,.75]).points)

    def test_rotated_map_origin_preserves_world_endpoints(self):
        frame=Frame2D(np.array([-2.,5.]),math.pi/2)
        grid=ReturnGrid(np.zeros((30,40)),.1,frame.translation,frame.yaw)
        start,home=frame.apply([1.,1.]),frame.apply([3.,1.])
        plan=grid.plan(start,home)
        self.assertTrue(plan.points);np.testing.assert_allclose(plan.points[-1],home)

    def test_current_laser_connects_observed_exterior_beyond_slam_crop(self):
        cells=np.zeros((20,20),dtype=np.int16)
        scan=S(ranges=[float('inf')]*360,range_min=.12,range_max=3.5,
               angle_min=0.,angle_increment=math.pi/180)
        view=S(scan=scan,origin=np.array([2.3,1.]),yaw=0.)
        self.assertFalse(ReturnGrid(cells,.1,[0.,0.]).plan([2.3,1.],[.5,1.]).points)
        grid=ReturnGrid(cells,.1,[0.,0.],observation=view)
        self.assertTrue(grid.plan([2.3,1.],[.5,1.]).points)
        self.assertTrue(np.all(cells==0))

    def test_ray_overlay_keeps_mapped_walls_and_unknown_behind_hits(self):
        cells=np.full((20,20),-1);cells[:,10]=100
        scan=S(ranges=[.5]*360,range_min=.12,range_max=3.5,
               angle_min=0.,angle_increment=math.pi/180)
        view=S(scan=scan,origin=np.array([.5,1.]),yaw=0.)
        grid=ReturnGrid(cells,.1,[0.,0.],observation=view)
        self.assertEqual(grid.cells[grid.cell([1.05,1.])],100)
        self.assertEqual(grid.cells[grid.cell([1.35,1.])],-1)

    def test_expansion_limit_does_not_return_a_partial_success(self):
        grid=ReturnGrid(np.zeros((30,50)),.1,[0.,0.])
        plan=grid.plan([.5,.5],[4.,2.],max_expansions=2)
        self.assertFalse(plan.points);self.assertIn('limit',plan.reason)

    def test_occupied_home_is_reported_as_unreachable(self):
        cells=np.zeros((30,40));cells[10,30]=100
        plan=ReturnGrid(cells,.1,[0.,0.]).plan([1.,1.],[3.05,1.05])
        self.assertFalse(plan.points);self.assertIn('home',plan.reason)

    def test_home_arrival_is_a_latched_final_success(self):
        events=[]
        state=S(home_position=np.zeros(2),position=np.array([.04,.03]),mission_phase='RETURNING',
                p=dict(return_goal_tolerance=.1),frame='map',
                result_pub=S(publish=lambda m:events.append(json.loads(m.data))),
                get_logger=lambda:S(info=lambda text:None))
        self.assertEqual(PolylineFollower.control_return(state,10.),(0.,0.))
        self.assertEqual(state.mission_phase,'HOME_REACHED');self.assertTrue(events[-1]['finished'])

    def test_failed_plan_stays_stopped_before_retry(self):
        state=S(home_position=np.zeros(2),position=np.array([1.,0.]),mission_phase='PLAN_RETURN',
                p=dict(return_goal_tolerance=.1,return_retry_period=2.),
                return_future=S(done=lambda:True,result=lambda:ReturnPlan([],20,0.,'no route')),
                return_ready_at=0.,return_next_plan=0.,get_logger=lambda:S(warn=lambda text:None))
        self.assertEqual(PolylineFollower.control_return(state,10.),(0.,0.))
        self.assertEqual(state.mission_phase,'RETURN_NO_PATH');self.assertEqual(state.return_next_plan,12.)

    def test_return_can_be_paused_and_resumed_without_clearing_route(self):
        state=S(mission_phase='RETURNING',exit_detector=S(reset_candidate=lambda:None),
                publish_command=lambda v,w:None,return_path=[np.zeros(2),np.ones(2)])
        response=S();PolylineFollower.set_enabled(state,S(data=False),response)
        self.assertTrue(response.success);self.assertFalse(state.enabled)
        PolylineFollower.set_enabled(state,S(data=True),response)
        self.assertTrue(response.success);self.assertTrue(state.enabled)
        self.assertEqual(len(state.return_path),2);self.assertEqual(state.mission_phase,'RETURNING')
