import math
from pathlib import Path
import sys
from types import SimpleNamespace as S
import unittest

import numpy as np

sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from motion_reference import MotionReference
from polyline_core import Frame2D
from polyline_follower import PolylineFollower


class MotionHandoffTests(unittest.TestCase):
    def state(self,position,heading):
        reference=MotionReference([[0.,.7],[0.,0.],[1.,0.]],.6,False)
        orientation=S(x=0.,y=0.,z=math.sin(heading/2),w=math.cos(heading/2))
        return S(motion_speed=.6,mission_phase='EXPLORING',planner=S(leg=S(terminal=False,returning=False),next_leg=None),
                 motion_builder=S(build=lambda *args,**kwargs:reference,report={},grid=None),
                 motion_mpc=S(reset=lambda:None,compute=lambda *args,**kwargs:(None,[],[],{'solver_status':'primal infeasible'})),
                 motion_recovering=False,position=np.asarray(position),heading=heading,
                 odom=S(twist=S(twist=S(linear=S(x=0.),angular=S(z=0.))),
                        pose=S(pose=S(position=S(x=position[0],y=position[1]),orientation=orientation))),
                 map_from_odom=Frame2D(np.zeros(2),0.),command=(0.,0.),view=None,aligning=False,
                 p=dict(max_linear_speed=.53,prediction_time=.1,max_alignment_speed=1.6,
                        align_enter_angle=2.4,align_exit_angle=.45,max_angular_speed=4.3,yaw_rate_gain=.5))

    def test_uturn_aligns_to_tangent_even_when_lookahead_shortcuts_next_corner(self):
        state=self.state([0.,.2],0.)
        command=PolylineFollower.track_motion_reference(state,[[0.,.7],[0.,0.],[1.,0.]],.53,.32,'TRACK_PP','ALIGN_PATH')
        self.assertEqual(command[0],0.)
        self.assertLess(command[1],0.)
        self.assertTrue(state.motion_recovering)
        self.assertEqual(state.motion_report['controller'],'tangent_align')

    def test_off_path_infeasibility_uses_bounded_recovery_instead_of_full_speed_pp(self):
        state=self.state([.14,.3],-math.pi/2)
        command=PolylineFollower.track_motion_reference(state,[[0.,.7],[0.,0.],[1.,0.]],.53,.32,'TRACK_PP','ALIGN_PATH')
        self.assertLessEqual(command[0],.25)
        self.assertEqual(state.state,'MPC_FALLBACK_PP')

    def test_recovery_resumes_forward_with_29_degree_heading_tolerance(self):
        state=self.state([0.,.35],-math.pi/2+.48)
        state.motion_recovering=True
        command=PolylineFollower.track_motion_reference(state,[[0.,.7],[0.,0.],[1.,0.]],.53,.32,'TRACK_PP','ALIGN_PATH')
        self.assertFalse(state.motion_recovering)
        self.assertGreater(command[0],0.)
        self.assertNotEqual(state.state,'ALIGN_PATH')

    def test_medium_heading_error_does_not_start_another_alignment(self):
        state=self.state([0.,.35],-math.pi/2+.75)
        state.command=(.2,0.);state.odom.twist.twist.linear.x=.2
        command=PolylineFollower.track_motion_reference(state,[[0.,.7],[0.,0.],[1.,0.]],.53,.32,'TRACK_PP','ALIGN_PATH')
        self.assertFalse(state.motion_recovering)
        self.assertGreater(command[0],0.)

    def test_uturn_uses_increased_rotation_limit(self):
        state=self.state([0.,.35],math.pi/2)
        state.p['max_alignment_speed']=2.4
        command=PolylineFollower.track_motion_reference(state,[[0.,.7],[0.,0.],[1.,0.]],.53,.32,'TRACK_PP','ALIGN_PATH')
        self.assertEqual(command[0],0.)
        self.assertAlmostEqual(abs(command[1]),2.4)

    def test_pp_exits_alignment_before_facing_the_path_exactly(self):
        state=self.state([0.,.60],-math.pi/2+.48)
        state.aligning=True;state.p['align_exit_angle']=.50
        command=PolylineFollower.track_reference(state,np.array([[0.,1.],[0.,0.]]),.53,.32,'TRACK_PP','ALIGN_PATH')
        self.assertFalse(state.aligning)
        self.assertGreater(command[0],0.)
