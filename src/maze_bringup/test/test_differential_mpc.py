import math
from pathlib import Path
import sys
import unittest

import numpy as np

sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from differential_mpc import DifferentialMPC, linearize, model, limit_wheel_command
from motion_reference import MotionReference
from polyline_core import Frame2D


class DifferentialMPCTests(unittest.TestCase):
    def test_linearization_matches_nonlinear_differential_model(self):
        state=np.array([.3,-.2,.7,.45,1.2]);control=np.array([.5,1.5])
        A,B,c=linearize(state,control);epsilon=1e-6
        numeric_a=np.column_stack([(model(state+np.eye(5)[i]*epsilon,control)-
                                     model(state-np.eye(5)[i]*epsilon,control))/(2*epsilon) for i in range(5)])
        numeric_b=np.column_stack([(model(state,control+np.eye(2)[i]*epsilon)-
                                     model(state,control-np.eye(2)[i]*epsilon))/(2*epsilon) for i in range(2)])
        np.testing.assert_allclose(A,numeric_a,atol=1e-7)
        np.testing.assert_allclose(B,numeric_b,atol=1e-7)
        np.testing.assert_allclose(A@state+B@control+c,model(state,control),atol=1e-9)

    def test_straight_startup_progresses_and_obeys_both_wheel_accelerations(self):
        mpc=DifferentialMPC();ref=MotionReference(np.array([[0.,0.],[3.,0.]]),.53,False)
        horizon=ref.horizon(np.zeros(2),0.)
        command,states,report=mpc.solve(horizon,np.zeros(5),[0.,0.])
        self.assertIsNotNone(command,report)
        self.assertGreater(command[0],.01)
        controls=mpc.previous_controls
        wheels=np.column_stack((controls[:,0]-.08*controls[:,1],controls[:,0]+.08*controls[:,1]))
        self.assertLessEqual(np.max(np.abs(np.diff(np.vstack((np.zeros(2),wheels)),axis=0))),.9*.05+.003)
        self.assertLess(np.abs(command[1]),.01)
        self.assertGreater(states[-1,0],.1)

    def test_closed_loop_recovers_lateral_error_with_zero_lateral_actuation(self):
        mpc=DifferentialMPC();ref=MotionReference(np.array([[0.,0.],[4.,0.]]),.53,False)
        state=np.array([0.,.08,0.,.3,0.]);previous=np.array([.3,0.]);failures=[]
        for _ in range(40):
            command,_,_,report=mpc.compute(ref,state[:2],state[:2],state[2],Frame2D(np.zeros(2),0.),state[3],state[4],previous)
            if command is None:failures.append(report);break
            self.assertEqual(len(command),2)
            state=model(state,command);previous=command
        self.assertFalse(failures,failures)
        self.assertLess(abs(state[1]),.025)
        self.assertGreater(state[0],.6)

    def test_curved_reference_tracks_without_holonomic_side_velocity(self):
        angle=np.linspace(0.,math.pi/2,100)
        ref=MotionReference(np.column_stack((.5*np.sin(angle),.5*(1-np.cos(angle)))),.53,True)
        mpc=DifferentialMPC();state=np.array([0.,0.,0.,.2,0.]);previous=np.array([.2,0.])
        for _ in range(25):
            command,_,_,report=mpc.compute(ref,state[:2],state[:2],state[2],Frame2D(np.zeros(2),0.),state[3],state[4],previous)
            self.assertIsNotNone(command,report)
            state=model(state,command);previous=command
        self.assertGreater(state[0],.25)
        self.assertLess(abs(np.linalg.norm(state[:2]-[0.,.5])-.5),.06)

    def test_zero_command_model_cannot_translate_sideways(self):
        state=model(np.array([0.,0.,0.,.3,0.]),[.3,0.])
        self.assertAlmostEqual(state[1],0.)
        self.assertGreater(state[0],0.)

    def test_invalid_reference_size_is_rejected(self):
        with self.assertRaises(ValueError):DifferentialMPC().solve(np.zeros((3,5)),np.zeros(5),[0.,0.])

    def test_pp_fallback_is_limited_in_wheel_space(self):
        previous=(.6,1.5)
        for _ in range(20):
            command=limit_wheel_command((.25,-3.),previous)
            for sign in (-1,1):
                change=(command[0]+sign*.08*command[1])-(previous[0]+sign*.08*previous[1])
                self.assertLessEqual(abs(change),.9*.05+1e-9)
            previous=command
        self.assertLess(command[0],.3)


if __name__=='__main__':unittest.main()
