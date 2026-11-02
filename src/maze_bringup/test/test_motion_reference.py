from pathlib import Path
import sys
import unittest

import numpy as np

sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from motion_reference import MotionReference, ReferenceBuilder, arc_length, project, resample


class MotionReferenceTests(unittest.TestCase):
    def builder(self,cells=None):
        builder=ReferenceBuilder()
        builder.set_map(np.zeros((160,160),dtype=int) if cells is None else cells,.025,[-2.,-2.])
        return builder

    def test_straight_reference_has_no_curvature_and_keeps_cruise(self):
        reference=MotionReference(np.array([[0.,0.],[2.,0.]]),.53,False)
        self.assertLess(np.max(np.abs(reference.k_track)),1e-8)
        np.testing.assert_allclose(reference.speed,.53)
        self.assertAlmostEqual(project(reference.xy,np.array([.7,.1]))[0],.7)

    def test_terminal_braking_is_continuous_and_respects_deceleration(self):
        ref=MotionReference(np.array([[0.,0.],[1.,0.]]),.6,True)
        self.assertEqual(ref.speed[-1],0.)
        self.assertTrue(np.all(ref.speed[:-1]**2<=ref.speed[1:]**2+2*ref.deceleration*np.diff(ref.s)+1e-7))
        self.assertAlmostEqual(ref.speed[0],.6)

    def test_dual_curvature_speed_never_exceeds_lateral_or_wheel_limits(self):
        angle=np.linspace(0.,np.pi/2,100)
        ref=MotionReference(np.column_stack((.3*np.sin(angle),.3*(1-np.cos(angle)))),.65,False)
        self.assertLess(np.max(ref.speed),.65)
        self.assertTrue(np.all(ref.speed*(1+.08*np.abs(ref.k_track))<=ref.wheel_limit+1e-6))
        self.assertTrue(np.all(np.abs(ref.k_track*ref.speed)<=ref.omega_limit+1e-6))

    def test_horizon_uses_measured_velocity_and_does_not_extend_beyond_endpoint(self):
        ref=MotionReference(np.array([[0.,0.],[.3,0.]]),.53,True)
        points=ref.horizon(np.zeros(2),0.,40,.05)
        self.assertEqual(points.shape,(41,5))
        self.assertLessEqual(points[0,3],ref.acceleration*.05+1e-6)
        self.assertLessEqual(points[:,0].max(),.3+1e-6)

    def test_cubic_candidate_stays_in_the_selected_topology_tube(self):
        builder=self.builder()
        ref=builder.build([[-1.,0.],[0.,0.],[0.,1.]],[-.9,0.],.53)
        self.assertIsNotNone(ref)
        self.assertIn('BSPLINE',builder.report['status'])
        self.assertTrue(all(builder.grid.line_safe(a,b) for a,b in zip(ref.xy[:-1],ref.xy[1:])))
        self.assertTrue(np.all(np.minimum(np.abs(ref.xy[:,0]),np.abs(ref.xy[:,1]))<=.2))

    def test_unknown_map_does_not_create_a_free_optimized_route(self):
        builder=self.builder(np.full((160,160),-1))
        self.assertIsNone(builder.build([[0.,0.],[1.,0.]],[0.,0.],.53))
        self.assertEqual(builder.report['status'],'NO_KNOWN_CORRIDOR')

    def test_narrow_l_corner_candidate_is_fully_collision_checked(self):
        rows,cols=np.indices((160,160));x=-2+(cols+.5)*.025;y=-2+(rows+.5)*.025
        free=((x>-1.5)&(x<.3)&(np.abs(y)<.3))|((np.abs(x)<.3)&(y>0)&(y<1.5))
        builder=self.builder(np.where(free,0,100))
        ref=builder.build([[-1.,0.],[0.,0.],[0.,1.]],[-.9,0.],.53)
        self.assertIsNotNone(ref)
        self.assertTrue(all(builder.grid.line_safe(a,b) for a,b in zip(ref.xy[:-1],ref.xy[1:])))

    def test_reuse_is_not_cumulative_smoothing_and_branch_change_rebuilds(self):
        builder=self.builder();path=[[-1.,0.],[0.,0.],[0.,1.]]
        first=builder.build(path,[-.9,0.],.53)
        second=builder.build(path,[-.88,0.],.53)
        self.assertIs(first,second)
        changed=builder.build([[-1.,0.],[0.,0.],[0.,-1.]],[-.85,0.],.53)
        self.assertIsNot(first,changed)
        self.assertLess(changed.xy[-1,1],0.)

    def test_local_window_boundary_does_not_become_a_false_final_goal(self):
        builder=self.builder();builder.set_map(np.zeros((160,320)),.025,[-2.,-2.])
        first=builder.build([[0.,0.],[5.,0.]],[0.,0.],.53,True)
        second=builder.build([[0.,0.],[5.,0.]],[.01,0.],.53,True)
        self.assertIs(first,second)
        self.assertFalse(first.stop_at_end)
        self.assertGreater(first.speed[-1],.5)

    def test_unknown_tail_shortens_window_without_fabricating_a_wall(self):
        cells=np.zeros((160,160),dtype=int);cells[:,104:]=-1
        builder=self.builder(cells)
        ref=builder.build([[0.,0.],[1.5,0.]],[0.,0.],.53)
        self.assertIsNotNone(ref)
        self.assertTrue(builder.report['observable_window_clipped'])
        self.assertLess(ref.xy[-1,0],.6)
        self.assertGreater(ref.speed[0],.5)

    def test_segment_extension_keeps_committed_curve_safe_and_branch_changes_do_not(self):
        builder=self.builder();position=np.array([-.25,0.])
        old=builder.build([[-1.,0.],[0.,0.],[0.,1.]],position,.53)
        old_s,_=project(old.xy,position);point=old.sample(old_s+.08)[0]
        new=builder.build([[-.5,0.],[0.,0.],[0.,1.5]],position,.53)
        self.assertTrue(builder.report['committed_prefix'])
        s,_=project(new.xy,position)
        self.assertLess(np.linalg.norm(new.sample(s+.08)[0]-point),.025)
        self.assertTrue(all(builder.grid.line_safe(a,b) for a,b in zip(new.xy[:-1],new.xy[1:])))
        builder.build([[-.5,0.],[0.,0.],[0.,-1.5]],position,.53)
        self.assertFalse(builder.report['committed_prefix'])


if __name__=='__main__':unittest.main()
