import math
from pathlib import Path
import sys
from types import SimpleNamespace as S
import unittest

import cv2
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
from branch_map import BranchMap
from polyline_core import Explorer, Frame2D, Junction, pursuit_target
from polyline_follower import PolylineFollower
from nav_msgs.msg import OccupancyGrid


RES = .025
ORIGIN = np.array([-2., -2.])
ROWS, COLS = np.indices((160, 160))
X, Y = ORIGIN[0] + (COLS + .5) * RES, ORIGIN[1] + (ROWS + .5) * RES


def box(x0, x1, y0, y1):
    return (X > x0) & (X < x1) & (Y > y0) & (Y < y1)


def scene(extra=()):
    free = box(-1.5, 1.5, -.3, .3) | box(-.3, .3, -1.3, 0.)
    for bounds in extra:
        free |= box(*bounds)
    # Unknown outside observed wall surfaces, just as in a SLAM map.
    walls = cv2.dilate(free.astype(np.uint8), np.ones((5, 5), dtype=np.uint8)) != 0
    cells = np.full(free.shape, -1, dtype=np.int16)
    cells[walls] = 100
    cells[free] = 0
    return cells


class BranchMapTests(unittest.TestCase):
    def grid(self, cells):
        return BranchMap(cells, RES, ORIGIN)

    def test_fully_mapped_right_pocket_is_proven_without_entering(self):
        cells = scene()
        proof = self.grid(cells).check([0., 0.], 3)
        self.assertTrue(proof.dead_end)
        self.assertGreater(proof.cells, 500)
        np.testing.assert_array_equal(cells, scene())

    def test_unknown_inside_visible_dead_end_prevents_pruning(self):
        cells = scene(); cells[box(.05, .15, -.95, -.85)] = -1
        proof = self.grid(cells).check([0., 0.], 3)
        self.assertFalse(proof.dead_end)
        self.assertIn('unknown', proof.reason)

    def test_far_wall_with_hidden_left_or_right_opening_is_kept(self):
        for side in (-1, 1):
            cells = scene([(-1.3, 0., -1.3, -.7) if side < 0 else (0., 1.3, -1.3, -.7)])
            cells[box(-1.3, -.95, -1.25, -.75) if side < 0 else box(.95, 1.3, -1.25, -.75)] = -1
            self.assertFalse(self.grid(cells).check([0., 0.], 3).dead_end)

    def test_narrow_connection_to_unknown_is_not_removed_by_inflation(self):
        cells = scene()
        cells[box(.29, .55, -.99, -.96)] = 0
        cells[box(.53, .58, -1.02, -.93)] = -1
        self.assertFalse(self.grid(cells).check([0., 0.], 3).dead_end)

    def test_loop_back_to_parent_corridor_is_kept(self):
        cells = scene([(0., 1.3, -1.3, -.7), (.7, 1.3, -1.3, 0.)])
        proof = self.grid(cells).check([0., 0.], 3)
        self.assertFalse(proof.dead_end)
        self.assertIn('reconnects', proof.reason)

    def test_fully_observed_bent_pocket_can_also_be_skipped(self):
        cells = scene([(0., 1.3, -1.3, -.7)])
        self.assertTrue(self.grid(cells).check([0., 0.], 3).dead_end)

    def test_open_exterior_and_map_boundary_are_kept(self):
        cells = scene([(-.3, .3, -2., 0.)])
        proof = self.grid(cells).check([0., 0.], 3)
        self.assertFalse(proof.dead_end)
        self.assertIn('boundary', proof.reason)

    def test_unknown_neck_and_absent_side_wall_are_kept(self):
        cells = scene(); cells[box(-.1, .1, -.5, -.25)] = -1
        self.assertFalse(self.grid(cells).check([0., 0.], 3).dead_end)
        self.assertFalse(self.grid(np.zeros((160, 160), dtype=np.int16)).check([0., 0.], 3).dead_end)

    def test_rotated_map_origin_and_all_four_headings(self):
        for turns in range(4):
            yaw = turns * math.pi / 2
            transform = Frame2D(np.array([3., -4.]), yaw)
            grid = BranchMap(scene(), RES, transform.apply(ORIGIN), yaw)
            self.assertTrue(grid.check(transform.apply([0., 0.]), (3 + turns) % 4).dead_end)

    def planner(self):
        planner = Explorer()
        planner.nodes = [Junction(np.zeros(2), None, {0:'open', 3:'open'})]
        planner.new_edge(0, 0, np.array([1., 0.]), False)
        right = planner.new_edge(0, 3, np.array([0., -1.]), True)
        cells = scene(); cells[box(1., 1.1, -.1, .1)] = -1
        planner.set_branch_map(self.grid(cells))
        return planner, right

    def test_branch_id_preserved_and_selection_skips_mapped_dead_end(self):
        planner, right = self.planner()
        self.assertEqual(planner.untried(0), [0])
        self.assertEqual(planner.nodes[0].branches[3], right.edge_id)
        self.assertFalse(right.visited)
        self.assertTrue(right.map_dead_end)
        self.assertEqual(planner.select_branch(0, 0).heading, 0)
        graph = planner.graph_report()
        self.assertTrue(graph['edges'][right.edge_id]['map_dead_end'])

    def test_new_map_with_unknown_or_opening_revives_same_branch(self):
        planner, right = self.planner()
        cells = scene(); cells[box(-.1, .1, -1., -.9)] = -1
        planner.set_branch_map(self.grid(cells))
        self.assertIn(3, planner.untried(0))
        self.assertFalse(right.map_dead_end)
        self.assertEqual(len(planner.edges), 2)
        planner.set_branch_map(self.grid(scene()))
        self.assertNotIn(3, planner.untried(0))
        planner.set_branch_map(None)
        self.assertIn(3, planner.untried(0))

    def test_visited_or_connected_roads_are_never_pruned(self):
        planner, right = self.planner()
        right.visited = True
        planner.set_branch_map(self.grid(scene()))
        self.assertFalse(right.map_dead_end)
        right.visited = False
        planner.untried(0)
        self.assertTrue(right.map_dead_end)
        right.destination = 0
        planner.set_branch_map(self.grid(scene()))
        self.assertFalse(right.map_dead_end)

    def test_backtracking_ignores_nearest_node_if_only_branch_is_proven_dead(self):
        planner = Explorer()
        points = ([-1., 0.], [0., 0.], [1., 0.], [1., 1.])
        planner.nodes = [Junction(np.array(point), None, {}) for point in points]
        for source, dest, heading in ((0, 1, 0), (1, 2, 0), (2, 3, 1)):
            edge = planner.new_edge(source, heading, planner.nodes[dest].point, True)
            edge.destination = dest; edge.visited = True
            planner.nodes[source].neighbors[heading] = dest
            planner.nodes[dest].neighbors[(heading+2)%4] = source
            planner.nodes[dest].branches[(heading+2)%4] = edge.edge_id
        planner.new_edge(1, 3, np.array([0., -1.]), True)
        planner.nodes[1].ports[3] = 'open'
        planner.new_edge(0, 1, np.array([-1., 1.]), False)
        planner.nodes[0].ports[1] = 'open'
        planner.set_branch_map(self.grid(scene()))
        planner.start_return(3)
        self.assertEqual(planner.return_target, 0)
        self.assertEqual(planner.last_return_edges, [2, 1, 0])

    def test_map_received_before_first_graph_node_is_used(self):
        state = S(frame='map', mission_phase='EXPLORING', planner=Explorer(),
                  p={'map_dead_end_pruning':True})
        message = OccupancyGrid(); message.header.frame_id = 'map'
        message.info.width = message.info.height = 160
        message.info.resolution = RES
        message.info.origin.position.x, message.info.origin.position.y = ORIGIN
        message.info.origin.orientation.w = 1.
        message.data = scene().ravel().tolist()
        PolylineFollower.on_map(state, message)
        self.assertIsNotNone(state.planner.branch_map)
        self.assertTrue(state.planner.branch_map.check([0., 0.], 3).dead_end)

    def forward_fixture(self, source=(.1, 0.)):
        cells = scene([(-1.3, -.7, 0., 1.3), (0., 1.9, -.3, .3)])
        cells[box(-1.15, -.85, .9, 1.2)] = -1
        planner = Explorer()
        planner.nodes = [Junction(np.array([-1., 0.]), None, {0:'tried', 1:'open'}),
                         Junction(np.array(source), 0, {0:'tried', 2:'tried'})]
        incoming = planner.new_edge(0, 0, np.array(source), True)
        incoming.destination = 1; incoming.visited = True
        planner.nodes[0].neighbors[0] = 1
        planner.nodes[1].neighbors[2] = 0
        planner.nodes[1].branches[2] = incoming.edge_id
        planner.new_edge(0, 1, np.array([-1., 1.]), False)
        planner.leg = planner.new_edge(1, 0, np.array([1.6, 0.]), True)
        planner.leg.visited = True
        planner.set_branch_map(self.grid(cells))
        return planner, cells

    def quiet_view(self):
        return S(opening=lambda *args:None, right_opening=lambda *args:None,
                 ports=lambda *args:{i:'unknown' for i in range(4)},
                 line_end=lambda anchor,*args:(np.asarray(anchor)+[2., 0.],False))

    def test_active_visited_straight_road_returns_far_before_preview(self):
        planner, _ = self.forward_fixture()
        original = planner.leg.edge_id; position = np.array([.3, 0.])
        planner.update(self.quiet_view(), position, 0.)
        self.assertEqual(planner.map_forward_turnbacks, 1)
        self.assertTrue(planner.leg.returning)
        self.assertEqual(planner.return_target, 0)
        self.assertEqual(planner.last_return_edges, [original, 0])
        self.assertGreater(planner.last_map_turnback['distance_to_tip_m'], 1.)
        self.assertLess(pursuit_target(planner.path(), position, .32)[0], position[0])
        self.assertEqual(planner.edges[original].edge_id, original)

    def test_unfinished_laser_ray_also_turns_back_from_map_evidence(self):
        planner, _ = self.forward_fixture(); planner.leg.terminal = False
        planner.update(self.quiet_view(), np.array([.3, 0.]), 0.)
        self.assertEqual(planner.map_forward_turnbacks, 1)
        self.assertTrue(planner.leg.returning)

    def test_unknown_or_side_exit_ahead_does_not_reverse_active_road(self):
        for side_exit in (False, True):
            planner, cells = self.forward_fixture()
            if side_exit:
                cells = scene([(-1.3, -.7, 0., 1.3), (0., 1.9, -.3, .3), (.7, 1.3, -1.3, 0.)])
                cells[box(.85, .95, -.95, -.85)] = -1
            else:cells[box(1., 1.1, -.1, .1)] = -1
            planner.set_branch_map(self.grid(cells))
            planner.update(self.quiet_view(), np.array([.3, 0.]), 0.)
            self.assertEqual(planner.map_forward_turnbacks, 0)
            self.assertFalse(planner.leg.returning)

    def test_unknown_behind_current_projection_does_not_force_driving_to_tip(self):
        planner, cells = self.forward_fixture(source=(-.5, 0.))
        cells[box(-.1, .1, -.95, -.85)] = -1
        planner.set_branch_map(self.grid(cells))
        planner.update(self.quiet_view(), np.array([.8, 0.]), 0.)
        self.assertEqual(planner.map_forward_turnbacks, 1)
        self.assertEqual(planner.last_map_turnback['proof_origin'], 'current projection')

    def test_prefetched_turn_inside_closed_pocket_is_released_and_not_a_frontier(self):
        planner, cells = self.forward_fixture()
        planner.bind_destination(); tip = planner.leg.destination
        pending = planner.new_edge(tip, 1, np.array([1.6, .2]), False)
        planner.nodes[tip].ports[1] = 'tried'; pending.visited = True
        planner.next_leg = pending
        planner.update(self.quiet_view(), np.array([.3, 0.]), 0.)
        self.assertEqual(planner.return_target, 0)
        self.assertFalse(pending.visited)
        self.assertTrue(pending.map_dead_end)
        self.assertNotIn(tip, [leg.destination for leg in planner.return_route])
        planner.set_branch_map(self.grid(cells))
        self.assertNotIn(1, planner.untried(tip))
        cells[box(1., 1.1, -.1, .1)] = -1
        planner.set_branch_map(self.grid(cells))
        self.assertIn(1, planner.untried(tip))

    def test_backtracking_and_turning_are_not_interrupted_by_forward_check(self):
        planner, _ = self.forward_fixture()
        planner.leg.returning = True
        self.assertFalse(planner.return_from_mapped_dead_end(np.array([.3, 0.]), 0.))
        planner.leg.returning = False
        self.assertFalse(planner.return_from_mapped_dead_end(np.array([.3, 0.]), math.pi/2))
        self.assertEqual(planner.map_forward_turnbacks, 0)

    def test_disabled_map_check_keeps_normal_forward_motion(self):
        planner, _ = self.forward_fixture(); planner.set_branch_map(None)
        planner.update(self.quiet_view(), np.array([.3, 0.]), 0.)
        self.assertFalse(planner.leg.returning)
        self.assertEqual(planner.map_forward_turnbacks, 0)

    def test_same_source_frontier_is_prepared_without_reentering_dead_road(self):
        planner, _ = self.forward_fixture()
        branch = planner.new_edge(1, 1, np.array([.1, .8]), False)
        planner.nodes[1].ports[1] = 'open'
        planner.update(self.quiet_view(), np.array([.3, 0.]), 0.)
        self.assertEqual(planner.return_target, 1)
        self.assertIs(planner.next_leg, branch)
        self.assertFalse(branch.returning)

    def test_forward_pocket_cut_follows_slam_pose_graph_correction(self):
        planner, _ = self.forward_fixture()
        planner.slam.poses = {0:np.array([0., 0.]), 1:np.array([1., 0.])}
        planner.update(self.quiet_view(), np.array([.3, 0.]), 0.)
        cut, _, _ = next(iter(planner.forward_dead_cuts.values()))
        planner.update_pose_graph({0:np.array([.2, -.1]), 1:np.array([1.2, -.1])})
        shifted, _, _ = next(iter(planner.forward_dead_cuts.values()))
        np.testing.assert_allclose(shifted, cut+[.2, -.1])

    def test_connected_road_inside_pocket_is_kept_but_not_an_exploration_target(self):
        planner, _ = self.forward_fixture()
        planner.bind_destination(); tip = planner.leg.destination
        edge = planner.new_edge(tip, 1, np.array([1.6, .1]), False)
        edge.destination = tip; planner.nodes[tip].ports[1] = 'open'
        planner.update(self.quiet_view(), np.array([.3, 0.]), 0.)
        self.assertEqual(planner.untried(tip), [])
        self.assertFalse(edge.retired)
        self.assertFalse(edge.visited)
        self.assertEqual(planner.return_target, 0)


if __name__ == '__main__':
    unittest.main()
