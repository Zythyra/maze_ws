import os
from pathlib import Path
import pty
import sys
import unittest

import rclpy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from std_srvs.srv import SetBool

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
from terminal_start import TerminalStartGate
from polyline_follower import PolylineFollower


class TerminalStartTests(unittest.TestCase):
    def setUp(self):
        self.master, self.slave = pty.openpty()
        self.messages = []
        self.gate = TerminalStartGate(os.ttyname(self.slave), self.messages.append)

    def tearDown(self):
        self.gate.close()
        os.close(self.master)
        os.close(self.slave)

    def test_requires_complete_start_line_and_is_one_shot(self):
        self.assertFalse(self.gate.poll())
        os.write(self.master, b'start')
        self.assertFalse(self.gate.poll())
        os.write(self.master, b'\n')
        self.assertTrue(self.gate.poll())
        self.assertIsNone(self.gate.fd)
        self.assertFalse(self.gate.poll())

    def test_invalid_input_keeps_waiting_then_start_works(self):
        os.write(self.master, b'go\nrestart\nSTART\n')
        self.assertFalse(self.gate.poll())
        os.write(self.master, b'  start  \n')
        self.assertTrue(any(self.gate.poll() for _ in range(4)))

    def test_missing_terminal_never_starts_automatically(self):
        gate = TerminalStartGate('/no/such/maze-terminal', self.messages.append)
        self.assertFalse(gate.poll())
        self.assertIsNone(gate.fd)

    def test_terminal_disconnect_keeps_gate_closed(self):
        os.close(self.master)
        self.master = os.open('/dev/null', os.O_RDONLY)
        self.assertFalse(self.gate.poll())
        self.assertIsNone(self.gate.fd)


class FollowerStartTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def run_node(self, mode='pp', wait=True):
        # Constructor overrides remain local; no global process arguments leak
        # into other controller regression tests.
        from unittest.mock import patch
        original = PolylineFollower.declare_parameter
        overrides = dict(controller_mode=mode, wait_for_start=wait,
                         terminal_device=os.ttyname(self.slave))
        def declare(node, name, value, *args, **kwargs):
            return original(node, name, overrides.get(name, value), *args, **kwargs)
        with patch.object(PolylineFollower, 'declare_parameter', declare):
            node = PolylineFollower()
        self.addCleanup(node.destroy_node)
        self.addCleanup(node.return_executor.shutdown, wait=False, cancel_futures=True)
        return node

    def setUp(self):
        self.master, self.slave = pty.openpty()
        self.addCleanup(os.close, self.master)
        self.addCleanup(os.close, self.slave)

    def test_all_modes_hold_zero_with_fresh_sensors_until_start(self):
        for mode in ('pp', 'optimized_pp', 'mpc'):
            with self.subTest(mode=mode):
                node = self.run_node(mode)
                node.on_odom(Odometry())
                node.on_scan(LaserScan())
                node.control()
                self.assertEqual(node.state, 'WAIT_START')
                self.assertFalse(node.enabled)
                self.assertIsNone(node.planner.leg)
                self.assertIsNone(node.home_position)
                self.assertEqual(node.command, (0., 0.))
                os.write(self.master, b'start\n')
                node.scan = None
                node.control()
                self.assertTrue(node.enabled)
                self.assertFalse(node.waiting_for_start)
                self.assertEqual(node.state, 'WAIT_SENSORS')
                self.assertIsNone(node.start_gate.fd)

    def test_explicit_automatic_mode_keeps_previous_behavior(self):
        node = self.run_node(wait=False)
        node.control()
        self.assertTrue(node.enabled)
        self.assertEqual(node.state, 'WAIT_SENSORS')
        self.assertIsNone(node.start_gate)

    def test_service_start_closes_terminal_and_cannot_restart_finished_mission(self):
        node = self.run_node()
        response = node.set_enabled(SetBool.Request(data=True), SetBool.Response())
        self.assertTrue(response.success)
        self.assertFalse(node.waiting_for_start)
        self.assertIsNone(node.start_gate.fd)
        node.set_enabled(SetBool.Request(data=False), SetBool.Response())
        os.write(self.master, b'start\n')
        node.control()
        self.assertEqual(node.state, 'PAUSED')
        node.mission_phase = 'HOME_REACHED'
        response = node.set_enabled(SetBool.Request(data=True), SetBool.Response())
        self.assertFalse(response.success)
        self.assertEqual(node.command, (0., 0.))
