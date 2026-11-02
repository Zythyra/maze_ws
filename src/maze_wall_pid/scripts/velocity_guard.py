#!/usr/bin/env python3
"""Clamp Burger commands and stop when the public cmd_vel stream expires."""
import math
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.clock import Clock, ClockType
from rclpy.node import Node


class VelocityGuard(Node):
    def __init__(self):
        super().__init__('velocity_guard')
        self.linear_limit = self.declare_parameter('max_linear_speed', 0.65).value
        self.angular_limit = self.declare_parameter('max_angular_speed', 4.8).value
        self.timeout = self.declare_parameter('command_timeout', 0.5).value
        self.command = Twist()
        self.received_at = float('-inf')
        self.publisher = self.create_publisher(Twist, '/burger/cmd_vel', 1)
        self.create_subscription(Twist, '/cmd_vel', self.on_command, 1)
        # Wall-clock freshness also expires commands while simulation is paused.
        self.create_timer(0.05, self.tick, clock=Clock(clock_type=ClockType.STEADY_TIME))

    def on_command(self, message):
        command = Twist()
        if math.isfinite(message.linear.x) and math.isfinite(message.angular.z):
            command.linear.x = max(-self.linear_limit, min(self.linear_limit, message.linear.x))
            command.angular.z = max(-self.angular_limit, min(self.angular_limit, message.angular.z))
        self.command = command
        self.received_at = time.monotonic()

    def tick(self):
        self.publisher.publish(self.command if time.monotonic() - self.received_at < self.timeout else Twist())


def main():
    rclpy.init()
    node = VelocityGuard()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
