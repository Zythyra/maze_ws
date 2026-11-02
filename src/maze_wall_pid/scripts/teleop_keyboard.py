#!/usr/bin/env python3
"""Dead-man keyboard teleop: hold WASD, release to stop; Q or Ctrl-C exits."""
import select
import sys
import termios
import time
import tty

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node


def main():
    if not sys.stdin.isatty():
        raise SystemExit('Run teleop_keyboard.py in an interactive terminal.')
    rclpy.init()
    node = Node('burger_keyboard')
    node.declare_parameter('linear_speed', 0.12)
    node.declare_parameter('angular_speed', 0.6)
    publisher = node.create_publisher(Twist, '/cmd_vel', 10)
    settings = termios.tcgetattr(sys.stdin)
    command = Twist()
    last_key = 0.0
    print('Hold W/S: forward/back; A/D: left/right. Space: stop. Q/Ctrl-C: quit.', flush=True)
    try:
        tty.setcbreak(sys.stdin.fileno())
        while rclpy.ok():
            if select.select([sys.stdin], [], [], 0.05)[0]:
                key = sys.stdin.read(1).lower()
                if key in ('q', '\x03'):
                    break
                command = Twist()
                linear = float(node.get_parameter('linear_speed').value)
                angular = float(node.get_parameter('angular_speed').value)
                if key in ('w', 's'):
                    command.linear.x = linear if key == 'w' else -linear
                elif key in ('a', 'd'):
                    command.angular.z = angular if key == 'a' else -angular
                last_key = time.monotonic()
            if time.monotonic() - last_key > 0.4:
                command = Twist()
            publisher.publish(command)
            rclpy.spin_once(node, timeout_sec=0)
    except KeyboardInterrupt:
        pass
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        if rclpy.ok():
            for _ in range(3):
                publisher.publish(Twist())
                time.sleep(0.02)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
