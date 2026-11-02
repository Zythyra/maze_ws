#!/usr/bin/env python3
"""Check the Burger topics/TF. --drive additionally moves it in an empty world."""
import argparse
import math
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu, JointState, LaserScan
from tf2_ros import Buffer, TransformListener


def yaw(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--drive', action='store_true', help='Run motion tests in an empty world')
    args = parser.parse_args()
    rclpy.init()
    node = rclpy.create_node('burger_smoke_test')
    data = {}
    for topic, msg_type in [('scan', LaserScan), ('odom', Odometry),
                            ('joint_states', JointState), ('imu', Imu), ('burger/cmd_vel', Twist)]:
        node.create_subscription(msg_type, '/' + topic,
                                 lambda msg, key=topic: data.update({key: msg}),
                                 qos_profile_sensor_data)
    buffer = Buffer()
    listener = TransformListener(buffer, node)
    publisher = node.create_publisher(Twist, '/cmd_vel', 1)

    def wait_until(predicate, timeout=20.0):
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=0.02)
            if predicate():
                return
        raise AssertionError('Timed out waiting for simulation feedback')

    def run_for(seconds, command=None):
        stamp = data['odom'].header.stamp
        start = stamp.sec + stamp.nanosec / 1e9
        end = time.monotonic() + max(15.0, 10 * seconds)
        while time.monotonic() < end:
            if command is not None:
                publisher.publish(command)
            rclpy.spin_once(node, timeout_sec=0.02)
            stamp = data['odom'].header.stamp
            if stamp.sec + stamp.nanosec / 1e9 - start >= seconds:
                return
        raise AssertionError('Simulation clock stopped advancing')

    try:
        wait_until(lambda: len(data) == 5 and publisher.get_subscription_count() > 0)
        scan = data['scan']
        assert len(scan.ranges) == 360
        assert abs(scan.angle_min) < 0.01 and scan.angle_max > 6.27
        assert scan.header.frame_id == 'base_scan'
        assert abs(scan.range_min - 0.12) < 0.001 and abs(scan.range_max - 3.5) < 0.001
        assert all(not math.isnan(value) for value in scan.ranges)
        assert set(data['joint_states'].name) == {'wheel_left_joint', 'wheel_right_joint'}
        assert data['odom'].header.frame_id == 'odom'
        assert data['odom'].child_frame_id == 'base_footprint'
        assert 8 < data['imu'].linear_acceleration.z < 11
        for frame in ('base_link', 'base_scan', 'imu_link', 'wheel_left_link', 'wheel_right_link'):
            wait_until(lambda f=frame: buffer.can_transform('odom', f, rclpy.time.Time()))
        transform = buffer.lookup_transform('base_footprint', 'base_scan', rclpy.time.Time())
        assert abs(transform.transform.translation.z - 0.182) < 0.001
        topics = [name for name, _ in node.get_topic_names_and_types()]
        assert not any(name.startswith(('/arm_controller/', '/gripper_controller/', '/camera/'))
                       for name in topics)
        print('PASS Burger: two wheel joints, base_scan (360 points, 0.12-3.5 m), IMU and full TF', flush=True)
        print('PASS no active iFLYTEK arm, gripper or camera topics', flush=True)

        if args.drive:
            before = data['odom'].pose.pose.position
            wheel_before = dict(zip(data['joint_states'].name, data['joint_states'].position))
            command = Twist()
            command.linear.x = 0.12
            run_for(2.0, command)
            run_for(0.7, Twist())
            after = data['odom'].pose.pose.position
            distance = math.hypot(after.x - before.x, after.y - before.y)
            assert 0.16 < distance < 0.32, distance
            wheels = dict(zip(data['joint_states'].name, data['joint_states'].position))
            assert abs(wheels['wheel_left_joint'] - wheel_before['wheel_left_joint']) > 3
            print(f'PASS straight motion and wheel feedback: {distance:.3f} m', flush=True)

            angle_before = yaw(data['odom'].pose.pose.orientation)
            physical_before = yaw(data['imu'].orientation)
            command = Twist()
            command.angular.z = -0.5
            run_for(1.5, command)
            run_for(0.7, Twist())
            delta = yaw(data['odom'].pose.pose.orientation) - angle_before
            delta = math.atan2(math.sin(delta), math.cos(delta))
            physical_delta = yaw(data['imu'].orientation) - physical_before
            physical_delta = math.atan2(math.sin(physical_delta), math.cos(physical_delta))
            assert -1.1 < delta < -0.45, delta
            assert -1.1 < physical_delta < -0.3, physical_delta
            print(f'PASS right turn: odom {delta:.3f} rad, IMU {physical_delta:.3f} rad', flush=True)

            command = Twist()
            command.linear.x = 0.1
            run_for(0.7, command)
            run_for(1.3)  # Stop publishing: the guard must emit zero commands.
            assert abs(data['odom'].twist.twist.linear.x) < 0.01
            assert data['burger/cmd_vel'].linear.x == 0.0
            print('PASS command timeout stops the robot', flush=True)

            command = Twist()
            command.linear.x = 10.0
            command.angular.z = -10.0
            run_for(0.1, command)
            guard = data['burger/cmd_vel']
            assert abs(guard.linear.x - 0.22) < 0.001, guard
            assert abs(guard.angular.z + 2.84) < 0.001, guard
            run_for(0.7, Twist())
            print('PASS Burger speed limits: 0.22 m/s and 2.84 rad/s', flush=True)
    finally:
        if rclpy.ok():
            publisher.publish(Twist())
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
