#!/usr/bin/env python3
"""Wheel forward distance + measured IMU yaw, without simulator ground truth."""
import math

import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu
from tf2_ros import TransformBroadcaster


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


class MazeOdometry(Node):
    def __init__(self):
        super().__init__('maze_odometry')
        self.x = self.y = self.yaw = self.rate = 0.
        self.imu_origin = self.imu_previous = self.previous_time = None
        self.previous_yaw = 0.
        self.previous_wheel = None
        self.imu_stamp = 0.
        self.publisher = self.create_publisher(Odometry, '/odom', 10)
        self.broadcaster = TransformBroadcaster(self)
        self.create_subscription(Imu, '/imu', self.on_imu, qos_profile_sensor_data)
        self.create_subscription(Odometry, '/wheel/odom', self.on_wheel, qos_profile_sensor_data)

    def on_imu(self, message):
        if message.orientation_covariance[0] < 0:
            return
        q = message.orientation
        angle = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
        if self.imu_origin is None:
            self.imu_origin = self.imu_previous = angle
        self.yaw += wrap(angle-self.imu_previous)
        self.imu_previous = angle
        self.rate = message.angular_velocity.z
        self.imu_stamp = message.header.stamp.sec + message.header.stamp.nanosec*1e-9

    def on_wheel(self, message):
        stamp = message.header.stamp.sec + message.header.stamp.nanosec*1e-9
        if self.imu_origin is None or abs(stamp-self.imu_stamp) > .5:
            return
        p = message.pose.pose.position
        q = message.pose.pose.orientation
        wheel_yaw = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
        velocity = 0.
        if self.previous_time is not None and self.previous_wheel is not None:
            dt = stamp-self.previous_time
            if 0 < dt < .3:
                yaw = (self.previous_yaw+self.yaw)/2
                # The plugin integrates encoder pose every physics step, but
                # its published twist is only the last instantaneous sample
                # (and linear.x loses the sign). Integrating that 30 Hz sample
                # aliases wheel vibration into large false forward travel.
                # Recover signed distance from the integrated pose instead.
                px, py, previous_angle = self.previous_wheel
                midpoint = previous_angle+wrap(wheel_yaw-previous_angle)/2
                distance = (p.x-px)*math.cos(midpoint)+(p.y-py)*math.sin(midpoint)
                velocity = distance/dt
                self.x += distance*math.cos(yaw)
                self.y += distance*math.sin(yaw)
        self.previous_wheel = p.x, p.y, wheel_yaw
        self.previous_time, self.previous_yaw = stamp, self.yaw
        result = Odometry()
        result.header.stamp = message.header.stamp
        result.header.frame_id = 'odom'; result.child_frame_id = 'base_footprint'
        result.pose.pose.position.x = self.x; result.pose.pose.position.y = self.y
        result.pose.pose.orientation.z = math.sin(self.yaw/2)
        result.pose.pose.orientation.w = math.cos(self.yaw/2)
        result.twist.twist.linear.x = velocity
        result.twist.twist.angular.z = self.rate
        for index, value in ((0,.0025),(7,.0025),(14,1e3),(21,1e3),(28,1e3),(35,.0004)):
            result.pose.covariance[index] = value
        self.publisher.publish(result)
        transform = TransformStamped()
        transform.header = result.header; transform.child_frame_id = result.child_frame_id
        transform.transform.translation.x = self.x; transform.transform.translation.y = self.y
        transform.transform.rotation = result.pose.pose.orientation
        self.broadcaster.sendTransform(transform)


def main():
    rclpy.init(); node = MazeOdometry()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()


if __name__ == '__main__': main()
