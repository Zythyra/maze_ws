#!/usr/bin/env python3
"""Read-only, 20 Hz PP tracking measurements from a visible running simulation."""
import argparse
import json
import math
from pathlib import Path
import time

import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry, Path as RosPath
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import String


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--duration', type=float, default=60.)
    parser.add_argument('--output', required=True)
    parser.add_argument('--stop-after-home',type=float,default=0.,help='Stop this many seconds after HOME_REACHED; 0 keeps fixed duration')
    args = parser.parse_args()
    rclpy.init()
    node = rclpy.create_node('tracking_monitor')
    latest = {}

    def odometry(message):
        p, q = message.pose.pose.position, message.pose.pose.orientation
        state=latest.get('status',{})
        angle=state.get('map_odom_yaw',0.);offset=state.get('map_odom_xy',[0.,0.])
        c,s=math.cos(angle),math.sin(angle)
        latest.update(odom_position=[p.x, p.y],
                      position=[offset[0]+c*p.x-s*p.y,offset[1]+s*p.x+c*p.y],
                      yaw=angle+math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z)),
                      yaw_rate=message.twist.twist.angular.z,
                      measured_linear=message.twist.twist.linear.x,
                      sim_time=message.header.stamp.sec+message.header.stamp.nanosec*1e-9)

    def command(message):
        latest.update(linear=message.linear.x, angular=message.angular.z)

    def status(message):
        latest['status'] = json.loads(message.data)
        # The controller reports pose in the same frame as its reference path.
        if 'position' in latest['status']:
            latest['position']=latest['status']['position']
            latest['yaw']=latest['status']['yaw']

    def path(message):
        latest['path'] = [[p.pose.position.x, p.pose.position.y] for p in message.poses]

    node.create_subscription(Odometry, '/odom', odometry, qos_profile_sensor_data)
    node.create_subscription(Twist, '/cmd_vel', command, 10)
    node.create_subscription(String, '/polyline/status', status, 10)
    node.create_subscription(RosPath, '/polyline/active_path', path, 10)
    start = time.monotonic()
    last = start
    home_at=None
    with Path(args.output).open('w') as output:
        while time.monotonic()-start < args.duration:
            rclpy.spin_once(node, timeout_sec=.01)
            now = time.monotonic()
            if now-last < .05 or not all(k in latest for k in ('position', 'path', 'status')):
                continue
            last = now
            report = dict(latest, elapsed=now-start)
            if args.stop_after_home>0 and report['status'].get('state')=='HOME_REACHED':
                if home_at is None:home_at=now
                elif now-home_at>=args.stop_after_home:break
            point = np.array(report['position'])
            errors = []
            for a, b in zip(report['path'][:-1], report['path'][1:]):
                a, b = np.array(a), np.array(b)
                d = b-a
                projection = a+np.clip((point-a)@d/max(d@d, 1e-9), 0., 1.)*d
                errors.append(float(np.linalg.norm(point-projection)))
            report['cross_track'] = min(errors, default=0.)
            output.write(json.dumps(report)+'\n')
            output.flush()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
