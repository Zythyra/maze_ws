#!/usr/bin/env python3
"""One-shot terminal start for the C++ wall follower; never publishes velocity."""
import json

import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import SetBool

from terminal_start import TerminalStartGate


class WallStart(Node):
    def __init__(self):
        super().__init__('wall_start')
        device=self.declare_parameter('terminal_device','/dev/tty').value
        service=self.declare_parameter('service_name','/wall_follow/set_enabled').value
        self.gate=TerminalStartGate(device,self.get_logger().info,service_name=service)
        self.client=self.create_client(SetBool,service)
        self.requested=False;self.done=False;self.future=None
        self.create_subscription(String,'/wall_follow/status',self.on_status,1)
        self.create_timer(.10,self.tick,clock=Clock(clock_type=ClockType.STEADY_TIME))

    def on_status(self,message):
        try:status=json.loads(message.data)
        except (ValueError,TypeError):return
        if status.get('enabled') or status.get('finished'):
            self.gate.close();self.done=True

    def tick(self):
        if self.future is not None and self.future.done():
            try:
                result=self.future.result()
                if result.success:self.get_logger().info('收到 start，左墙巡线开始。')
                else:self.get_logger().warning(result.message)
            except Exception as error:self.get_logger().error(f'启动服务调用失败：{error}')
            self.future=None;self.done=True
        if self.done:return
        if self.gate.poll():self.requested=True
        if self.requested and self.future is None and self.client.service_is_ready():
            self.future=self.client.call_async(SetBool.Request(data=True))

    def destroy_node(self):
        self.gate.close()
        return super().destroy_node()


def main():
    rclpy.init();node=WallStart()
    try:rclpy.spin(node)
    except (KeyboardInterrupt,rclpy.executors.ExternalShutdownException):pass
    finally:
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()


if __name__=='__main__':main()
