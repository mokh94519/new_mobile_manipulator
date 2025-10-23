#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class ExploreAndCollect(Node):
    def __init__(self) -> None:
        super().__init__('explore_and_collect')
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.timer = self.create_timer(0.1, self._on_timer)
        self.t = 0.0
        self.get_logger().info('ExploreAndCollect node started')

    def _on_timer(self) -> None:
        self.t += 0.1
        msg = Twist()
        msg.linear.x = 0.15
        msg.angular.z = 0.5 * math.sin(self.t / 3.0)
        self.cmd_pub.publish(msg)


def main() -> None:
    rclpy.init()
    node = ExploreAndCollect()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
