#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from geometry_msgs.msg import Twist, PoseStamped
from sensor_msgs.msg import LaserScan, Image
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from nav2_simple_commander.robot_navigator import NavigationResult
from nav2_msgs.srv import ClearEntireCostmap
import math
import random

class ExploreAndCollect(Node):
    def __init__(self):
        super().__init__('explore_and_collect')
        self.navigator = BasicNavigator()
        self.navigator.waitUntilNav2Active()
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.scan_sub = self.create_subscription(LaserScan, 'scan', self.on_scan, 10)
        self.image_sub = self.create_subscription(Image, 'camera/image_raw', self.on_image, 10)
        self.timer = self.create_timer(1.0, self.wander_goal)
        self.last_scan = None
        self.get_logger().info('ExploreAndCollect node started')

    def on_scan(self, msg: LaserScan):
        self.last_scan = msg

    def on_image(self, msg: Image):
        # Placeholder: in real use, detect object blobs/colors
        pass

    def wander_goal(self):
        if not self.navigator.isTaskComplete():
            return
        # Send a random nearby goal to encourage exploration
        goal = PoseStamped()
        goal.header.frame_id = 'map'
        goal.header.stamp = self.navigator.get_clock().now().to_msg()
        goal.pose.position.x = random.uniform(-3.0, 3.0)
        goal.pose.position.y = random.uniform(-3.0, 3.0)
        yaw = random.uniform(-math.pi, math.pi)
        goal.pose.orientation.z = math.sin(yaw/2.0)
        goal.pose.orientation.w = math.cos(yaw/2.0)
        self.navigator.goToPose(goal)


def main():
    rclpy.init()
    node = ExploreAndCollect()
    try:
        rclpy.spin(node)
    finally:
        node.navigator.cancelTask()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
