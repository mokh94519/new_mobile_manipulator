#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64, Bool

class PickPlace(Node):
    def __init__(self) -> None:
        super().__init__('pick_place')
        self.det_sub = self.create_subscription(Bool, 'object_detected', self.on_detected, 10)
        self.j1 = self.create_publisher(Float64, 'arm_controller/joint1', 10)
        self.j2 = self.create_publisher(Float64, 'arm_controller/joint2', 10)
        self.j3 = self.create_publisher(Float64, 'arm_controller/joint3', 10)
        self.j4 = self.create_publisher(Float64, 'arm_controller/joint4', 10)
        self.g_l = self.create_publisher(Float64, 'gripper/left', 10)
        self.g_r = self.create_publisher(Float64, 'gripper/right', 10)
        self.step = 0

    def on_detected(self, msg: Bool) -> None:
        if not msg.data:
            return
        # Very naive scripted pick and place cycle
        self.get_logger().info('Object detected, attempting pick-and-place')
        def set_pub(pub, val):
            m = Float64(); m.data = float(val); pub.publish(m)
        # Open gripper
        set_pub(self.g_l, 0.04)
        set_pub(self.g_r, -0.04)
        # Move arm to pre-grasp
        set_pub(self.j1, 0.0)
        set_pub(self.j2, -0.8)
        set_pub(self.j3, 1.2)
        set_pub(self.j4, -0.4)
        # Close gripper
        set_pub(self.g_l, 0.0)
        set_pub(self.g_r, 0.0)
        # Move arm to drop pose
        set_pub(self.j2, 0.2)
        set_pub(self.j3, -0.5)
        # Open gripper
        set_pub(self.g_l, 0.04)
        set_pub(self.g_r, -0.04)


def main() -> None:
    rclpy.init()
    node = PickPlace()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
