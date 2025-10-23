#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64

class ArmController(Node):
    def __init__(self):
        super().__init__('arm_controller')
        self.j1 = self.create_publisher(Float64, 'arm_controller/joint1', 10)
        self.j2 = self.create_publisher(Float64, 'arm_controller/joint2', 10)
        self.j3 = self.create_publisher(Float64, 'arm_controller/joint3', 10)
        self.j4 = self.create_publisher(Float64, 'arm_controller/joint4', 10)
        self.left = self.create_publisher(Float64, 'gripper/left', 10)
        self.right = self.create_publisher(Float64, 'gripper/right', 10)

        self.timer = self.create_timer(2.0, self.wave)
        self.pos = 0.0
        self.sign = 1.0

    def wave(self):
        self.pos += 0.3 * self.sign
        if abs(self.pos) > 1.0:
            self.sign *= -1.0
        msg = Float64()
        msg.data = self.pos
        self.j1.publish(msg)
        self.j2.publish(msg)
        self.j3.publish(msg)
        self.j4.publish(msg)
        msg2 = Float64()
        msg2.data = 0.02 if self.sign > 0 else -0.02
        self.left.publish(msg2)
        self.right.publish(msg2)


def main():
    rclpy.init()
    node = ArmController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
