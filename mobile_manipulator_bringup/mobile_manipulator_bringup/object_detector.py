#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Bool

class SimpleColorDetector(Node):
    def __init__(self) -> None:
        super().__init__('simple_color_detector')
        self.sub = self.create_subscription(Image, 'camera/image_raw', self.on_image, 10)
        self.pub = self.create_publisher(Bool, 'object_detected', 10)

    def on_image(self, msg: Image) -> None:
        # Placeholder: publish True periodically to simulate detection
        out = Bool()
        out.data = True
        self.pub.publish(out)


def main() -> None:
    rclpy.init()
    node = SimpleColorDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
