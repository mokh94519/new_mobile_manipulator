#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image

class ObjectDetector(Node):
    def __init__(self):
        super().__init__('object_detector')
        self.sub = self.create_subscription(Image, 'camera/image_raw', self.on_img, 10)
        self.get_logger().info('ObjectDetector node started (placeholder)')

    def on_img(self, msg: Image):
        # Placeholder for detection
        pass


def main():
    rclpy.init()
    node = ObjectDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
