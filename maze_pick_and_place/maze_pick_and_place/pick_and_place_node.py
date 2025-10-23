#!/usr/bin/env python3
import math
import time
from typing import List

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState


class SimpleIK:
    def __init__(self):
        # Link lengths from xacro
        self.l1 = 0.2
        self.l2 = 0.2
        self.l3 = 0.2
        self.l4 = 0.25

    def solve_planar(self, x: float, z: float) -> List[float]:
        # 3-DOF planar IK for arm_joint_02..arm_joint_04 assuming arm_joint_01 yaw=0
        # Very naive IK for demo purposes
        # target is in arm_base_link frame with y=0
        dx = x
        dz = z
        L = [self.l1, self.l2, self.l3]
        # Use 3-link chain approximation (ignore l4/gripper)
        L1, L2, L3 = L
        # Collapse to 2-link by combining L2+L3 for simplicity
        L12 = L2 + L3
        r2 = dx * dx + dz * dz
        cos_th2 = (r2 - L1 * L1 - L12 * L12) / (2 * L1 * L12)
        cos_th2 = max(min(cos_th2, 1.0), -1.0)
        th2 = math.acos(cos_th2)
        k1 = L1 + L12 * math.cos(th2)
        k2 = L12 * math.sin(th2)
        th1 = math.atan2(dz, dx) - math.atan2(k2, k1)
        # distribute th2 across joint_02 and joint_03, keep joint_04 for pitch to keep end level
        j2 = th1
        j3 = th2 / 2.0
        j4 = th2 / 2.0
        return [j2, j3, j4]


class PickAndPlaceNode(Node):
    def __init__(self):
        super().__init__('pick_and_place_node')
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.j1_pub = self.create_publisher(Float64, '/arm_controller/joint1', 10)
        self.j2_pub = self.create_publisher(Float64, '/arm_controller/joint2', 10)
        self.j3_pub = self.create_publisher(Float64, '/arm_controller/joint3', 10)
        self.j4_pub = self.create_publisher(Float64, '/arm_controller/joint4', 10)
        self.grip_l_pub = self.create_publisher(Float64, '/gripper/left', 10)
        self.grip_r_pub = self.create_publisher(Float64, '/gripper/right', 10)

        self.ik = SimpleIK()

        # Simple startup timer to run sequence once
        self.create_timer(2.0, self.run_sequence)
        self.sequence_started = False

    def run_sequence(self):
        if self.sequence_started:
            return
        self.sequence_started = True
        self.get_logger().info('Starting pick-and-place sequence')

        # 1) Drive forward a bit into maze entry
        self.drive_forward(1.5, 0.2)

        # 2) Rotate arm base towards object area (keep 0 for now)
        self.publish_joint(self.j1_pub, 0.0)
        self.sleep(0.5)

        # 3) Open gripper
        self.set_gripper(opening=0.035)
        self.sleep(0.5)

        # 4) Move arm to a pre-grasp pose (x,z in arm base frame)
        j2, j3, j4 = self.ik.solve_planar(0.30, 0.15)
        self.publish_joint(self.j2_pub, j2)
        self.publish_joint(self.j3_pub, j3)
        self.publish_joint(self.j4_pub, j4)
        self.sleep(1.5)

        # 5) Lower slightly and close gripper (mock grasp)
        j2b, j3b, j4b = self.ik.solve_planar(0.30, 0.10)
        self.publish_joint(self.j2_pub, j2b)
        self.publish_joint(self.j3_pub, j3b)
        self.publish_joint(self.j4_pub, j4b)
        self.sleep(1.0)
        self.set_gripper(opening=0.0)
        self.sleep(0.5)

        # 6) Raise and place into bin area (assume bin near +x,+y; we just position arm upwards)
        j2c, j3c, j4c = self.ik.solve_planar(0.25, 0.25)
        self.publish_joint(self.j2_pub, j2c)
        self.publish_joint(self.j3_pub, j3c)
        self.publish_joint(self.j4_pub, j4c)
        self.sleep(1.5)
        self.set_gripper(opening=0.03)
        self.sleep(0.5)

        self.get_logger().info('Sequence complete')

    def publish_joint(self, pub, value: float):
        msg = Float64()
        msg.data = float(value)
        pub.publish(msg)

    def set_gripper(self, opening: float):
        # opening is symmetric extent for both fingers
        l = Float64(); l.data = opening
        r = Float64(); r.data = -opening
        self.grip_l_pub.publish(l)
        self.grip_r_pub.publish(r)

    def drive_forward(self, duration_sec: float, linear_x: float):
        msg = Twist()
        msg.linear.x = linear_x
        start = time.time()
        while time.time() - start < duration_sec and rclpy.ok():
            self.cmd_vel_pub.publish(msg)
            rclpy.spin_once(self, timeout_sec=0.1)
        msg.linear.x = 0.0
        self.cmd_vel_pub.publish(msg)

    def sleep(self, sec: float):
        end = time.time() + sec
        while time.time() < end and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.05)


def main():
    rclpy.init()
    node = PickAndPlaceNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

