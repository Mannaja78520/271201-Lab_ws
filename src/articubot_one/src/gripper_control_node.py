#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64

class GripperControlNode(Node):
    def __init__(self):
        super().__init__('gripper_control_node')
        # Create publishers for controlling the gripper joints
        self.gripper_vertical_pub = self.create_publisher(Float64, '/gripper_vertical_joint/command', 10)
        self.finger_left_pub = self.create_publisher(Float64, '/gripper_finger_left_joint/command', 10)
        self.finger_right_pub = self.create_publisher(Float64, '/gripper_finger_right_joint/command', 10)

    def move_gripper_vertical(self, position):
        msg = Float64()
        msg.data = position
        self.gripper_vertical_pub.publish(msg)
        self.get_logger().info(f'Moving vertical gripper to {position} meters')

    def move_finger_left(self, position):
        msg = Float64()
        msg.data = position
        self.finger_left_pub.publish(msg)
        self.get_logger().info(f'Moving left finger to {position} meters')

    def move_finger_right(self, position):
        msg = Float64()
        msg.data = position
        self.finger_right_pub.publish(msg)
        self.get_logger().info(f'Moving right finger to {position} meters')

def main(args=None):
    rclpy.init(args=args)
    gripper_control_node = GripperControlNode()

    # Example usage: move the gripper
    gripper_control_node.move_gripper_vertical(0.05)  # Move gripper vertical joint to 5 cm
    gripper_control_node.move_finger_left(0.02)       # Move left finger to 2 cm
    gripper_control_node.move_finger_right(0.02)      # Move right finger to 2 cm

    rclpy.spin(gripper_control_node)

if __name__ == '__main__':
    main()
