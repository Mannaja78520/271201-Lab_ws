#!/usr/bin/env python3


import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64

class GripperPublisher(Node):
    def __init__(self):
        super().__init__('gripper_publisher')
        self.publisher_ = self.create_publisher(Float64, '/gripper_control', 10)
        self.timer = self.create_timer(1.0, self.timer_callback)
        self.counter = 0

    def timer_callback(self):
        msg = Float64()
        msg.data = 0.1  # The desired gripper position (example)
        self.publisher_.publish(msg)
        self.get_logger().info(f"Publishing gripper position: {msg.data}")

def main(args=None):
    rclpy.init(args=args)
    node = GripperPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
