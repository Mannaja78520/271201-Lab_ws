#!/usr/bin/env python3
import rclpy
import time

from rclpy.node import Node
from geometry_msgs.msg import Twist

class RobotMainState(Node):
    def __init__(self):
        super().__init__("cmd")

        # Publisher for /cmd_vel
        self.move_robot_pub = self.create_publisher(
            Twist, "/cmd_vel", qos_profile=rclpy.qos.qos_profile_system_default
        )
        
        # Create a timer to periodically call the timer_callback function
        self.create_timer(1.0, self.timer_callback)

    def timer_callback(self):
        # Create a Twist message
        move_robot_msg = Twist()
        # move_robot_msg.linear.x = 0.5
        move_robot_msg.angular.z = -0.8

        # Publish the Twist message
        self.move_robot_pub.publish(move_robot_msg)
        time.sleep(3)
        self.get_logger().info(f"Published: linear.x = {move_robot_msg.linear.x}, angular.z = {move_robot_msg.angular.z}")
        
def main():
    rclpy.init()
    node = RobotMainState()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
