#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy import qos
from ament_index_python.packages import get_package_share_directory
from launch import LaunchService
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription
from launch import LaunchDescription
from std_msgs.msg import String
import os
import subprocess

class MonitoringNode(Node):
    def __init__(self):
        super().__init__('monitoring_node')
        self.sub_robot_state = self.create_subscription(
            String,
            "robot_state",
            self.sub_robot_state_callback,
            qos_profile=qos.qos_profile_sensor_data,
        )
        self.launch_service = LaunchService()
        self.launch_descriptions = {
            'navigate.launch.py': self.create_launch_description('wrg_core', 'navigate.launch.py'),
            'robot.launch.py': self.create_launch_description('turtlebot3_bringup', 'robot.launch.py')
        }
        self.active_launches = {}

        # Kill existing nodes and start initial launch files
        self.start_launch_file('navigate.launch.py')
        # self.start_launch_file('robot.launch.py')
        
    def sub_robot_state_callback(self, msg: String):
        if msg.data == "None":
            self.restart_launch_file("navigate.launch.py")
        # if msg.data == "Init":
        #     self.restart_launch_file("robot.launch.py")

    def start_launch_file(self, launch_file_name):
        self.get_logger().info(f'Starting launch file: {launch_file_name}')
        launch_description = self.launch_descriptions[launch_file_name]
        self.launch_service.include_launch_description(launch_description)
        self.launch_service.run()
        self.active_launches[launch_file_name] = self.launch_service

    def restart_launch_file(self, launch_file_name):
        self.get_logger().info(f'Restarting launch file: {launch_file_name}')
        self.kill_process(launch_file_name)
        self.start_launch_file(launch_file_name)

    def kill_process(self, launch_file_name):
        # Map launch file names to process names
        process_names = {
            'navigate.launch.py': 'navigate.py',
            'robot.launch.py': 'robot_bringup.py'  # Adjust this if the executable name is different
        }
        process_name = process_names.get(launch_file_name)
        if process_name:
            try:
                self.get_logger().info(f'Killing process: {process_name}')
                subprocess.run(['killall', process_name], check=True)
            except subprocess.CalledProcessError as e:
                self.get_logger().warn(f'Failed to kill process {process_name}: {e}')

    def create_launch_description(self, package_name, launch_file_name):
        launch_file_dir = os.path.join(get_package_share_directory(package_name), 'launch')
        return LaunchDescription([
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([launch_file_dir, f'/{launch_file_name}']),
                launch_arguments={}.items()
            )
        ])

    def destroy_node(self):
        self.get_logger().info('Shutting down MonitoringNode...')
        for launch_file_name in self.active_launches.keys():
            self.kill_process(launch_file_name)
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    monitoring_node = MonitoringNode()
    rclpy.spin(monitoring_node)
    monitoring_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
