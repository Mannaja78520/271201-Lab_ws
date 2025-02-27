import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess

def generate_launch_description():
    ld = LaunchDescription()

    monitor = Node(
        package='wrg_core',
        executable='monitoring_node.py',
        name='monitoring_node',
        output='screen',
    )

    ld.add_action(monitor)

    return ld
