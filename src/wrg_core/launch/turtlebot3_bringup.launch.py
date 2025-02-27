import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()
    turtlebot3_launch_file_dir = os.path.join(get_package_share_directory('turtlebot3_bringup'), 'launch')
    turtlebot3_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(turtlebot3_launch_file_dir, 'robot.launch.py')
        )
    )
    
    ld.add_action(turtlebot3_launch)
    
    return ld
    
