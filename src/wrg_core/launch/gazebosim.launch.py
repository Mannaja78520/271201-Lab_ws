import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()

    # Declare the use_sim_time argument
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    # Define the path to the launch file directories
    wrg_gazebo_launch_file_dir = os.path.join(get_package_share_directory('wrg_turtlebot3_gazebo'), 'launch')
    wrg_core_launch_file_dir = os.path.join(get_package_share_directory('wrg_core'), 'launch')

    # Include navigate.launch.py and pass the use_sim_time argument
    navigate_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(wrg_core_launch_file_dir, 'navigate.launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time
        }.items()
    )
    
    # Include wrg_world.launch.py and pass the use_sim_time argument
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(wrg_gazebo_launch_file_dir, 'wrg_world.launch.py')
        )
    )

    # Add actions to the launch description
    # ld.add_action(DeclareLaunchArgument('use_sim_time', default_value='true', description='Use simulation (Gazebo) clock if true'))
    ld.add_action(navigate_launch)
    ld.add_action(gazebo_launch)

    return ld
