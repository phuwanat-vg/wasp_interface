import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    

    return LaunchDescription([
        
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            parameters=[{'use_sim_time': False}],
            namespace = 'robot1',
            remappings = [('/tf','tf'),('/tf_static','tf_static'),('/initialpose','initialpose'),('/waypoints','waypoints')],
            output='screen'),
    ])
