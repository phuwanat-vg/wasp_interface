import os
from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.actions import GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.substitutions import TextSubstitution
from launch_ros.actions import Node
from launch_ros.actions import PushRosNamespace
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():

    
    return LaunchDescription([
    	
    	Node(
            package='nav2_map_server',
            executable='map_saver_cli',
            name='map_save_node',
            output='screen',
            namespace='robot1',
            arguments=['-f', 'map'],
            remappings=[
        ('/tf','tf'),('/tf_static','tf_static')
        ]),
        
       
    ])
