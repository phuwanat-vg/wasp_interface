from http.server import executable
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():

    lidar_launch_path = PathJoinSubstitution(
        [FindPackageShare('ydlidar_ros2_driver'),'launch','ydlidar_launch.py']
    )

    return LaunchDescription([
        Node(
            package="wasp_interface",
            executable="wasp_transmission",
            name="wasp_transmission_node",
            output='screen',
            remappings = [('/tf','tf'),('/tf_static','tf_static')],
            namespace='robot1',
        ),

      
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(lidar_launch_path)

        ),
       
    ])
