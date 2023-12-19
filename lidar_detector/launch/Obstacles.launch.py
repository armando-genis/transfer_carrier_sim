from launch import LaunchDescription
from launch_ros.actions import Node
import launch_ros
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    
    return LaunchDescription([
        Node(
            package='lidar_detector',
            executable='obstacle_avoidance_node',
            name='obstacle_avoidance_node',
            output='screen'
        )
    ])