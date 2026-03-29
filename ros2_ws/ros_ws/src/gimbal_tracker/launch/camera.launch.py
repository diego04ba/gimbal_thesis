import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    return LaunchDescription([
        # NODE 1: Camera Node
        Node(
            package='gimbal_tracker',
            executable='camera_node', 
            name='camera_source',
            output='screen',
            parameters=[{'use_sim_time': False}]
        ),

        # NODE 2: ArUco Detector
        Node(
            package='gimbal_tracker',
            executable='aruco_detector', 
            name='aruco_processor',
            output='screen',
        )
    ])