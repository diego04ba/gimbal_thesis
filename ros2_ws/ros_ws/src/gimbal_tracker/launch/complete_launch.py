# FLIR Camera Node, ArUco Determination Node, Gimbal Control Node, Gimbal Hardware Node for feedback and control.

import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Node 1: FLIR Camera Driver
    spinnaker_dir = get_package_share_directory('spinnaker_camera_driver')
    flir_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(spinnaker_dir, 'launch', 'driver_node.launch.py')
        )
    )

    # Node 2: ArUco Detector
    aruco_node = Node(
        package='gimbal_tracker',
        executable='aruco_detector', 
        name='aruco_processor',
        output='screen'
    )

    # Node 3: PID Controller for Gimbal
    pid_node = Node(
        package='gimbal_tracker',
        executable='pid_controller',
        name='pid_controller',
        output='screen'
    )

    # Node 4: Gimbal Hardware Interface
    gimbal_driver = Node(
        package='gimbal_tracker',
        executable='gimbal_driver',
        name='gimbal_driver',
        output='screen'
    )

    return LaunchDescription([
        flir_launch,
        aruco_node,
        pid_node,
        gimbal_driver
    ])