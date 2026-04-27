# FLIR Camera Node, ArUco Determination Node, Gimbal Control Node, Gimbal Hardware Node for feedback and control.

import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # FLIR Camera Driver
    spinnaker_dir = get_package_share_directory('spinnaker_camera_driver')
    flir_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(spinnaker_dir, 'launch', 'driver_node.launch.py')
        ),
        # To change settings, modify the driver_node.launch.py in the spinnaker_camera_driver package.
        launch_arguments={
            'serial': "'22115751'" 
        }.items()
    )

    # ArUco Detector
    aruco_node = Node(
        package='gimbal_tracker',
        executable='aruco_detector', 
        name='aruco_detector',
        output='screen'
    )

    # PID Controller for Gimbal
    pid_node = Node(
        package='gimbal_tracker',
        executable='pid_controller',
        name='pid_controller',
        output='screen'
    )

    # Gimbal Hardware Interface: modify serial port or baudrate as needed
    gimbal_driver = Node(
        package='gimbal_tracker',
        executable='gimbal_driver',
        name='gimbal_driver',
        output='screen',
        parameters=[{'serial_port': '/dev/ttyUSB0', 'baudrate': 115200}]
    )

    # RQT Image View to visualize the camera feed
    rqt_image_view = Node(
        package='rqt_image_view',
        executable='rqt_image_view',
        name='rqt_image_view',
        output='screen',
    )

    return LaunchDescription([
        flir_launch,
        aruco_node,
        pid_node,
        gimbal_driver,
        rqt_image_view
    ])