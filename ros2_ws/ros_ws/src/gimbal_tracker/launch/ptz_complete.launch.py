import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

hostname = '10.73.0.152' # 10.73.0.152 for router (could change), 192.168.0.90 default without router
username = 'root'
password = 'pass'
encrypted_password = 'true' # 'true' for digest auth, 'false' for basic auth (not recommended)

# ros2 launch gimbal_tracker ptz_complete.launch.py 2>&1 | grep -v "sequence size exceeds remaining buffer"

def generate_launch_description():
    # Axis Camera Driver
    axis_dir = get_package_share_directory('axis_camera')
    axis_launch = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(
            os.path.join(axis_dir, 'launch', 'axis_camera.launch')
        ),
        # To change settings, modify the driver_node.launch.py in the axis_camera package.
        launch_arguments={'hostname': hostname,'username': username,'password': password,'use_encrypted_password': encrypted_password}.items()
    )

    # ArUco Detector
    aruco_node = Node(
        package='gimbal_tracker',
        executable='aruco_detector', 
        name='aruco_detector',
        output='screen',
        parameters=[{'resize_factor': 1.0, 'queue_size': 1, 'follow_time': 10.0}] # queue_size 0 means the aruco detector locks onto the first marker detected.
    )

    # PID Controller for PTZ movement
    pid_node = Node(
            package='gimbal_tracker',
            executable='pid_controller',
            name='pid_controller',
            output='screen'
        )

    # PTZ driver for Axis Camera: modify IP, username, and password as needed
    axis_node = Node(
            package='gimbal_tracker',
            executable='axis_driver',
            name='axis_driver',
            output='screen',
            parameters=[{'ip': hostname, 'user': username, 'password': password, 'use_encrypted_password': encrypted_password}]
        )

    # RQT Image View to visualize the camera feed
    rqt_image_view = Node(
            package='rqt_image_view',
            executable='rqt_image_view',
            name='rqt_image_view',
            output='screen',
        )

    return LaunchDescription([
        axis_launch,
        aruco_node,
        pid_node,
        axis_node,
        rqt_image_view
    ])