import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    pkg_share = get_package_share_directory('gimbal_tracker')
    video_path = os.path.join(pkg_share, 'media', 'aruco_detector_test.mp4')

    return LaunchDescription([
        # NODE 1: The MP4 Video Simulator
        # This node reads the video file and publishes frames to /image_raw
        Node(
            package='gimbal_tracker',
            executable='video_sim', 
            name='video_simulator',
            output='screen',
            parameters=[{
                'mp4_path': video_path
            }]
        ),

        # NODE 2: ArUco Detector
        # This node subscribes to /image_raw and processes the frames
        Node(
            package='gimbal_tracker',
            executable='aruco_detector', 
            name='aruco_processor',
            output='screen'
        ),

        # NODE 3: PID Controller
        # This node subscribes to the ArUco detection results and computes control commands
        Node(
            package='gimbal_tracker',
            executable='pid_controller', 
            name='pid_controller',
            output='screen'
        )
    ])