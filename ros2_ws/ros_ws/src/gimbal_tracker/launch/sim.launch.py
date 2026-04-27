import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    pkg_share = get_package_share_directory('gimbal_tracker')
    video_path = os.path.join(pkg_share, 'media', 'aruco_detector_test.mp4')

    return LaunchDescription([
        # The MP4 Video Simulator
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

        # ArUco Detector
        Node(
            package='gimbal_tracker',
            executable='aruco_detector', 
            name='aruco_detector',
            output='screen'
        ),

        Node(
            package='rqt_image_view',
            executable='rqt_image_view',
            name='rqt_image_view',
            output='screen',
        ),

    ])