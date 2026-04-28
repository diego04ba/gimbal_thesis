import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge
import os

class VideoSimNode(Node):
    def __init__(self):
        super().__init__('video_simulator')
        self.declare_parameter('mp4_path', 'aruco_detector_test.mp4')
        self.video_path = self.get_parameter('mp4_path').get_parameter_value().string_value
        
        self.bridge = CvBridge()
        
        self.cap = cv2.VideoCapture(self.video_path)
        
        # Check if the video file was opened successfully.
        if not self.cap.isOpened():
            self.get_logger().error(f'CRITICAL: Could not open video file at: {self.video_path}')
        else:
            self.get_logger().info(f'SUCCESS: Simulation started with file: {self.video_path}')

        self.publisher_ = self.create_publisher(Image, '/flir_camera/image_raw', 10)
        
        self.timer_period = 0.1  
        self.timer = self.create_timer(self.timer_period, self.timer_callback)

    def timer_callback(self):
        
        if not self.cap.isOpened():
            return

        # Capture a single frame from the video source
        ret, frame = self.cap.read()
        
        # If the video reaches the end (ret is False), reset to frame 0.
        if not ret:
            self.get_logger().info('End of video reached. Looping back to the beginning.')
            self.cap.set(cv2.CAP_PROP_POS_FRAMES, 0)
            ret, frame = self.cap.read()

        # If a frame is successfully read, convert it to a ROS Image message and publish.
        if ret:
            # Convert OpenCV BGR format to ROS sensor_msgs/Image
            frame = cv2.resize(frame, (640, 480))
            msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
            
            # Add timestamp and frame_id for synchronization and tracking purposes
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = 'sim_camera_frame'
            
            self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = VideoSimNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Node stopped by user.')
    finally:
        node.cap.release()
        node.destroy_node()
        rclpy.shutdown()