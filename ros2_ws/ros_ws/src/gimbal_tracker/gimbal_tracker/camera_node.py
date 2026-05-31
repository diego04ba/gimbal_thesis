import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from sensor_msgs.msg import CompressedImage
import cv2
from cv_bridge import CvBridge

class CameraNode(Node):
    def __init__(self):
        super().__init__('camera_node')
        
        # --- SOURCE CONFIGURATION ---
        # Webcam Laptop: 0
        # Smartphone IP Webcam: "http://192.168.1.202:8080/video"
        # Smartphone localhost with IP Webcam: "http://host.docker.internal:8080/video"
        self.declare_parameter('source', "http://192.168.1.202:8080/video")
        self.source = self.get_parameter('source').value

        self.cap = cv2.VideoCapture(self.source)
        self.br = CvBridge()
        
        # Publisher: sending frames to /image topic
        self.publisher_ = self.create_publisher(CompressedImage, '/image_raw/compressed', 10)
        
        # Timer: 0.1s period for 10Hz sampling
        self.timer = self.create_timer(0.1, self.timer_callback)
        
        self.get_logger().info(f'Camera Node started on source: {self.source}')

    def timer_callback(self):
        ret, frame = self.cap.read()
        
        if ret:
            # Convert the OpenCV frame to ROS CompressedImage message and publish it
            msg = self.br.cv2_to_compressed_imgmsg(frame)
            self.publisher_.publish(msg)
        else:
            self.get_logger().warn('Unable to retrieve frame from source.')

    def __del__(self):
        # Ensure resources are released
        if hasattr(self, 'cap') and self.cap.isOpened():
            self.cap.release()

def main(args=None):
    rclpy.init(args=args)
    node = CameraNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Explicitly release the camera before shutting down
        if hasattr(node, 'cap') and node.cap.isOpened():
            node.cap.release()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()