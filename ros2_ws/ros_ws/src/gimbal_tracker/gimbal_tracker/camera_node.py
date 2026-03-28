import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge

class CameraNode(Node):
    def __init__(self):
        super().__init__('camera_node')
        
        # --- SOURCE CONFIGURATION ---
        # Webcam Laptop: 0
        # Smartphone IP Webcam: "http://192.168.1.202:8080/video"
        # Smartphone localhost with IP Webcam: "http://host.docker.internal:8080/video"
        self.source = "http://192.168.1.202:8080/video"  # Change this to your desired source
        
        self.cap = cv2.VideoCapture(self.source)
        self.br = CvBridge()
        
        # Publisher: sending frames to /image topic
        self.publisher_ = self.create_publisher(Image, '/image', 10)
        
        # Timer: 0.1s period for 10Hz sampling
        self.timer = self.create_timer(0.1, self.timer_callback)
        
        self.get_logger().info(f'Camera Node started on source: {self.source}')

    def timer_callback(self):
        ret, frame = self.cap.read()
        
        if ret:
            # --- DEBUG SECTION (Comment this out later) ---
            # This shows a local window to confirm the connection
            # cv2.imshow("Camera Debugger", frame)
            # cv2.waitKey(1) 
            # ----------------------------------------------

            # Convert the OpenCV frame to ROS Image message and publish it
            msg = self.br.cv2_to_imgmsg(frame, encoding="bgr8")
            self.publisher_.publish(msg)
        else:
            self.get_logger().warn('Unable to retrieve frame from source.')

    def __del__(self):
        # Ensure resources are released
        if self.cap.isOpened():
            self.cap.release()
        cv2.destroyAllWindows()

def main(args=None):
    rclpy.init(args=args)
    node = CameraNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Explicitly release the camera before shutting down
        node.cap.release()
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()