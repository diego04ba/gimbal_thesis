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
        # Smartphone using IP Webcam: "http://192.168.1.XX:8080/video"
        self.source = 0 
        
        self.cap = cv2.VideoCapture(self.source)
        self.br = CvBridge()
        
        # Publisher: invia messaggi di tipo Image sul topic /image
        self.publisher_ = self.create_publisher(Image, '/image', 10)
        
        # Timer
        self.timer = self.create_timer(0.1, self.timer_callback)
        
        self.get_logger().info(f'Camera Node avviato sulla sorgente: {self.source}')

    def timer_callback(self):
        ret, frame = self.cap.read()
        
        if ret:
            # Convertiamo il frame OpenCV in messaggio ROS Image
            msg = self.br.cv2_to_imgmsg(frame, encoding="bgr8")
            self.publisher_.publish(msg)
        else:
            self.get_logger().warn('Impossibile recuperare il frame dalla sorgente.')

    def __del__(self):
        self.cap.release()

def main(args=None):
    rclpy.init(args=args)
    node = CameraNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()