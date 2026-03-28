import sys
import rclpy
from rclpy.node import Node

# Importing messages to use frames and give position errors
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point

# Importing OpenCV and ArUco libraries for marker detection
import cv2
import cv2.aruco as aruco
from cv_bridge import CvBridge

class ArucoNode(Node):
    def __init__(self):
        super().__init__('aruco_node')
        # ArUco Configuration
        self.aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
        self.parameters = aruco.DetectorParameters()
        # Creating Bridge 
        self.br = CvBridge()
        # Subscribing to the /image topic to get frames
        self.subscription = self.create_subscription(
            Image,
            '/image',
            self.image_callback,
            10)
        # Publisher to publish the position error
        self.publisher_ = self.create_publisher(Point, '/position', 10)
        
        self.get_logger().info('Aruco Node initialized and waiting for images...')

    # Creating the function to process the frames everytime the node gets one
    def image_callback(self, msg):
            # Convert ROS Image message to OpenCV format
            cv_image = self.br.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            # Detect ArUco markers in the frame
            corners, ids, _ = aruco.detectMarkers(cv_image, self.aruco_dict, parameters=self.parameters)
            
            if ids is not None:
                for corner in corners:
                    # Calculate the center of the detected marker
                    center_x = int(corner[0][:, 0].mean())
                    center_y = int(corner[0][:, 1].mean())
                    
                    # Create a Point message to publish the position error
                    position_error = Point()
                    position_error.x = center_x - (cv_image.shape[1] / 2)  # Error in x-axis
                    position_error.y = center_y - (cv_image.shape[0] / 2)  # Error in y-axis
                    position_error.z = 0.0  # Assuming a 2D plane, could use it later if needed
                    
                    # Publish the position error
                    self.publisher_.publish(position_error)
                    
                    # Draw a circle at the center of the detected marker for visualization
                    cv2.circle(cv_image, (center_x, center_y), 5, (0, 255, 0), -1)
            
            # Display the processed image with detected markers (only if needed)
            cv2.imshow('Aruco Detection', cv_image)
            cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)
    aruco_node = ArucoNode()
    try:
        rclpy.spin(aruco_node)
    except KeyboardInterrupt:
        pass
    finally:
        aruco_node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()