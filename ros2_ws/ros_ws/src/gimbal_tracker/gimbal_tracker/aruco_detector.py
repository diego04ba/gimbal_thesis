import sys
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

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

        # Specific ID to follow (for example, ID 1)
        self.target_id = 1 # change this to the ID you want to track

        # Creating Bridge 
        self.br = CvBridge()

        # Subscribing to the /image topic to get frames
        self.subscription = self.create_subscription(
            Image,
            '/flir_camera/image_raw',
            self.image_callback,
            10)
        self.subscription  # prevent unused variable warning
        
        # Publisher to publish the position error
        self.publisher_ = self.create_publisher(Point, '/position', 10)

        self.last_marker_time = self.get_clock().now()
        self.marker_timeout = 5.0
        self.watchdog_timer = self.create_timer(5, self.watchdog_callback)

        self.target_fps = 20.0
        self.min_time_between_frames = 1.0 / self.target_fps
        self.last_processed_time = self.get_clock().now()

        self.resize_factor = 0.5 # To reduce the resolution for faster processing, needs testing
        
        self.get_logger().info(f'Aruco Node initialized. Tracking target ID: {self.target_id}')

    # Creating the function to process the frames everytime the node gets one
    def image_callback(self, msg):
        try:
            # Elaborating only 20 frames per second to reduce CPU load
            current_time = self.get_clock().now()
            time_since_last_frame = (current_time - self.last_processed_time).nanoseconds / 1e9
            if time_since_last_frame < self.min_time_between_frames:
                return  # Skip processing to maintain target FPS
            self.last_processed_time = current_time

            # Conversion of the ROS Image message to OpenCV format
            cv_image = self.br.imgmsg_to_cv2(msg, desired_encoding='mono8')

            # Reducing the resolution for faster processing
            if self.resize_factor < 1.0:
                small_image = cv2.resize(cv_image, (0, 0), fx=self.resize_factor, fy=self.resize_factor)
            else:
                small_image = cv_image
            
            # Dinamically get the dimensions of the frame to calculate the center (Setpoint)
            height, width = small_image.shape[:2]
            
            # Exact center of the frame (Setpoint for the gimbal)
            frame_center_x = width / 2.0
            frame_center_y = height / 2.0

            # ArUco Marker Detection
            corners, ids, _ = aruco.detectMarkers(small_image, self.aruco_dict, parameters=self.parameters)

            if ids is not None:
                # Flatten the ID array for easier searching
                ids_flat = ids.flatten()

                # Check if the target ID is among the detected markers
                if self.target_id in ids_flat:
                    self.last_marker_time = self.get_clock().now() # Update last seen time
                    # Find the index of the target ID to get its corresponding corners
                    idx = list(ids_flat).index(self.target_id)
                    marker_corners = corners[idx][0]

                    # Calculation of the marker center (in pixel coordinates)
                    marker_center_x = float(marker_corners[:, 0].mean())
                    marker_center_y = float(marker_corners[:, 1].mean())

                    # Calculation of the error (Distance between marker center and frame center)
                    # If the error is 0, the marker is perfectly centered
                    error_x = marker_center_x - frame_center_x
                    error_y = marker_center_y - frame_center_y

                    # Resize the error values to match the original image dimensions
                    real_error_x = error_x * (1 / self.resize_factor)
                    real_error_y = error_y * (1 / self.resize_factor)

                    # Publishing the Point message
                    error_msg = Point()
                    error_msg.x = real_error_x
                    error_msg.y = real_error_y
                    error_msg.z = 0.0 
                    
                    self.publisher_.publish(error_msg)

                    # Logging the error for debugging purposes
                    self.get_logger().info(f'TARGET ID {self.target_id} - Error X: {real_error_x:.2f}, Y: {real_error_y:.2f}')
                else:
                    self.get_logger().info(f'Marker ID {self.target_id} not found.', once=False, throttle_duration_sec=1.0)

        except Exception as e:
            self.get_logger().error(f'Error in callback: {str(e)}')
    
    def watchdog_callback(self):
        current_time = self.get_clock().now()
        time_since_last_marker = (current_time - self.last_marker_time).nanoseconds / 1e9

        if time_since_last_marker > self.marker_timeout:
            self.get_logger().warn(f'No marker detected for {time_since_last_marker:.2f} seconds. Check camera feed or marker visibility.')

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

if __name__ == '__main__':
    main()