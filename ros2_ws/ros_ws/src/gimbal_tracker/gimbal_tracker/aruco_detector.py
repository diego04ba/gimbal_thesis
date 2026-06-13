import sys
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

# Importing messages to use frames and give position errors
from sensor_msgs.msg import Image
from sensor_msgs.msg import CompressedImage
from geometry_msgs.msg import Point

# Importing OpenCV and ArUco libraries for marker detection
import cv2
import cv2.aruco as aruco
from cv_bridge import CvBridge

# Importing queue for managing the marker blacklist
from collections import deque

class ArucoNode(Node):
    def __init__(self):
        super().__init__('aruco_node')

        # ArUco Configuration
        self.aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
        self.parameters = aruco.DetectorParameters()

        self.target_id = -1 # Avoiding auto-detection of the marker 
        self.declare_parameter('id_limit', 4)
        self.id_limit = self.get_parameter('id_limit').get_parameter_value().integer_value
        self.declare_parameter('queue_size', 1)
        self.queue_size = self.get_parameter('queue_size').get_parameter_value().integer_value
        self.marker_queue = deque(maxlen=self.queue_size)

        self.declare_parameter('follow_time', 10.0)
        self.follow_time = self.get_parameter('follow_time').get_parameter_value().double_value
        self.follow_marker_time = self.get_clock().now()

        # Creating Bridge 
        self.br = CvBridge()

        # Subscribing to the /image_raw/compressed topic to get frames from the Device camera
        self.subscription = self.create_subscription(CompressedImage,'/image_raw/compressed',self.image_callback,10)
        self.subscription  # prevent unused variable warning
        
        # Publisher to publish the position error
        self.publisher_ = self.create_publisher(Point, '/position', 10)

        # Targeting 20 FPS for processing to reduce CPU load
        self.target_fps = 20.0
        self.min_time_between_frames = 1.0 / self.target_fps
        self.last_processed_time = self.get_clock().now()

        # Parameter to resize the input frames for faster processing (1.0 means no resizing)
        self.declare_parameter('resize_factor', 1.0) # To reduce the resolution for faster processing
        self.resize_factor = self.get_parameter('resize_factor').get_parameter_value().double_value

        self.get_logger().info(f'Aruco Node initialized.')

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
            cv_image = self.br.compressed_imgmsg_to_cv2(msg, desired_encoding='mono8')

            # Reducing the resolution for faster processing
            if self.resize_factor < 1.0 and self.resize_factor > 0.0:
                small_image = cv2.resize(cv_image, (0, 0), fx=self.resize_factor, fy=self.resize_factor)
            else:
                small_image = cv_image
            
            # Dinamically get the dimensions of the frame to calculate the center (Setpoint)
            height, width = small_image.shape[:2]
            
            # Exact center of the frame (Setpoint)
            frame_center_x = width / 2.0
            frame_center_y = height / 2.0

            # ArUco Marker Detection
            corners, ids, _ = aruco.detectMarkers(small_image, self.aruco_dict, parameters=self.parameters)

            if ids is not None:
                # Flatten the ID array for easier searching
                ids_flat = ids.flatten()

                if self.target_id == -1:
                    for marker_id in ids_flat:
                        if marker_id <= self.id_limit and marker_id not in self.marker_queue: # Avoid detecting markers that are not used for testing
                            self.target_id = marker_id
                            self.follow_marker_time = self.get_clock().now() # Reset the follow marker timer
                            self.get_logger().info(f'Auto-detected target ID: {self.target_id}')
                            break
                    if self.target_id == -1:
                        cycle_array = []
                        valid_blacklisted_ids = []
                        for marker_id in ids_flat:
                            if marker_id <= self.id_limit and marker_id in self.marker_queue: # Only consider markers that are used for testing
                                position = self.marker_queue.index(marker_id) + 1
                                cycle_array.append(position)
                                valid_blacklisted_ids.append(marker_id)
                        if not valid_blacklisted_ids:
                            return # No valid blacklisted markers found, exit the block
                        ids_str = ", ".join(map(str, valid_blacklisted_ids))
                        cycles_str = ", ".join(map(str, cycle_array))
                        if len(cycle_array) == 1 and position == 1:
                            self.get_logger().info(f'Marker ID {ids_str} blacklisted, cannot be tracked for another cycle.', throttle_duration_sec=5.0)
                        elif len(cycle_array) == 1:
                            self.get_logger().info(f'Marker ID {ids_str} blacklisted, cannot be tracked for another {cycles_str} cycles.', throttle_duration_sec=5.0)
                        else:
                            self.get_logger().info(f'Marker IDs {ids_str} blacklisted, cannot be tracked respectively for another {cycles_str} cycles.', throttle_duration_sec=5.0)
                        return # No new marker found, exit the block
                
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
                    # self.get_logger().info(f'TARGET ID {self.target_id} - Error X: {real_error_x:.2f}, Y: {real_error_y:.2f}', throttle_duration_sec=0.1)

            if self.target_id != -1 and self.queue_size > 0:
                current_follow_time = self.get_clock().now()
                if (current_follow_time - self.follow_marker_time) > rclpy.duration.Duration(seconds=self.follow_time):
                    self.marker_queue.append(self.target_id) # Add the marker to the queue to avoid immediate re-detection
                    if self.queue_size == 1:
                        self.get_logger().info(f'Tracking timeout ({self.follow_time}s) reached. Marker ID {self.target_id} blacklisted until another target is tracked.')
                    else:
                        self.get_logger().info(f'Tracking timeout ({self.follow_time}s) reached. Marker ID {self.target_id} blacklisted for the next {self.queue_size} targets.')
                    self.target_id = -1

        except Exception as e:
            self.get_logger().error(f'Error in callback: {str(e)}')

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