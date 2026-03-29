# Using the SimpleBGC (SBGC) Protocol to interface with the BGC 2.2 gimbal controller.
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import serial
import struct

class GimbalDriver(Node):
    def __init__(self):
        super().__init__('gimbal_driver')

        # Serial Configuration: to modify as needed
        self.declare_parameter('serial_port', '/dev/ttyUSB0')
        self.declare_parameter('baudrate', 115200)
        
        port = self.get_parameter('serial_port').value
        baud = self.get_parameter('baudrate').value

        try:
            self.ser = serial.Serial(port, baud, timeout=0.01)
            self.get_logger().info(f"Connected to BGC on {port}")
        except Exception as e:
            self.get_logger().error(f"Failed to connect: {e}")
            self.ser = None

        # ROS Interface
        # Subscribe to PID output
        self.control_sub = self.create_subscription(Twist, '/control', self.control_callback, 10)
        
        # Publish Feedback to PID
        self.feedback_pub = self.create_publisher(Twist, '/feedback', 10)

        # Feedback Loop Timer, set to 10 Hz
        self.timer = self.create_timer(0.1, self.request_feedback_callback)

    def control_callback(self, msg):
        # It receives Pitch (y) and Yaw (z)from the PID controller and sends them to the BGC.
        if self.ser and self.ser.is_open:
            # Prepare SBGC CMD_CONTROL packet
            # Mode: 2 (Speed mode), Data: Pitch speed, Yaw speed
            self.send_sbgc_control(pitch=msg.angular.y, yaw=msg.angular.z)

    def request_feedback_callback(self):
        # Informing the PID controller about the current angles of the gimbal by reading from the BGC and publishing to /feedback topic.
        if self.ser and self.ser.is_open:
            # Send CMD_REALTIME_DATA request to BGC
            # Then parse the incoming bytes
            current_angles = self.read_sbgc_feedback()
            
            if current_angles:
                feedback_msg = Twist()
                feedback_msg.angular.x = 0.0
                feedback_msg.angular.y = current_angles['pitch']
                feedback_msg.angular.z = current_angles['yaw']
                
                feedback_msg.linear.x = 0.0
                feedback_msg.linear.y = 0.0
                feedback_msg.linear.z = 0.0

                self.feedback_pub.publish(feedback_msg)

    def send_sbgc_control(self, pitch, yaw):
        # Pack the control command into the SBGC protocol format and send it over serial.
        # Header ($), ID (C), Size, Checksum, Payload...
        # Note: BGC 2.2 8-bit requires specific binary framing
        pass 

    def read_sbgc_feedback(self):
        # Parse the serial buffer to extract the current pitch and yaw angles from the BGC's feedback.
        # Logic to read serial buffer and extract IMU angles
        return None # Placeholder

def main(args=None):
    rclpy.init(args=args)
    node = GimbalDriver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node.ser: node.ser.close()
        node.destroy_node()
        rclpy.shutdown()