# ==============================================================================
# Gimbal Driver Node for BGC 2.2 (Firmware 2.2b2 - 8-bit AlexMos)
# 
# CURRENT INTERFACE: 
# Overriding setpoints via USB/Serial using the SimpleBGC (SBGC) Binary Protocol.
# CMD_CONTROL ('C') is used to send speed commands.
# CMD_REALTIME_DATA ('D') is used to request IMU angles.
#
# ==============================================================================

import math

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
        # It receives Roll (x), Pitch (y) and Yaw (z) from the PID controller and sends them to the BGC.
        if self.ser and self.ser.is_open:
            # Prepare SBGC CMD_CONTROL packet
            # Mode: 2 (Speed mode), Data: Roll speed, Pitch speed, Yaw speed
            self.send_sbgc_control(roll=msg.angular.x, pitch=msg.angular.y, yaw=msg.angular.z)

    def request_feedback_callback(self):
        # Informing the PID controller about the current angles of the gimbal by reading from the BGC and publishing to /feedback topic.
        if self.ser and self.ser.is_open:
            # Send CMD_REALTIME_DATA request to BGC
            # Then parse the incoming bytes
            current_angles = self.read_sbgc_feedback()
            
            if current_angles:
                feedback_msg = Twist()
                feedback_msg.angular.x = math.radians(current_angles['roll'])
                feedback_msg.angular.y = math.radians(current_angles['pitch'])
                feedback_msg.angular.z = math.radians(current_angles['yaw'])

                feedback_msg.linear.x = 0.0
                feedback_msg.linear.y = 0.0
                feedback_msg.linear.z = 0.0

                self.feedback_pub.publish(feedback_msg)

    def send_sbgc_control(self, roll, pitch, yaw):
        # Convert Float ROS to Int16 for SBGC
        # Using a multiplier to convert PID output to a suitable range for the BGC
        # Tuning may be required if too slow or too fast.
        multiplier = 100.0

        roll_speed = int(roll * multiplier)
        pitch_speed = int(pitch * multiplier)
        yaw_speed = int(yaw * multiplier) 

        # Construct SBGC CMD_CONTROL Payload
        # Mode: 1 (Speed mode that ignores angle commands), Speed/Angle for Roll , Pitch, Yaw
        payload = struct.pack(
                              '<Bhhhhhh', # 13 bytes total
                              1, # Mode: Speed mode 
                              roll_speed, # Roll speed
                              0, #  Roll angle 
                              pitch_speed,
                              0, # Pitch angle 
                              yaw_speed,
                              0 # Yaw angle
                              )
        
        # Checksum calculation
        cmd_id = 67 # 'C' for CMD_CONTROL
        payload_size = len(payload)
        header_checksum = (cmd_id + payload_size) % 256
        payload_checksum = sum(payload) % 256

        # Assemble the final packet
        packet = struct.pack('<cBBB', b'>', cmd_id, payload_size, header_checksum) + payload + struct.pack('<B', payload_checksum)
        
        # Send the packet over serial
        try:
            self.ser.write(packet)
        except Exception as e:
            self.get_logger().error(f"Failed to send control command: {e}")

    def read_sbgc_feedback(self):
        # Send request for real-time data (CMD_REALTIME_DATA)
        cmd_id = 68 # 'D' for CMD_REALTIME_DATA
        request_packet = struct.pack('<cBBBB', b'>', cmd_id, 0, cmd_id %256, 0)

        try:
            self.ser.write(request_packet)
        except Exception as e:
            self.get_logger().debug(f"Write error: {e}")
            return None
        
        # Read and parse the response
        if self.ser.in_waiting >= 4:
            # Syncronization: Search for the start byte '>'
            if self.ser.read() == b'>':
                # Reads Header
                header_bytes = self.ser.read(3)
                
                if len(header_bytes) < 3:
                    self.get_logger().debug("Incomplete header received.")
                    return None

                resp_cmd = header_bytes[0]
                resp_size = header_bytes[1]
                resp_hdr_chk = header_bytes[2]

                # Verify Header Checksum
                if (resp_cmd + resp_size) % 256 == resp_hdr_chk:
                    # Reads Payload and Final Checksum
                    payload = self.ser.read(resp_size)
                    chk_byte = self.ser.read(1)

                    if len(payload) == resp_size and len(chk_byte) == 1:
                        payload_chk = struct.unpack('<B', chk_byte)[0]

                        # Verify Checksum of Payload
                        if sum(payload) % 256 == payload_chk:
                        
                            # Extraction of Angles
                            # Attention: The offsets for Roll, Pitch and Yaw in the payload may vary based on the BGC firmware version and configuration.
                            try: 
                                # Uncomment the line below the first time you run the node.
                                # Move the Gimbal by hand up and down: watch which bytes change in the terminal.
                                # That will be your offset for the Pitch. Then do the same to the right/left for the Yaw.
                            
                                self.get_logger().info(f"Payload length: {len(payload)} | Data: {payload.hex()}")

                                # Assume offset values for older 8-bit boards (e.g., 32, 34 and 36).
                                # 'h' means signed 16-bit integer. '<' means Little-Endian.
                                # Change 32, 34 and 36 to the correct values you discover through debugging or the manual.
                                roll_raw = struct.unpack_from('<h', payload, offset=32)[0]
                                pitch_raw = struct.unpack_from('<h', payload, offset=34)[0]
                                yaw_raw = struct.unpack_from('<h', payload, offset=36)[0]
                            
                                # BGC board maps angles with a resolution of 0.02197265625 degrees per unit
                                # (This value is derived from 360 degrees / 16384)
                                pitch_deg = pitch_raw * 0.02197265625
                                yaw_deg = yaw_raw * 0.02197265625
                                roll_deg = roll_raw * 0.02197265625

                                return {'roll': roll_deg, 'pitch': pitch_deg, 'yaw': yaw_deg}
                            
                            except struct.error as e:
                                # If we get the offset wrong or the packet is shorter than expected,
                                # struct raises an error. We ignore it to prevent the node from crashing.
                                self.get_logger().debug(f"Error extracting angles (Wrong offset?): {e}")
                                pass
        
        return None

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