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
import resource

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
        self.test_timer = self.create_timer(0.1, self.test_send_control)
        
        # Publish Feedback to PID
        self.feedback_pub = self.create_publisher(Twist, '/feedback', 10)

        # Feedback Loop Timer, set to 10 Hz
        self.timer = self.create_timer(0.1, self.request_feedback_callback)

    def control_callback(self, msg):
        # It receives Roll (x), Pitch (y) and Yaw (z) from the PID controller and sends them to the BGC.
        if self.ser is None:
            self.get_logger().error("Serial connection not established. Cannot send control commands.")
            return
        if not self.ser.is_open:
            self.get_logger().error("Serial port is not open. Cannot send control commands.")
            return
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

    def test_send_control(self):
        # This function is for testing purposes. It sends a simple oscillating command to the BGC to verify communication.
        if self.ser and self.ser.is_open:
            t = self.get_clock().now().nanoseconds / 1e9
            roll_cmd = 0.5 * math.sin(2 * math.pi * 0.5 * t) # Oscillates between -0.5 and 0.5
            pitch_cmd = 0.5 * math.cos(2 * math.pi * 0.5 * t) # Oscillates between -0.5 and 0.5
            yaw_cmd = 0.0 # No yaw command for testing

            self.send_sbgc_control(roll=roll_cmd, pitch=pitch_cmd, yaw=yaw_cmd)

    def send_sbgc_control(self, roll, pitch, yaw):
        # Convert Float ROS to Int16 for SBGC
        # Using a multiplier to convert PID output to a suitable range for the BGC
        # Tuning may be required if too slow or too fast.
        multiplier = 450.0

        roll_speed = int(roll * multiplier)
        pitch_speed = int(pitch * multiplier)
        yaw_speed = int(yaw * multiplier * 0) # Avoiding noise 

        self.get_logger().info(
            f"[SBGC_TX] ROS: r={roll:.3f}, p={pitch:.3f}, y={yaw:.3f} | "
            f"SPEED: R={roll_speed}, P={pitch_speed}, Y={yaw_speed}"
        )

        # Construct SBGC CMD_CONTROL Payload
        payload = struct.pack('<Bhhhhhh', # 13 bytes total
                              1, # Mode: Speed mode 
                              roll_speed, 0, # Roll
                              pitch_speed, 0, # Pitch
                              yaw_speed, 0 # Yaw
                              )
        
        # Checksum calculation
        cmd_id = 67 # 'C' for CMD_CONTROL
        payload_size = len(payload)
        header_checksum = (cmd_id + payload_size) % 256
        payload_checksum = sum(payload) % 256

        self.get_logger().debug(f"[SBGC_TX] Size={payload_size}, Hdr_CHK={header_checksum}, Pld_CHK={payload_checksum}")

        # Assemble the final packet
        packet = struct.pack('<cBBB', b'>', cmd_id, payload_size, header_checksum) + payload + struct.pack('<B', payload_checksum)

        hex_string = ' '.join(f'{byte:02X}' for byte in packet)
        self.get_logger().info(f"[SBGC_TX] RAW HEX: {hex_string}")
        
        # Send the packet over serial
        try:
            # self.ser.write(packet)
            bytes_written = self.ser.write(packet)
            if bytes_written == len(packet):
                self.get_logger().debug(f"[SBGC_TX] Success: written {bytes_written} byte.")
            else:
                self.get_logger().warning(f"[SBGC_TX] Attention: written {bytes_written} on {len(packet)} byte!")
                
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
                                # That will be your offset for the Pitch. Then do the same to the right/left for the Roll.
                            
                                # self.get_logger().info(f"Payload length: {len(payload)} | Data: {payload.hex()}")

                                # Assume offset values for older 8-bit boards (e.g., 38, 40 and 42).
                                # 'h' means signed 16-bit integer. '<' means Little-Endian.
                                # Change 38, 40 and 42 to the correct values you discover through debugging or the manual.
                                roll_raw = struct.unpack_from('<h', payload, offset=38)[0]
                                pitch_raw = struct.unpack_from('<h', payload, offset=40)[0]
                                yaw_raw = struct.unpack_from('<h', payload, offset=42)[0]
                            
                                multiplier_8bit = 0.1

                                roll_deg = roll_raw * multiplier_8bit
                                pitch_deg = pitch_raw * multiplier_8bit
                                yaw_deg = yaw_raw * multiplier_8bit * 0 # Avoiding noise

                                self.get_logger().info(f"GIMBAL STATE -> Roll: {roll_deg:.2f}°, Pitch: {pitch_deg:.2f}°, Yaw: {yaw_deg:.2f}°", throttle_duration_sec=1.0)
                                return {'roll': roll_deg, 'pitch': pitch_deg, 'yaw': yaw_deg}
                            
                            except struct.error as e:
                                # If we get the offset wrong or the packet is shorter than expected,
                                # struct raises an error. We ignore it to prevent the node from crashing.
                                self.get_logger().debug(f"Error extracting angles (Wrong offset?): {e}")
                                pass
        
        return None

def main(args=None):
    try:
        # Setting Core Dump limit to 0 (Soft limit, Hard limit)
        resource.setrlimit(resource.RLIMIT_CORE, (0, 0))
    except Exception as e:
        print(f"Could not disable core dumps: {e}")
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