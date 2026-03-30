# PID Controller Node for Gimbal Tracking:
# it receives pixel error from ArUco detector and angle feedback from the Gimbal,
# and computes the necessary speed/angle correction to center the target.
import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point, Twist

class PIDControlNode(Node):
    def __init__(self):
        super().__init__('pid_controller')

        # Declare PID Parameters:
        # With some tests using turtlesim, 
        # I determined good values for the PID gains to start with, 
        # but they can be tuned in real-time using ROS2 parameters.
        # Commands to change parameters in real-time (using ros2 param set):
        # ros2 param set /pid_controller kp_* <float_value>
        # ros2 param set /pid_controller ki_* <float_value>
        # ros2 param set /pid_controller kd_* <float_value>
        
        # Pitch (Tilt/Y-axis) PID gains
        self.declare_parameter('kp_pitch', 0.04) # Proportional gain that gives a good velocity response without too much overshoot
        self.declare_parameter('ki_pitch', 0.001) # Integral gain that helps eliminate steady-state error, small to avoid instability
        self.declare_parameter('kd_pitch', 0.005) # Derivative gain that helps dampen the response and reduce overshoot, small to avoid noise amplification

        # Yaw (Pan/Z-axis) PID gains
        self.declare_parameter('kp_yaw', 0.04)
        self.declare_parameter('ki_yaw', 0.001)
        self.declare_parameter('kd_yaw', 0.005)

        # State Variables for PID calculations
        self.integral_pitch = 0.0
        self.integral_yaw = 0.0
        self.prev_error_pitch = 0.0
        self.prev_error_yaw = 0.0
        # self.last_time = self.get_clock().now()

        # Feedback variables
        self.current_pitch_angle = 0.0
        self.current_yaw_angle = 0.0
        self.feedback_received = False

        # Subscribers
        # Subscribing to ArUco pixel error (Target offset from center)
        self.error_sub = self.create_subscription(
            Point,
            '/position',
            self.error_callback,
            10
        )
        
        # Subscribing to Gimbal IMU feedback (Current angles)
        self.feedback_sub = self.create_subscription(
            Twist,
            '/feedback',
            self.feedback_callback,
            10
        )

        # Publisher
        # Publishes the calculated control signal (Speed or Angle correction) to the Gimbal Driver
        self.control_pub = self.create_publisher(
            Twist,
            '/control',
            10
        )

        self.get_logger().info('PID Control Node initialized and ready for tuning.')

    def feedback_callback(self, msg):
        # Update current angles from Gimbal feedback
        self.current_pitch_angle = msg.angular.y
        self.current_yaw_angle = msg.angular.z
        self.feedback_received = True

    def error_callback(self, msg):
        # Calculate PID control output based on the pixel error from ArUco and the current feedback from the Gimbal.
        # Calculate time difference (dt)
        # -----------------------------------------------------------------
        # current_time = self.get_clock().now()
        # dt_duration = current_time - self.last_time
        # dt = dt_duration.nanoseconds / 1e9  # Convert nanoseconds to seconds
        # -----------------------------------------------------------------
        # Or use dt as the fixed sample time:
        dt = 0.1  # Assuming a fixed control loop of 10 Hz (0.1 seconds)

        # Prevent division by zero on the very first callback or fast bursts
        if dt <= 0.0:
            return

        # Fetch current tunable gains
        
        kp_pitch = self.get_parameter('kp_pitch').value
        ki_pitch = self.get_parameter('ki_pitch').value
        kd_pitch = self.get_parameter('kd_pitch').value

        kp_yaw = self.get_parameter('kp_yaw').value
        ki_yaw = self.get_parameter('ki_yaw').value
        kd_yaw = self.get_parameter('kd_yaw').value

        # The error is directly the pixel distance from the center (calculated by ArucoNode)
        error_pitch = msg.y  # Positive Y error means target is below center
        error_yaw = msg.x    # Positive X error means target is to the right

        # --- PITCH (Y-axis) PID Calculation ---
        p_pitch = kp_pitch * error_pitch
        self.integral_pitch += error_pitch * dt
        i_pitch = ki_pitch * self.integral_pitch
        d_pitch = kd_pitch * ((error_pitch - self.prev_error_pitch) / dt)
        control_pitch = p_pitch + i_pitch + d_pitch

        # --- YAW (Z-axis) PID Calculation ---
        p_yaw = kp_yaw * error_yaw
        self.integral_yaw += error_yaw * dt
        i_yaw = ki_yaw * self.integral_yaw
        d_yaw = kd_yaw * ((error_yaw - self.prev_error_yaw) / dt)
        control_yaw = p_yaw + i_yaw + d_yaw

        # To avoid overshooting and to keep the control signal within reasonable limits, we can clamp the output to a maximum speed.
        MAX_SPEED = 1.0
        control_yaw = max(min(control_yaw, MAX_SPEED), -MAX_SPEED)
        control_pitch = max(min(control_pitch, MAX_SPEED), -MAX_SPEED)

        # Imlementing the Anti-Windup structure for the integral term,
        # When the control detects from the feedback that the gimbal has reached its physical limit,
        # it resets the integral term to prevent it from accumulating further and causing overshoot when the target moves back within range.
        # I need to determine the physical limits of the gimbal in terms of pitch and yaw angles, for now I'll use some reasonable assumptions based on typical gimbal capabilities. 
        # These limits can be adjusted based on the actual hardware specifications.
        PITCH_LIMIT = math.radians(60.0)
        YAW_LIMIT = math.radians(120.0)
        if abs(self.current_pitch_angle) >= PITCH_LIMIT:
            self.integral_pitch = 0.0
            if (self.current_pitch_angle * control_pitch) > 0:
                control_pitch = 0.0 # Stop pitch movement if it's trying to move further in the same direction as the current pitch angle

        if abs(self.current_yaw_angle) >= YAW_LIMIT:
            self.integral_yaw = 0.0
            if (self.current_yaw_angle * control_yaw) > 0:
                control_yaw = 0.0 # Stop yaw movement if it's trying to move further in the same direction as the current yaw angle

        # Publish the control signal
        control_msg = Twist()
        control_msg.angular.x = 0.0   
        control_msg.angular.y = control_pitch 
        control_msg.angular.z = control_yaw

        control_msg.linear.x = 0.0
        control_msg.linear.y = 0.0
        control_msg.linear.z = 0.0

        self.control_pub.publish(control_msg)

        # Update state for the next iteration
        self.prev_error_yaw = error_yaw
        self.prev_error_pitch = error_pitch
        # self.last_time = current_time

        # Debugging output (can be commented out during actual operation)
        self.get_logger().debug(f'PID OUT -> Pitch: {control_pitch:.2f}, Yaw: {control_yaw:.2f}, (dt: {dt:.3f}s)')

def main(args=None):
    rclpy.init(args=args)
    node = PIDControlNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()