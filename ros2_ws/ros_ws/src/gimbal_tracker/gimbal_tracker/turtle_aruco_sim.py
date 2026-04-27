import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from turtlesim.msg import Pose
import math

class TurtleArucoSim(Node):
    def __init__(self):
        super().__init__('turtle_aruco_sim')
        
        # Variables to store the poses of both turtles
        self.target_pose = None  # The prey (turtle1)
        self.gimbal_pose = None  # The hunter (turtle2)

        # Subscriptions to the poses of both turtles
        self.target_sub = self.create_subscription(Pose, '/turtle1/pose', self.target_callback, 10)
        self.gimbal_sub = self.create_subscription(Pose, '/turtle2/pose', self.gimbal_callback, 10)
        
        # Publisher of the error towards the PID
        self.error_pub = self.create_publisher(Point, '/position', 10)
        
        # A timer at 10 Hz (0.1s) to constantly calculate the error
        self.timer = self.create_timer(0.1, self.publish_error)
        
        self.get_logger().info("Turtle Aruco Sim (2 Turtles) avviato!")

    def target_callback(self, msg):
        self.target_pose = msg

    def gimbal_callback(self, msg):
        self.gimbal_pose = msg

    def publish_error(self):
        # Waits until we have both poses before calculating the error
        if self.target_pose is None or self.gimbal_pose is None:
            return

        # Calculate the angle from the gimbal (turtle2) to the target (turtle1)
        dy = self.target_pose.y - self.gimbal_pose.y
        dx = self.target_pose.x - self.gimbal_pose.x
        angle_to_target = math.atan2(dy, dx)
        
        # Calculate the yaw error (how much the gimbal needs to turn to face the target)
        error_yaw = angle_to_target - self.gimbal_pose.theta
        
        # Normalizes the error to be between -pi and pi
        error_roll = math.atan2(math.sin(error_yaw), math.cos(error_yaw))

        # To simulate pixel error:
        pixel_error_roll = error_roll * 100.0

        # Publish the error as a Point message (x = roll error, y and z unused)
        err_msg = Point()
        err_msg.x = pixel_error_roll
        err_msg.y = 0.0
        err_msg.z = 0.0
        
        self.error_pub.publish(err_msg)

def main(args=None):
    rclpy.init(args=args)
    node = TurtleArucoSim()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()