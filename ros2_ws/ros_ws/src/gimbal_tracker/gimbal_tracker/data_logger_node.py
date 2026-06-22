import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point, Twist
from std_msgs.msg import Float64, Int32
import csv
import os
import time

class DataLoggerNode(Node):
    def __init__(self):
        super().__init__('data_logger')

        self.declare_parameter('test_name', 'test_generic')
        self.test_name = self.get_parameter('test_name').get_parameter_value().string_value

        self.error_yaw = 0.0
        self.error_pitch = 0.0
        self.control_yaw = 0.0
        self.control_pitch = 0.0
        self.angle_yaw = 0.0
        self.angle_pitch = 0.0
        self.integral_pitch = 0.0 
        self.integral_yaw = 0.0
        self.target_id = -1
        self.kp = 0.0
        self.ki = 0.0
        self.kd = 0.0       

        self.create_subscription(Point, '/position', self.position_callback, 10)
        self.create_subscription(Point, '/debug/error', self.position_callback, 10)
        self.create_subscription(Twist, '/control', self.control_callback, 10)
        self.create_subscription(Twist, '/feedback', self.feedback_callback, 10)
        self.create_subscription(Float64, '/debug/integral_pitch', self.integral_pitch_callback, 10)
        self.create_subscription(Float64, '/debug/integral_yaw', self.integral_yaw_callback, 10)
        self.create_subscription(Int32, '/debug/target_id', self.target_id_callback, 10)
        self.create_subscription(Point, '/debug/pid_gains', self.gains_callback, 10)

        log_dir = os.path.join(os.getcwd(), 'test_logs')
        os.makedirs(log_dir, exist_ok=True)
        counter = 1
        filename = f"{self.test_name}_{counter}.csv"
        self.filepath = os.path.join(log_dir, filename)

        while os.path.exists(self.filepath):
            counter += 1
            filename = f"{self.test_name}_{counter}.csv"
            self.filepath = os.path.join(log_dir, filename)

        self.csv_file = open(self.filepath, mode='w', newline='')
        self.csv_writer = csv.writer(self.csv_file)

        self.csv_writer.writerow([
            'time', 'error_yaw', 'error_pitch', 
            'control_yaw', 'control_pitch', 
            'angle_yaw', 'angle_pitch', 
            'integral_pitch', 'integral_yaw', 'target_id',
            'kp', 'ki', 'kd'
        ])

        self.start_time = time.time()

        self.timer = self.create_timer(0.05, self.log_data)
        
        self.get_logger().info(f'Data Logger initialized. Writing to: {self.filepath}')

    def position_callback(self, msg):
        self.error_yaw = msg.x
        self.error_pitch = msg.y

    def control_callback(self, msg):
        self.control_pitch = msg.angular.y
        self.control_yaw = msg.angular.z

    def feedback_callback(self, msg):
        self.angle_pitch = msg.angular.y
        self.angle_yaw = msg.angular.z

    def integral_pitch_callback(self, msg):
        self.integral_pitch = msg.data

    def integral_yaw_callback(self, msg):
        self.integral_yaw = msg.data

    def target_id_callback(self, msg):
        self.target_id = msg.data

    def gains_callback(self, msg):
        self.kp = msg.x
        self.ki = msg.y
        self.kd = msg.z
    
    def log_data(self):
        current_time = time.time() - self.start_time
        self.csv_writer.writerow([
            f"{current_time:.3f}",
            f"{self.error_yaw:.2f}",
            f"{self.error_pitch:.2f}",
            f"{self.control_yaw:.2f}",
            f"{self.control_pitch:.2f}",
            f"{self.angle_yaw:.2f}",
            f"{self.angle_pitch:.2f}",
            f"{self.integral_pitch:.4f}",
            f"{self.integral_yaw:.4f}",
            self.target_id,
            f"{self.kp:.2f}",
            f"{self.ki:.2f}",
            f"{self.kd:.2f}"
        ])

    def destroy_node(self):
        self.csv_file.close()
        self.get_logger().info(f'File CSV saved correctly and closed.')
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = DataLoggerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()