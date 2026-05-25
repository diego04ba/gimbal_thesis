import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import requests
from requests.auth import HTTPDigestAuth
from requests.auth import HTTPBasicAuth

class AxisDriver(Node):
    def __init__(self):
        super().__init__('axis_driver')
        self.get_logger().info('Axis Driver Node has been started.')

        self.declare_parameter('ip', '192.168.0.90')
        self.declare_parameter('user', 'root')
        self.declare_parameter('password', 'pass')

        self.ip = self.get_parameter('ip').value
        self.user = self.get_parameter('user').value
        self.password = self.get_parameter('password').value

        self.session = requests.Session()
        self.session.auth = HTTPDigestAuth(self.user, self.password)
        # self.session.auth = HTTPBasicAuth(self.user, self.password)

        self.subscription = self.create_subscription(Twist,'/control',self.ptz_control_callback,10)
        
        self.publisher = self.create_publisher(Twist, '/feedback',10)
        self.timer = self.create_timer(0.2, self.ptz_feedback)
        self.is_requesting_feedback = False

    def ptz_control_callback(self, msg):
        pan_speed = int((msg.angular.z / 2.0) * 100)
        tilt_speed = -int((msg.angular.y / 2.0) * 100)
        pan_speed = max(-100, min(100, pan_speed))
        tilt_speed = max(-100, min(100, tilt_speed))

        url = f"http://{self.ip}/axis-cgi/com/ptz.cgi"
        params = {'continuouspantiltmove': f"{pan_speed},{tilt_speed}"}
        try:
            self.session.get(url, params=params, timeout=0.1)
        except requests.exceptions.RequestException:
            pass
    
    def ptz_feedback(self):
        if self.is_requesting_feedback:
            return
        
        self.is_requesting_feedback = True
        url = f"http://{self.ip}/axis-cgi/com/ptz.cgi"
        params = {'query': 'position'}

        try:
            response = self.session.get(url, params=params, timeout=0.1)
            
            if response.status_code == 200:
                lines = response.text.splitlines()
                pan_pos = 0.0
                tilt_pos = 0.0

                for line in lines:
                    if line.startswith('pan='):
                        pan_pos = float(line.split('=')[1])
                    elif line.startswith('tilt='):
                        tilt_pos = float(line.split('=')[1])
                
                self.get_logger().info(f"PTZ STATE -> Pan: {pan_pos:.2f}°, Tilt: {tilt_pos:.2f}°", throttle_duration_sec=0.5)

                feedback_msg = Twist()
                feedback_msg.angular.z = pan_pos  
                feedback_msg.angular.y = tilt_pos 
                feedback_msg.angular.x = 0.0
                
                self.publisher.publish(feedback_msg)

        except Exception as e:
            self.get_logger().debug(f"Error reading feedback: {e}")
        finally:
            self.is_requesting_feedback = False
    
def main(args=None):
    rclpy.init(args=args)
    axis_driver = AxisDriver()
    try:
        rclpy.spin(axis_driver)
    except KeyboardInterrupt:
        axis_driver.get_logger().info('Currently stopping Axis movement and shutting down...')
        try:
            axis_driver.session.get(
                f"http://{axis_driver.ip}/axis-cgi/com/ptz.cgi", 
                params={'continuouspantiltmove': "0,0"}, 
                timeout=0.2
            )
        except:
            pass
    finally:
        axis_driver.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':    
    main()