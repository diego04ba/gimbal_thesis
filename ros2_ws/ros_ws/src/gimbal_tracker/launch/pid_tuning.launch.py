from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess

def generate_launch_description():
    return LaunchDescription([
        # Official ROS2 Turtlesim, 'turtle1' moved with 'turtle_teleop_key' node
        Node(
            package='turtlesim',
            executable='turtlesim_node',
            name='turtlesim'
        ),
        
        # Spawn 'turtle2' in Turtlesim
        ExecuteProcess(
            cmd=['ros2', 'service', 'call', '/spawn', 'turtlesim/srv/Spawn', "{x: 5.5, y: 5.5, theta: 0.0, name: \'turtle2\'}"],
            output='screen'
        ),

        # TurtleSim ArUco Simulation 
        Node(
            package='gimbal_tracker',
            executable='turtle_aruco_sim', 
            name='turtle_aruco_sim',
            output='screen'
        ),

        # PID Controller
        Node(
            package='gimbal_tracker',
            executable='pid_controller', 
            name='pid_controller',
            output='screen',
            remappings=[
                ('/control', '/turtle2/cmd_vel')
            ]
        )

        # ros2 run turtlesim turtle_teleop_key
        # Commands to change parameters in real-time (using ros2 param set):
        # ros2 param set /pid_controller kp_* <float_value>
        # ros2 param set /pid_controller ki_* <float_value>
        # ros2 param set /pid_controller kd_* <float_value>
    ])