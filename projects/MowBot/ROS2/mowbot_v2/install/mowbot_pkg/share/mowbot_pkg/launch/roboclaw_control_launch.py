from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Start joy_node to read from the PS4 controller
        Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            output='screen',
            parameters=[{'dev': '/dev/input/js0'}],  # Ensure this is the correct device
        ),

        # Start teleop_twist_joy to convert joystick input to Twist messages
        Node(
            package='teleop_twist_joy',
            executable='teleop_node',  # Updated to 'teleop_node' for ROS 2 Jazzy
            name='teleop_twist_joy_node',
            output='screen',
            parameters=['config/ps4.config.yaml']  # Load your PS4 controller config
        ),

        # Start your Roboclaw control node
        Node(
            package='mowbot_pkg',
            executable='roboclaw_control',
            name='roboclaw_control_node',
            output='screen',
        ),

        # Lidar Node (sllidar_ros2 package)
        Node(
            package='sllidar_ros2',
            executable='sllidar_node',
            name='sllidar_node',
            output='screen',
            parameters=[{
                'serial_port': '/dev/ttyUSB0',
                'serial_baudrate': 115200,
                'frame_id': 'laser',
                'inverted': False,
                'angle_compensate': True
            }]
        ),

        # Object Avoidance Node
        Node(
            package='mowbot_pkg',
            executable='object_avoidance',
            name='object_avoidance_node',
            output='screen',
        ),
    ])
