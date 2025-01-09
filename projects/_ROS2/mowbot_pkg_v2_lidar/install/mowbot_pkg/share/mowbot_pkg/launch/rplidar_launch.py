from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='rplidar_ros',
            executable='rplidarNode',
            name='rplidar_node',
            output='screen',
            parameters=[{
                'serial_port': '/dev/ttyUSB0',  # Adjust if needed
                'serial_baudrate': 115200,      # Set baudrate for your Lidar
                'frame_id': 'lidar_link',       # Should match your URDF
                'inverted': False,
                'angle_compensate': True
            }],
        ),
    ])
