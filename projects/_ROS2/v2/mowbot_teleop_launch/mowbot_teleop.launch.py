import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # Path to Switch Config
    config_file_path = '/home/dadmin/mowbot_teleop/src/mowbot_teleop/config/switch_teleop.yaml'

    return LaunchDescription([
        # Launch Joy Node
        Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            output='screen',
        ),
        # Launch Twist Node
        Node(
            package='teleop_twist_joy',
            executable='teleop_node',
            name='teleop_twist_joy_node',
            parameters=[config_file_path],
            output='screen'
        ),
        # Launch Video Stream
        Node(
            package='mowbot_teleop',
            executable='mowbot_video_node',
            name='mowbot_video_node',
            output='screen',
        )
    ])

if __name__ == '__main__':
    generate_launch_description()
