
import time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from .roboclaw.roboclaw_3 import Roboclaw  # Import from the local library
import yaml
import logging

# Set up logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger('roboclaw_control')

logger.info("Starting")

# Constants for RoboClaw
ROBOCLAW_ADDRESS = 0x80  # Update if necessary
MAX_SPEED = 127
SMOOTHING_FACTOR = 0.1  # Adjusted for smoother transitions

logger.info("Initializing Roboclaw")

# Initialize the serial connection to RoboClaw
rc = Roboclaw("/dev/ttyACM0", 38400)
if not rc.Open():
    logger.error("Failed to open RoboClaw serial port.")
else:
    logger.info("Successfully opened RoboClaw serial port.")

# Previous motor speeds for smoothing
prev_left_speed = 0
prev_right_speed = 0

# Load PS4 controller configuration
def load_ps4_config(config_file):
    with open(config_file, 'r') as file:
        config = yaml.safe_load(file)
    return config

# Apply scaling based on ps4.config.yaml
def scale_speed(speed, scale_factor):
    return speed * scale_factor

# Smooth the speed transitions
def smooth_speed(current_speed, target_speed, smoothing_factor):
    return current_speed + smoothing_factor * (target_speed - current_speed)

class RoboclawControlNode(Node):
    def __init__(self):
        super().__init__('roboclaw_control')
        
        # Load PS4 config for scaling values
        self.ps4_config = load_ps4_config('/path/to/ps4.config.yaml')  # Set the correct path
        
        # Subscribing to cmd_vel topic to receive Twist messages
        self.subscription = self.create_subscription(
            Twist,
            'cmd_vel',
            self.listener_callback,
            10
        )
        
    def listener_callback(self, msg):
        global prev_left_speed, prev_right_speed

        # Extract linear and angular velocities from Twist message
        linear_vel = msg.linear.x
        angular_vel = msg.angular.z

        # Scale velocities based on PS4 config scaling factors
        linear_vel = scale_speed(linear_vel, self.ps4_config['scaling']['linear'])
        angular_vel = scale_speed(angular_vel, self.ps4_config['scaling']['angular'])

        # Calculate the motor speeds for the left and right tracks
        left_speed = linear_vel - angular_vel
        right_speed = linear_vel + angular_vel

        # Smooth the motor speed transitions
        left_speed = smooth_speed(prev_left_speed, left_speed, SMOOTHING_FACTOR)
        right_speed = smooth_speed(prev_right_speed, right_speed, SMOOTHING_FACTOR)

        # Convert speeds to RoboClaw format
        left_motor_speed = int(MAX_SPEED * left_speed)
        right_motor_speed = int(MAX_SPEED * right_speed)

        # Send motor speeds to RoboClaw
        rc.ForwardM1(ROBOCLAW_ADDRESS, left_motor_speed)
        rc.ForwardM2(ROBOCLAW_ADDRESS, right_motor_speed)

        # Update previous speeds for the next cycle
        prev_left_speed = left_speed
        prev_right_speed = right_speed
