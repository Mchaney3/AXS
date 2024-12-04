import time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from .roboclaw.roboclaw_3 import Roboclaw  # Import from the local library

import logging

# Set up logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger('roboclaw_control')

logger.info("Starting")

# Constants for RoboClaw
ROBOCLAW_ADDRESS = 0x80  # Update if necessary
MAX_SPEED = 127
SMOOTHING_FACTOR = 0.5  # Adjusted for smoother transitions

logger.info("Initializing Roboclaw")

# Initialize the serial connection to RoboClaw
rc = Roboclaw("/dev/ttyACM0", 115200)
if not rc.Open():
    logger.error("Failed to open RoboClaw serial port.")
else:
    logger.info("Successfully opened RoboClaw serial port.")

# Previous motor speeds for smoothing
prev_left_speed = 0
prev_right_speed = 0

def smooth_speed(current_speed, target_speed, smoothing_factor):
    return current_speed + smoothing_factor * (target_speed - current_speed)

class RoboclawControlNode(Node):
    def __init__(self):
        super().__init__('roboclaw_control_node')
        self.subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10
        )
        self.subscription  # prevent unused variable warning

        version = rc.ReadVersion(ROBOCLAW_ADDRESS)
        if not version[0]:
            logger.error("GETVERSION Failed")
        else:
            logger.info(f"RoboClaw Version: {repr(version[1])}")

        logger.info("Roboclaw Control Node initialized and subscribed to /cmd_vel")

    def cmd_vel_callback(self, msg):
        global prev_left_speed, prev_right_speed

        # Extract linear and angular velocities from cmd_vel
        linear_x = msg.linear.x
        angular_z = msg.angular.z

        logger.info(f"Received /cmd_vel: linear_x={linear_x}, angular_z={angular_z}")

        # Calculate the base motor speed from the linear velocity
        base_speed = int(linear_x * MAX_SPEED)

        # Calculate the turning speed based on the angular velocity
        turn_speed = int(angular_z * (MAX_SPEED / 2))

        # Apply differential drive logic
        target_left_speed = base_speed - turn_speed
        target_right_speed = base_speed + turn_speed

        # Constrain motor speeds to MAX_SPEED
        target_left_speed = max(min(target_left_speed, MAX_SPEED), -MAX_SPEED)
        target_right_speed = max(min(target_right_speed, MAX_SPEED), -MAX_SPEED)

        # Smooth the speed transitions
        smoothed_left_speed = int(smooth_speed(prev_left_speed, target_left_speed, SMOOTHING_FACTOR))
        smoothed_right_speed = int(smooth_speed(prev_right_speed, target_right_speed, SMOOTHING_FACTOR))

        logger.debug(f"Target Left: {target_left_speed}, Smoothed Left: {smoothed_left_speed}")
        logger.debug(f"Target Right: {target_right_speed}, Smoothed Right: {smoothed_right_speed}")

        # Send motor commands to RoboClaw
        if smoothed_left_speed >= 0:
            rc.ForwardM1(ROBOCLAW_ADDRESS, smoothed_left_speed)
        else:
            rc.BackwardM1(ROBOCLAW_ADDRESS, abs(smoothed_left_speed))

        if smoothed_right_speed >= 0:
            rc.ForwardM2(ROBOCLAW_ADDRESS, smoothed_right_speed)
        else:
            rc.BackwardM2(ROBOCLAW_ADDRESS, abs(smoothed_right_speed))

        # Update previous speeds
        prev_left_speed = smoothed_left_speed
        prev_right_speed = smoothed_right_speed

def main(args=None):
    rclpy.init(args=args)
    roboclaw_control_node = RoboclawControlNode()

    try:
        rclpy.spin(roboclaw_control_node)
    except KeyboardInterrupt:
        logger.info("Shutting down Roboclaw Control Node")

    roboclaw_control_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
