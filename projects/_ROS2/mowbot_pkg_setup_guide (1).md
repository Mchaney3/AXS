
# ROS 2 Python Package for Differential Drive Robot (Using Roboclaw)

This guide provides instructions on how to set up a Python-based ROS 2 package for controlling a differential drive robot using the **Roboclaw motor controller** and **PS4 controller** for teleoperation. The package is built for **ROS 2 Jazzy Jalisco**.

## Directory Structure

```bash
mowbot_pkg/
├── package.xml
├── setup.py
├── setup.cfg
├── resource/
│   └── mowbot_pkg
├── launch/
│   └── roboclaw_control_launch.py
├── mowbot_pkg/
│   ├── __init__.py
│   ├── roboclaw_control.py
│   └── roboclaw/  # This folder contains the Roboclaw library
│       ├── __init__.py
│       ├── roboclaw.py
│       └── (other required Roboclaw files)
└── config/
    └── ps4.config.yaml
```

## Step-by-Step Instructions

### 1. Create the Package Directory

Navigate to your ROS 2 workspace's `src` directory and create the package:

```bash
cd ~/ros2_ws/src
mkdir -p mowbot_pkg/{launch,config,mowbot_pkg,resource}
cd mowbot_pkg
```

### 2. `package.xml`

```xml
<?xml version="1.0"?>
<package format="3">
  <name>mowbot_pkg</name>
  <version>0.0.1</version>
  <description>A ROS 2 package for controlling a differential drive robot with Roboclaw and PS4 teleop.</description>

  <maintainer email="your_email@example.com">Your Name</maintainer>
  <license>MIT</license>

  <!-- Dependencies -->
  <buildtool_depend>ament_python</buildtool_depend>
  
  <depend>rclpy</depend>
  <depend>geometry_msgs</depend>
  <depend>joy</depend>
  <depend>teleop_twist_joy</depend>

  <export>
    <build_type>ament_python</build_type>
  </export>
</package>
```

### 3. `setup.py`

```python
from setuptools import setup
import os
from glob import glob

package_name = 'mowbot_pkg'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your_email@example.com',
    description='A ROS 2 package for controlling a differential drive robot with Roboclaw and PS4 teleop.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'roboclaw_control = mowbot_pkg.roboclaw_control:main',
        ],
    },
)
```

### 4. `setup.cfg` (Optional)

```ini
[develop]
script-dir=$base/lib/mowbot_pkg
[install]
install-scripts=$base/lib/mowbot_pkg
```

### 5. `resource/mowbot_pkg`

```bash
touch resource/mowbot_pkg
```

### 6. `mowbot_pkg/__init__.py`

```bash
touch mowbot_pkg/__init__.py
```

### 7. `mowbot_pkg/roboclaw_control.py`

```python
import time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from .roboclaw.roboclaw import Roboclaw  # Import from the local library

import logging

# Set up logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger('roboclaw_control')

# Constants for RoboClaw
ROBOCLAW_ADDRESS = 0x80  # Update if necessary
MAX_SPEED = 127
SMOOTHING_FACTOR = 0.1  # Adjusted for smoother transitions

# Initialize the serial connection to RoboClaw
rc = Roboclaw("/dev/ttyS0", 38400)
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
```

### 8. `launch/roboclaw_control_launch.py`

```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='mowbot_pkg',
            executable='roboclaw_control',
            name='roboclaw_control_node',
            output='screen',
            parameters=[{'use_sim_time': False}, 'config/ps4.config.yaml']
        )
    ])
```

### 9. `config/ps4.config.yaml`

```yaml
joy_node:
  dev: /dev/input/js0
  deadzone: 0.1
  autorepeat_rate: 20

teleop_twist_joy:
  axis_linear: 1  # Typically left stick vertical
  scale_linear: 1.0
  axis_angular: 0  # Typically right stick horizontal
  scale_angular: 1.0
  enable_button: 0  # X button
  enable_turbo_button: 1  # Circle button
  enable_turbo_scale: 2.0
```

### 10. Install Roboclaw as a Local Library

Manually copy the necessary Roboclaw library files into the `mowbot_pkg/roboclaw/` directory. These files can be found in the [Roboclaw Python library GitHub repository](https://github.com/basicmicro/roboclaw_python_library).

### 11. Build the Package with `colcon`

```bash
cd ~/ros2_ws
colcon build --packages-select mowbot_pkg
```

