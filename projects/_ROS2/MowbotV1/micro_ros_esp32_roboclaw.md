
# micro-ROS on ESP32 with RoboClaw Motor Controller

## Overview
This guide provides steps to implement a micro-ROS application on an ESP32 DevKit C that subscribes to the `cmd_vel` topic and controls two motors using a RoboClaw motor driver for differential steering.

## Prerequisites

1. **Install micro-ROS for Arduino:**
   - Open Arduino IDE and install the **micro-ROS library** by navigating to `Sketch -> Include Library -> Manage Libraries`.
   - Search for `micro-ROS` and install the library.

2. **Install RoboClaw Library:**
   - Similarly, install the **RoboClaw library** from Arduino’s Library Manager.
   - Go to `Sketch -> Include Library -> Manage Libraries`.
   - Search for `RoboClaw` and install the library.

3. **Wiring Setup:**
   - Connect the **ESP32** to the **RoboClaw motor driver** via UART (use ESP32 TX/RX pins).
   - Motors are connected to the M1 and M2 outputs of the RoboClaw motor controller.

## Code

Below is the code for the ESP32 that subscribes to the `cmd_vel` topic and drives the motors via the RoboClaw motor driver. The ESP32 will receive `geometry_msgs/Twist` messages containing linear and angular velocity data.

### ESP32 micro-ROS and RoboClaw Code

```cpp
#include <micro_ros_arduino.h>
#include <Wire.h>
#include <RoboClaw.h>
#include <geometry_msgs/msg/twist.h>

// Serial connection to RoboClaw (adjust to your setup)
#define ROBOCLAW_SERIAL_PORT Serial2
#define ROBOCLAW_BAUDRATE 38400
#define ROBOCLAW_ADDRESS 0x80  // Default RoboClaw address

// RoboClaw motor parameters
#define MAX_SPEED 3000  // Max motor speed, adjust according to your motors

// RoboClaw object initialization
RoboClaw roboclaw(&ROBOCLAW_SERIAL_PORT, ROBOCLAW_BAUDRATE);

// Micro-ROS variables
rcl_node_t node;
rcl_subscription_t subscription;
geometry_msgs__msg__Twist msg;

// Function to handle `cmd_vel` messages
void cmd_vel_callback(const void *msg_in) {
  const geometry_msgs__msg__Twist *msg = (const geometry_msgs__msg__Twist *)msg_in;
  
  // Extract linear and angular velocities from cmd_vel
  float linear_x = msg->linear.x;     // Forward/backward speed
  float angular_z = msg->angular.z;   // Turning speed
  
  // Differential drive calculations
  float left_motor_speed = linear_x - angular_z;
  float right_motor_speed = linear_x + angular_z;

  // Convert velocities to motor commands
  int left_speed = constrain(left_motor_speed * MAX_SPEED, -MAX_SPEED, MAX_SPEED);
  int right_speed = constrain(right_motor_speed * MAX_SPEED, -MAX_SPEED, MAX_SPEED);

  // Send commands to RoboClaw motors
  roboclaw.ForwardBackwardM1(ROBOCLAW_ADDRESS, left_speed);
  roboclaw.ForwardBackwardM2(ROBOCLAW_ADDRESS, right_speed);
}

void setup() {
  // Initialize serial communication
  Serial.begin(115200);
  ROBOCLAW_SERIAL_PORT.begin(ROBOCLAW_BAUDRATE);

  // Initialize RoboClaw motor driver
  if (!roboclaw.begin()) {
    Serial.println("Failed to initialize RoboClaw");
    while (1);
  }

  // Initialize micro-ROS transport (WiFi, Serial, or custom transport)
  set_microros_transports();

  // Initialize micro-ROS node and subscription
  node = rcl_get_zero_initialized_node();
  rclc_node_init_default(&node, "esp32_cmd_vel_node", rcl_get_default_allocator());
  
  subscription = rcl_get_zero_initialized_subscription();
  rclc_subscription_init_default(
    &subscription,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
    "cmd_vel"
  );
  
  // Add the subscription to the executor
  rclc_executor_t executor = rclc_executor_get_zero_initialized_executor();
  rclc_executor_init(&executor, &node, 1, rcl_get_default_allocator());
  rclc_executor_add_subscription(&executor, &subscription, &msg, &cmd_vel_callback, ON_NEW_DATA);
}

void loop() {
  // Spin the executor to handle messages
  rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
}
```

## Code Breakdown

### RoboClaw Initialization
- The RoboClaw motor controller is connected to the ESP32 via the `Serial2` port (pins RX2/TX2). This can be modified based on your setup.
- The `ROBOCLAW_BAUDRATE` is set to `38400`, which is the default baud rate for RoboClaw motor controllers.

### micro-ROS Setup
- The micro-ROS system is initialized using the `set_microros_transports()` function, which configures communication over Wi-Fi, serial, or other means. Ensure that the correct transport is set up for micro-ROS (Wi-Fi or serial).
- The `cmd_vel` topic is subscribed to using the `geometry_msgs__msg__Twist` message type, which carries `linear.x` (forward/backward velocity) and `angular.z` (turning velocity).

### Differential Drive Calculation
- The left and right motor speeds are calculated using the typical differential steering formula:  
  `left_motor_speed = linear_x - angular_z`  
  `right_motor_speed = linear_x + angular_z`
- The motor speeds are then constrained between the defined `MAX_SPEED` values and sent to the RoboClaw motor driver.

### RoboClaw Motor Control
- The `roboclaw.ForwardBackwardM1` and `roboclaw.ForwardBackwardM2` functions are used to control the two motors (M1 and M2) connected to the RoboClaw.

## Hardware Wiring

1. **ESP32 to RoboClaw**:
   - **TX2 (GPIO 17)** → **RoboClaw RX**  
   - **RX2 (GPIO 16)** → **RoboClaw TX**
   - Ensure that you connect **ground** between the ESP32 and the RoboClaw.

2. **Motors to RoboClaw**:
   - Connect motor 1 to the M1 terminals on the RoboClaw.
   - Connect motor 2 to the M2 terminals on the RoboClaw.

3. **Power**:
   - Make sure your motors and the RoboClaw motor controller are powered correctly according to their voltage requirements.

## Notes

- **Micro-ROS Transports**: Ensure you have set up the transport mechanism for micro-ROS (such as Wi-Fi, Ethernet, or Serial). For Wi-Fi transport, configure your network settings in the `set_microros_transports()` function. 
  Example for Wi-Fi transport:
  ```cpp
  set_microros_wifi_transports("your_ssid", "your_password", "192.168.1.100", 8888);
  ```
  Modify the IP address and port to match the Raspberry Pi or ROS 2 system.

- **Raspberry Pi Setup**: The Raspberry Pi or your ROS 2 system should publish to the `cmd_vel` topic. You can use teleoperation nodes, joystick controllers, or navigation stacks to publish `geometry_msgs/Twist` messages.

## Test the System

1. **On the Raspberry Pi** (or any ROS 2 machine):
   - Publish `cmd_vel` messages manually or using a teleop node:
     ```bash
     ros2 topic pub /cmd_vel geometry_msgs/msg/Twist '{linear: {x: 1.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.5}}'
     ```

2. **On the ESP32**:
   - Once the ESP32 is powered and connected to the network, it will receive messages from the `cmd_vel` topic and control the motors accordingly.

---

By following these steps, you'll have an ESP32 that subscribes to ROS 2 `cmd_vel` messages via micro-ROS and controls two motors using the RoboClaw motor driver in a differential drive setup.
