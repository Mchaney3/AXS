import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

class ObjectAvoidanceNode(Node):
    def __init__(self):
        super().__init__('object_avoidance_node')
        self.scan_subscriber = self.create_subscription(
            LaserScan,
            '/scan',  # Lidar scan topic
            self.scan_callback,
            10
        )
        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)  # Robot velocity publisher
        self.stop_distance = 0.5  # Stop if an object is within 0.5 meters

    def scan_callback(self, msg):
        # Get the minimum distance from the Lidar data
        min_distance = min(msg.ranges)
        
        # Create a Twist message for movement control
        twist = Twist()

        if min_distance < self.stop_distance:
            self.get_logger().info('Object too close! Stopping.')
            twist.linear.x = 0.0  # Stop forward motion
            twist.angular.z = 0.0  # Stop turning
        else:
            self.get_logger().info(f"Clear path: {min_distance} meters")
            twist.linear.x = 0.2  # Move forward
            twist.angular.z = 0.0  # No turning

        # Publish the velocity command
        self.cmd_vel_publisher.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = ObjectAvoidanceNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
