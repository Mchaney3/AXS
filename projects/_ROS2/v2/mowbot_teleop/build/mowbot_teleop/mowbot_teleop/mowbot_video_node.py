import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
import threading

class MowbotVideoNode(Node):
    def __init__(self):
        super().__init__('mowbot_video_node')

        self.bridge = CvBridge()
        self.cv_window_name = 'MowBot v1.1'
        self.should_close = False
        self.current_frame = None
        self.frame_lock = threading.Lock()

        self.get_logger().info("MowbotVideoNode initialized.")

        # Set OpenCV window to full screen with new name
        cv2.namedWindow(self.cv_window_name, cv2.WND_PROP_FULLSCREEN)
        cv2.setWindowProperty(self.cv_window_name, cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_FULLSCREEN)

        # Initialize subscriptions
        self.image_subscription = self.create_subscription(
            Image,
            '/mowbot_image1',
            self.image_callback,
            10
        )
#        self.joy_subscription = self.create_subscription(
#            Joy,
#            '/joy',
#            self.joy_callback,
#            10
#        )

        # Start a thread to handle video display
        self.display_thread = threading.Thread(target=self.display_video_feed, daemon=True)
        self.display_thread.start()

    def image_callback(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

            # Acquire the lock to update the current frame safely
            with self.frame_lock:
                self.current_frame = frame

        except Exception as e:
            self.get_logger().error(f'Failed to process image: {e}')

    def joy_callback(self, msg):
        if len(msg.buttons) > 9 and msg.buttons[9] == 1:
            self.should_close = True

    def display_video_feed(self):
        while not self.should_close and rclpy.ok():
            # Acquire the lock to read the current frame safely
            with self.frame_lock:
                if self.current_frame is not None:
                    image = self.current_frame.copy()

                    # Display the image
                    cv2.imshow(self.cv_window_name, image)

            # Handle OpenCV window events
            if cv2.waitKey(1) & 0xFF == ord('q'):
                self.should_close = True

        # Cleanup when loop ends
        self.cleanup()

    def cleanup(self):
        self.get_logger().info("Destroying MowbotVideoNode and closing OpenCV windows.")
        if cv2.getWindowProperty(self.cv_window_name, cv2.WND_PROP_VISIBLE) >= 1:
            cv2.destroyWindow(self.cv_window_name)
        cv2.destroyAllWindows()
        self.get_logger().info("MowbotVideoNode resources cleaned up successfully.")

    def destroy_node(self):
        self.should_close = True
        self.display_thread.join()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = MowbotVideoNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down MowbotVideoNode")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
