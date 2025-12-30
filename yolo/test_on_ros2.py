import threading
import time
import json

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import zenoh

# =============================
# ROS2 Publisher (A)
# =============================
class Ros2Publisher(Node):
    def __init__(self):
        super().__init__('ros2_pub')
        self.pub = self.create_publisher(String, '/test_ros2_to_zenoh', 10)
        self.timer = self.create_timer(1.0, self.timer_callback)
        self.count = 0

    def timer_callback(self):
        msg = String()
        msg.data = f'Hello from ROS2 {self.count}'
        self.pub.publish(msg)
        self.get_logger().info(f'ROS2 Published: {msg.data}')
        self.count += 1

# =============================
# ROS2 Subscriber (D)
# =============================
class Ros2Subscriber(Node):
    def __init__(self):
        super().__init__('ros2_sub')
        self.sub = self.create_subscription(String, '/test_zenoh_to_ros2', self.callback, 10)

    def callback(self, msg):
        self.get_logger().info(f'ROS2 Received: {msg.data}')


# =============================
# Main
# =============================
def main():
    rclpy.init()
    
    ros2_pub_node = Ros2Publisher()
    ros2_sub_node = Ros2Subscriber()

    try:
        # spin ROS2 nodes
        rclpy.spin(ros2_pub_node)
        rclpy.spin(ros2_sub_node)
    except KeyboardInterrupt:
        pass
    finally:
        ros2_pub_node.destroy_node()
        ros2_sub_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
