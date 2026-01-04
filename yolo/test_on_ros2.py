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
        self.pub = self.create_publisher(String, '/test_zenoh_to_ros2', 10)
        config = zenoh.Config()
        config.insert_json5(
            "connect/endpoints",
            '["tcp/127.0.0.1:7447"]'
        )

        session = zenoh.open(config)
        def callback(sample):
            data = bytes(sample.payload).decode("utf-8")
            print(f"Zenoh Subscriber received: {sample.key_expr} -> {data}")
            msg = String()
            msg.data = data
            self.pub.publish(msg)
    
        sub = session.declare_subscriber("rt/test_zenoh_to_ros2", callback)

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
        executor = rclpy.executors.MultiThreadedExecutor()
        executor.add_node(ros2_pub_node)
        executor.add_node(ros2_sub_node)
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        ros2_pub_node.destroy_node()
        ros2_sub_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
