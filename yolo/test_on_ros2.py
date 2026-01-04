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
        super().__init__('zenoh_to_ros2_bridge')

        # ROS2 Publisher
        self.pub = self.create_publisher(
            String,
            '/test_zenoh_to_ros2',
            10
        )

        # Zenoh session
        config = zenoh.Config()
        config.insert_json5(
            "connect/endpoints",
            '["tcp/127.0.0.1:7447"]'
        )
        self.session = zenoh.open(config)

        self.get_logger().info('Connected to Zenoh')

        # Zenoh subscriber
        self.sub = self.session.declare_subscriber(
            "rt/test_zenoh_to_ros2",
            self.zenoh_callback
        )

    def zenoh_callback(self, sample):
        try:
            # Zenoh payload 是 ZBytes
            data = bytes(sample.payload).decode("utf-8")

            self.get_logger().info(
                f'Zenoh received: {sample.key_expr} -> {data}'
            )

            # 封装成 ROS2 消息
            msg = String()
            msg.data = data

            self.pub.publish(msg)

        except Exception as e:
            self.get_logger().error(f'Zenoh decode failed: {e}')

    def destroy_node(self):
        self.session.close()
        super().destroy_node()


class Ros2Printer(Node):
    def __init__(self):
        super().__init__('ros2_printer')

        self.sub = self.create_subscription(
            String,
            '/test_zenoh_to_ros2',
            self.callback,
            10
        )

    def callback(self, msg):
        self.get_logger().info(
            f'ROS2 received: {msg.data}'
        )

# =============================
# Main
# =============================
def main():
    rclpy.init()
    
    ros2_pub_node = Ros2Publisher()
    ros2_sub_node = Ros2Subscriber()
    ros2_printer_node = Ros2Printer()

    try:
        executor = rclpy.executors.MultiThreadedExecutor()
        executor.add_node(ros2_pub_node)
        executor.add_node(ros2_sub_node)
        executor.add_node(ros2_printer_node)

        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        ros2_pub_node.destroy_node()
        ros2_sub_node.destroy_node()
        ros2_printer_node.destroy_node()

        rclpy.shutdown()

if __name__ == '__main__':
    main()
