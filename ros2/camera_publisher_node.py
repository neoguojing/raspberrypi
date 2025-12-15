import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from camera.camera import RpiCamera # 确保这个路径正确

class CameraPublisherNode(Node):
    def __init__(self):
        super().__init__('camera_publisher_node')
        self.get_logger().info('📷 摄像头发布节点启动...')

        self.bridge = CvBridge()
        self.camera_driver = RpiCamera()

        # 参数设置
        self.declare_parameter('camera_frequency', 15.0)
        self.camera_frequency = self.get_parameter('camera_frequency').get_parameter_value().double_value

        # 发布器：发布图像帧
        self.image_publisher = self.create_publisher(Image, 'image_raw', 10)
        
        # 定时器：周期性发布图像帧
        self.timer = self.create_timer(1.0 / self.camera_frequency, self.image_timer_callback)

    
    def image_timer_callback(self):
        """定时器触发，用于周期性地发布 Image 数据。"""
        current_time = self.get_clock().now().to_msg()
        self.publish_image(current_time)


    def publish_image(self, timestamp):
        """
        从 RpiCamera 获取图像帧并使用 cv_bridge 发布 ROS 2 Image 消息。
        """
        try:
            # 1. 获取 BGR 格式的 OpenCV 图像
            cv_image = self.camera_driver.get_frame()

            # 2. 使用 cv_bridge 转换为 ROS 2 Image 消息，指定 'bgr8' 编码
            image_msg = self.bridge.cv2_to_imgmsg(cv_image, encoding='bgr8')

            # 3. 设置 Header
            image_msg.header.stamp = timestamp
            image_msg.header.frame_id = 'camera_link'

            # 4. 发布消息
            self.image_publisher.publish(image_msg)

        except Exception as e:
            self.get_logger().error(f'发布图像失败: {e}')


def main_camera_publisher(args=None):
    rclpy.init(args=args)
    node = CameraPublisherNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('摄像头发布节点被中断...')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main_camera_publisher()