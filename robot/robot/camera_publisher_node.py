import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image,CompressedImage,CameraInfo
from cv_bridge import CvBridge
import cv2
import json
import numpy as np
from copy import deepcopy

class CameraPublisherNode(Node):
    def __init__(self):
        super().__init__('camera_publisher_node')

        self.declare_parameter('camera_frequency', 15.0)
        self.camera_frequency = self.get_parameter('camera_frequency').get_parameter_value().double_value

        self.declare_parameter('is_camera', True)
        self.is_camera = self.get_parameter('is_camera').get_parameter_value().bool_value

        self.declare_parameter('source', '')
        self.source = self.get_parameter('source').get_parameter_value().string_value

        self.declare_parameter('compressed', True)
        self.compressed = self.get_parameter('compressed').get_parameter_value().bool_value

        self.declare_parameter('camera_config', '')
        self.camera_config = self.get_parameter('camera_config').get_parameter_value().string_value

        self.get_logger().info('📷 摄像头发布节点启动...')
        self.camera_config_loaded = False
        self.load_sensor_config(self.camera_config)
        self.camera_info_template = self.build_camera_info_template()

        self.bridge = CvBridge()
        # 发布器：发布图像帧
        if self.compressed:
            self.image_publisher = self.create_publisher(CompressedImage, '/camera/image_raw/compressed', 10)
        else:
            self.image_publisher = self.create_publisher(Image, '/camera/image_raw', 10)
        
        self.camera_info_publisher = self.create_publisher(CameraInfo, '/camera/camera_info', 10)

        
        # 定时器：周期性发布图像帧
        self.timer = self.create_timer(1.0 / self.camera_frequency, self.image_timer_callback)

        if self.is_camera:
            from robot.camera.camera import RpiCamera 
            self.camera_driver = RpiCamera()
            self.camera_driver.start()
        else:
            from robot.camera.video_reader import VideoReader
            self.camera_driver = VideoReader(self.source)
    
    def load_sensor_config(self, path):
        if not path:
            return 
        
        with open(path, 'r') as f:
            config = json.load(f)
        self.K = np.array(config['camera_matrix'], dtype=np.float32)
        self.dist_coeffs = np.array(config['dist_coeffs'], dtype=np.float32)
        self.cy = self.K[1, 2]
        self.width = config['width']
        self.height = config['height']

        self.camera_config_loaded = True

    def build_camera_info_template(self):
        if not self.camera_config_loaded:
            return None
        
        msg = CameraInfo()
        msg.header.frame_id = 'camera_optical_frame'

        msg.width = self.width
        msg.height = self.height

        msg.distortion_model = 'plumb_bob'

        # ---- D: 畸变参数（必须大写 D）----
        d = self.dist_coeffs.flatten().tolist()
        if len(d) < 5:
            d.extend([0.0] * (5 - len(d)))
        msg.D = d[:5]   # k1, k2, t1, t2, k3

        fx = float(self.K[0, 0])
        fy = float(self.K[1, 1])
        cx = float(self.K[0, 2])
        cy = float(self.K[1, 2])

        # ---- K: 相机内参矩阵（3x3）----
        msg.K = [
            fx, 0.0, cx,
            0.0, fy, cy,
            0.0, 0.0, 1.0
        ]

        # ---- R: 单目相机 = 单位阵 ----
        msg.R = [
            1.0, 0.0, 0.0,
            0.0, 1.0, 0.0,
            0.0, 0.0, 1.0
        ]

        # ---- P: 投影矩阵（3x4），单目无 baseline ----
        msg.P = [
            fx, 0.0, cx, 0.0,
            0.0, fy, cy, 0.0,
            0.0, 0.0, 1.0, 0.0
        ]

        return msg

    def publish_camera_info(self, stamp):
        if self.camera_info_template is None:
            return 
        
        msg = deepcopy(self.camera_info_template)
        msg.header.stamp = stamp

        self.camera_info_publisher.publish(msg)


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
            res = self.camera_driver.get_frame()
            
            if res is None or res[1] is None:
                return

            ts_from_driver, cv_image = res
            
            if self.compressed:
                # 1. 创建消息对象
                msg = CompressedImage()
                msg.header.stamp = self.get_clock().now().to_msg()
                msg.header.frame_id = "camera_link"
                msg.format = "jpeg" # 设置压缩格式
                
                # 2. 将 OpenCV 图像压缩为 JPEG 字节流
                # [int(cv2.IMWRITE_JPEG_QUALITY), 80] 80表示质量，数值越小压缩率越高
                success, buffer = cv2.imencode('.jpg', cv_image, [int(cv2.IMWRITE_JPEG_QUALITY), 80])
                if success:
                    msg.data = buffer.tobytes()
                    self.image_publisher.publish(msg)
                    print(f"image_publisher: CompressedImage: {msg.header}") # 看看输出是什么
            else:
                # 2. 使用 cv_bridge 转换为 ROS 2 Image 消息，指定 'bgr8' 编码
                # image_msg = self.bridge.cv2_to_imgmsg(cv_image, encoding='bgr8')
                image_msg = self.bridge.cv2_to_imgmsg(cv_image, encoding='rgb8')

                # 3. 设置 Header
                image_msg.header.stamp = timestamp
                image_msg.header.frame_id = 'camera_link'

                # 4. 发布消息
                self.image_publisher.publish(image_msg)
                print(f"image_publisher: image_msg: {image_msg.header}") # 看看输出是什么
            # 发布摄像头信息 
            self.publish_camera_info(timestamp)
        except Exception as e:
            self.get_logger().error(f'发布图像失败: {e}')
    

    def destroy_node(self):
        # 在这里显式关闭摄像头驱动，防止资源泄露
        self.get_logger().info('正在关闭摄像头驱动...')
        self.camera_driver.stop() 
        super().destroy_node()

def main(args=None):
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
    main()