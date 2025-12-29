import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image,CompressedImage
from cv_bridge import CvBridge
from robot.camera.camera import RpiCamera
import cv2
import numpy as np
import json
import os

class CameraPublisherNode(Node):
    def __init__(self,compressed=True):
        super().__init__('camera_publisher_node')
        self.get_logger().info('📷 加载参数')
    
        self.declare_parameter('config_path', 'config.json') #相机内参

        self.declare_parameter('camera_frequency', 15.0)

        # --- 1. 相机安装位姿参数 (Transform Parameters) ---
        self.declare_parameter('camera_x_offset', 0.08)    # 相机相对于 base_link 前向偏移
        self.declare_parameter('camera_y_offset', 0.0)     # 左右偏移（通常为0）
        self.declare_parameter('camera_height', 0.05)      # 相机安装高度 (Z)
        self.declare_parameter('camera_pitch', 0.1484)     # 俯仰角 (弧度)，正数为向下俯视

        # --- 2. 视觉算法性能参数 (Functional Parameters) ---
        self.declare_parameter('max_detection_range', 5.0) # 最大有效感知距离 (米)
        self.declare_parameter('min_detection_range', 0.0) # 最近感知距离 (米)

        # --- 3. 获取参数值并存储在变量中 ---
        # 加载相机内参
        config_path = self.get_parameter('config_path').get_parameter_value().string_value
        self.load_sensor_config(config_path)
        self.camera_frequency = self.get_parameter('camera_frequency').get_parameter_value().double_value
        self.camera_x_offset = self.get_parameter('camera_x_offset').get_parameter_value().double_value
        self.camera_y_offset = self.get_parameter('camera_y_offset').get_parameter_value().double_value
        self.camera_height = self.get_parameter('camera_height').get_parameter_value().double_value
        self.camera_pitch = self.get_parameter('camera_pitch').get_parameter_value().double_value
        
        self.max_range = self.get_parameter('max_detection_range').get_parameter_value().double_value
        self.min_range = self.get_parameter('min_detection_range').get_parameter_value().double_value

        self.get_logger().info('📷 摄像头发布节点启动...')
        self.compressed=compressed
        
        self.bridge = CvBridge()
        self.camera_driver = RpiCamera()

        # 发布器：发布图像帧
        if self.compressed:
            self.image_publisher = self.create_publisher(CompressedImage, '/camera/image_raw/compressed', 10)
        else:
            self.image_publisher = self.create_publisher(Image, '/camera/image_raw', 10)
        
        
        # 定时器：周期性发布图像帧
        self.timer = self.create_timer(1.0 / self.camera_frequency, self.image_timer_callback)

        self.camera_driver.start()

    def load_sensor_config(self, path):
        """从 JSON 文件加载相机参数，保持矩阵格式"""
        try:
            with open(path, 'r') as f:
                config = json.load(f)
            
            # 直接转换为 NumPy 矩阵 K
            self.K = np.array(config['camera_matrix'], dtype=np.float32)
            
            # 提取畸变系数
            self.dist_coeffs = np.array(config['dist_coeffs'], dtype=np.float32)
            
            # 记录关键中心点用于后续过滤（从 K 中提取）
            self.cy = self.K[1, 2]
            
            print("--- 相机配置加载成功 ---")
            print(f"K 矩阵:\n{self.K}")
            print(f"畸变系数: {self.dist_coeffs}")
        except Exception as e:
            print(f"加载配置文件失败: {e}")

     # 像素坐标到，ros坐标的转换，参考系base_footprint
    def pixel_to_base(self, u, v):
        # 0. 基础过滤：地平线以上不处理
        if v < self.cy: return None

        # 1. 获取归一化像平面坐标 (xn, yn)
        # 此时得到的 (xn, yn) 已经消除了广角畸变，是在单位焦距平面上的投影
        pts = np.array([[[u, v]]], dtype=np.float32)
        undist_pts = cv2.undistortPoints(pts, self.K, self.dist_coeffs)
        xn, yn = undist_pts[0][0]

        # 2. ✅ 物理严格正确：构建并单位化相机光学射线 (Optical Frame)
        # Optical Frame: X-右, Y-下, Z-前
        ray_opt = np.array([xn, yn, 1.0])
        ray_opt /= np.linalg.norm(ray_opt) # 归一化方向矢量

        # 3. 坐标系转换 (Optical -> Robot base_link)
        # 符合 REP-103: Base_X=Opt_Z, Base_Y=-Opt_X, Base_Z=-Opt_Y
        r_vec = np.array([
            ray_opt[2],  # 前
           -ray_opt[0],  # 左
           -ray_opt[1]   # 上
        ])

        # 4. 处理 Pitch (绕机器人 Y 轴旋转)
        # 注意：这里的 r_vec 已经是单位向量，旋转后 rb_z 的物理含义更明确
        p = self.camera_pitch
        c, s = np.cos(p), np.sin(p)
        rb_x = r_vec[0] * c - r_vec[2] * s
        rb_y = r_vec[1]
        rb_z = r_vec[0] * s + r_vec[2] * c

        # 5. 与地面 Z=0 求交 (射线 P = [0, 0, h] + t * rb_vec)
        # 求 t 使得 h + t * rb_z = 0
        if rb_z >= -1e-6: 
            return None # 射线水平或朝上
            
        t = -self.camera_height / rb_z
        
        # 6. 计算最终位置并截断
        X = (t * rb_x) + self.camera_x_offset
        Y = t * rb_y

        if 0 < X < self.max_range:
            return X, Y
        return None
    
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