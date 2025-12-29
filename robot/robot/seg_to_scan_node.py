import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, Image,CompressedImage
from cv_bridge import CvBridge
import numpy as np
from robot.vision.detector import SegDetector
import cv2
import json
import math

class SegScanNode(Node):
    def __init__(self):
        super().__init__('seg_to_scan_node')
        
        self.get_logger().info('📷 加载参数')
    
        self.declare_parameter('config_path', 'config.json') #相机内参
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
        self.camera_x_offset = self.get_parameter('camera_x_offset').get_parameter_value().double_value
        self.camera_y_offset = self.get_parameter('camera_y_offset').get_parameter_value().double_value
        self.camera_height = self.get_parameter('camera_height').get_parameter_value().double_value
        self.camera_pitch = self.get_parameter('camera_pitch').get_parameter_value().double_value
        
        self.max_range = self.get_parameter('max_detection_range').get_parameter_value().double_value
        self.min_range = self.get_parameter('min_detection_range').get_parameter_value().double_value

        # 1. 订阅与发布
        self.subscription = self.create_subscription(Image, '/camera/image_raw', self.image_callback, 10)
        self.subscription_compressed = self.create_subscription(CompressedImage, '/camera/image_raw/compressed', self.image_callback, 10)

        self.scan_pub = self.create_publisher(LaserScan, '/scan', 10)
        self.bridge = CvBridge()

        self.detector = SegDetector()

        # 3. 激光雷达模拟参数
        self.angle_min = -0.8  # 约 -45度
        self.angle_max = 0.8   # 约 +45度
        self.angle_increment = 0.017 # 1度
        self.num_readings = int(
            round((self.angle_max - self.angle_min) / self.angle_increment)
        ) + 1
        self.range_min = 0.05
        self.range_max = 2

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

    def image_callback(self, msg):
        try:
            # 1. 判断消息类型并解码为 BGR 格式的 numpy 数组
            if isinstance(msg, CompressedImage):
                # 处理压缩图像 (JPEG/PNG)
                frame = self.bridge.compressed_imgmsg_to_cv2(msg, desired_encoding='bgr8')
            else:
                # 处理原始图像 (Raw)
                frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
                            
            # 检测并投影
            uv_points, annotated_frame = self.detector.get_ground_contact_points(frame, render=True)
            scan_ranges = np.full(self.num_readings, np.inf)

            for u, v in uv_points:
                res = self.pixel_to_base(u, v)
                if res:
                    x, y = res
                    dist = math.hypot(x, y)
                    if dist < self.range_min or dist > self.range_max:
                        continue
                    angle = math.atan2(y, x)
                    if not (self.angle_min <= angle <= self.angle_max):
                        continue
                    # 将角度映射到 LaserScan 索引
                    idx = int(round((angle - self.angle_min) / self.angle_increment))
                    idx = max(0, min(idx, self.num_readings - 1))

                    # 角度扩散
                    for di in (-1, 0, 1):
                        j = idx + di
                        if 0 <= j < self.num_readings:
                            scan_ranges[j] = min(scan_ranges[j], dist)

            
            self.publish_scan(scan_ranges,msg.header.stamp)
        except Exception as e:
            self.get_logger().error(f'发布scan失败: {e}')

    def publish_scan(self, ranges,stamp):
        scan_msg = LaserScan()
        scan_msg.header.stamp = stamp
        scan_msg.header.frame_id = 'base_link'
        scan_msg.angle_min = self.angle_min
        scan_msg.angle_max = self.angle_max
        scan_msg.angle_increment = self.angle_increment
        scan_msg.range_min = self.range_min
        scan_msg.range_max = self.range_max
        scan_msg.ranges = ranges.tolist()
        self.scan_pub.publish(scan_msg)
    
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

def main():
    rclpy.init()
    node = SegScanNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()

if __name__ == '__main__':
    main()