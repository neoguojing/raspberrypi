import argparse
import zenoh
import numpy as np
import cv2
import json
import math
import time
import sys
from robot.robot.vision.detector import SegDetector # 假设你的 SegDetector 已经改造为 ONNX

class ZenohSegScan:
    def __init__(self, config_path='config.json'):
        # --- 1. 参数设置 (模拟 ROS 2 Parameter) ---
        self.camera_x_offset = 0.08
        self.camera_y_offset = 0.0
        self.camera_height = 0.05
        self.camera_pitch = 0.1484
        self.max_range = 5.0
        self.min_range = 0.0
        
        # 激光雷达模拟参数
        self.angle_min = -0.8
        self.angle_max = 0.8
        self.angle_increment = 0.017
        self.num_readings = int(round((self.angle_max - self.angle_min) / self.angle_increment)) + 1
        self.range_min = 0.05
        self.range_max = 5.0

        # 加载相机内参
        self.load_sensor_config(config_path)

        # 初始化检测器
        self.detector = SegDetector()
        
        # --- 2. Zenoh 初始化 ---
        print("🔗 正在连接到 Zenoh 网络...")
        config = zenoh.Config()
        self.session = zenoh.open(config)
        
        # 话题定义 (对应 ROS 2 Bridge 映射路径)
        # 假设 ROS 2 话题是 /camera/image_raw/compressed
        self.image_topic = "rt/camera/image_raw/compressed"
        self.scan_topic = "rt/scan"

        # 订阅图像
        self.sub = self.session.declare_subscriber(self.image_topic, self.on_image_data)
        
        # 定义发布者 (发送处理后的 JSON)
        self.pub = self.session.declare_publisher(self.scan_topic)
        
        print(f"✅ 节点已就绪. 订阅: {self.image_topic}, 发布: {self.scan_topic}")

    def load_sensor_config(self, path):
        with open(path, 'r') as f:
            config = json.load(f)
        self.K = np.array(config['camera_matrix'], dtype=np.float32)
        self.dist_coeffs = np.array(config['dist_coeffs'], dtype=np.float32)
        self.cy = self.K[1, 2]

    def on_image_data(self, sample):
        """Zenoh 订阅回调"""
        try:
            # 1. 解码图像 (假设是 CompressedImage 字节流) 或者 Image字节流
            # ROS 2 Bridge 传输的 CompressedImage 负载通常就是 JPEG 数据
            # 但注意：某些 Bridge 可能会包含 ROS 消息头，这里直接尝试 imdecode
            # 如果解码失败，可能需要跳过前几个字节的 ROS Header
            print("🔹 收到新图像数据，大小:", len(sample.payload), "bytes")
            # 1. 解码图像并获取时间戳
            frame, stamp = self.decode_ros2_image(sample.payload, default_shape=(480, 640, 3))
            if frame is None:
                print("⚠ 无法解码图像")
                return
            print(f"🖼 图像解码成功: shape={frame.shape}, timestamp={stamp:.6f}")

            # 2. 推理检测
            uv_points, _ = self.detector.get_ground_contact_points(frame, render=False)
            print(f"🔍 推理完成，检测到 {len(uv_points)} 个接触点")
            # 3. 激光数据初始化
            scan_ranges = np.full(self.num_readings, np.float('inf'))

            # 4. 投影逻辑 (逻辑与原代码一致)
            valid_points = 0
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
                        
                    idx = int(round((angle - self.angle_min) / self.angle_increment))
                    idx = max(0, min(idx, self.num_readings - 1))

                    for di in (-1, 0, 1):
                        j = idx + di
                        if 0 <= j < self.num_readings:
                            scan_ranges[j] = min(scan_ranges[j], dist)
                    valid_points += 1
            print(f"📡 投影完成，有效激光点: {valid_points}/{len(uv_points)}")
            # 5. 发布结果 (封装为 JSON，方便 ROS 2 侧解析)
            self.publish_as_json(scan_ranges,stamp)
            
        except Exception as e:
            print(f"处理错误: {e}")

    def decode_ros2_image(self, payload, default_shape=(480, 640, 3)):
        """
        自动判定 ROS2 消息类型 (CompressedImage / Image)，返回 frame 和时间戳
        frame: np.ndarray (H, W, 3)
        stamp: float, Unix timestamp
        """
        import struct, time
        frame = None
        stamp = time.time()  # 默认使用当前时间

        # --- 1. 检测 JPEG 开头 (CompressedImage) ---
        if payload[:2] == b'\xff\xd8':  # JPEG SOI
            # 尝试解析前 8 字节为 ROS2 Header stamp
            if len(payload) >= 8:
                try:
                    sec, nsec = struct.unpack_from('<II', payload, 0)
                    stamp = sec + nsec * 1e-9
                    # 找真正 JPEG 开头
                    idx = payload.find(b'\xff\xd8', 8)
                    if idx != -1:
                        payload = payload[idx:]
                except Exception:
                    print("⚠ Header 时间戳解析失败，使用本地时间")
            nparr = np.frombuffer(payload, np.uint8)
            frame = cv2.imdecode(nparr, cv2.IMREAD_COLOR)
            return frame, stamp

        # --- 2. 尝试 raw Image ---
        try:
            frame = np.frombuffer(payload, np.uint8).reshape(default_shape)
            return frame, stamp
        except Exception:
            print("⚠ raw Image reshape 失败")

        # --- 3. fallback ---
        return None, stamp
    
    def publish_as_json(self, ranges,stamp):
        """将雷达数据以 JSON 格式发布到 Zenoh"""
        # 替换 inf 为一个大数，因为标准 JSON 不支持 Infinity
        ranges_list = [r if r != float('inf') else 10.0 for r in ranges]
        
        msg = {
            "stamp": stamp,
            "frame_id": "base_link",
            "angle_min": self.angle_min,
            "angle_max": self.angle_max,
            "angle_increment": self.angle_increment,
            "ranges": ranges_list,
            "range_min": self.range_min,
            "range_max": self.range_max
        }
        self.pub.put(json.dumps(msg).encode('utf-8'))

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

if __name__ == '__main__':
    # 1. 配置命令行参数解析
    parser = argparse.ArgumentParser(description="Zenoh YOLO Segmentation to LaserScan Node")
    parser.add_argument(
        '--config', 
        type=str, 
        default='config.json', 
        help='Path to the camera configuration JSON file (default: config.json)'
    )
    
    args = parser.parse_args()

    # 2. 传入解析后的路径
    try:
        node = ZenohSegScan(config_path=args.config)
        
        print(f"🌟 节点已启动，使用配置文件: {args.config}")
        while True:
            time.sleep(1)
            
    except FileNotFoundError:
        print(f"❌ 错误: 找不到配置文件 '{args.config}'，请检查路径。")
        sys.exit(1)
    except KeyboardInterrupt:
        print("\n👋 正在关闭 Zenoh 节点...")
    finally:
        # 建议在类中添加一个 close 方法或直接在这里关闭 session
        if 'node' in locals():
            node.session.close()