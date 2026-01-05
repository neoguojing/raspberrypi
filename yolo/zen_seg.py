import argparse
import zenoh
import numpy as np
import cv2
import json
import math
import time
import sys
import struct
from robot.robot.vision.detector import SegDetector # 假设你的 SegDetector 已经改造为 ONNX

class ZenohSegScan:
    def __init__(self, config_path='config.json'):
        
        self.frame_count = 0
        self.skip_n = 3 # 每 3 帧处理 1 帧
        
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
        self.detector = SegDetector(conf=0.1)
        
        # --- 2. Zenoh 初始化 ---
        print("🔗 正在连接到 Zenoh 网络...")
        config = zenoh.Config()
        config.insert_json5(
            "connect/endpoints",
            '["tcp/127.0.0.1:7447"]'
        )
        self.session = zenoh.open(config)
        
        # 话题定义 (对应 ROS 2 Bridge 映射路径)
        # 假设 ROS 2 话题是 /camera/image_raw/compressed
        self.image_topic = "rt/camera/image_raw"
        self.image_topic_compress = "rt/camera/image_raw/compressed"
        self.scan_topic = "rt/scan"

        # 订阅图像
        self.sub = self.session.declare_subscriber(self.image_topic, self.on_image_data)
        self.sub_compress = self.session.declare_subscriber(self.image_topic_compress, self.on_image_data)

        # 定义发布者 (发送处理后的 JSON)
        self.pub = self.session.declare_publisher(self.scan_topic)

        print(f"✅ 节点已就绪. 订阅: {self.image_topic},{self.image_topic_compress}, 发布: {self.scan_topic}")

    def load_sensor_config(self, path):
        with open(path, 'r') as f:
            config = json.load(f)
        self.K = np.array(config['camera_matrix'], dtype=np.float32)
        self.dist_coeffs = np.array(config['dist_coeffs'], dtype=np.float32)
        self.cy = self.K[1, 2]
        self.width = config['width']
        self.height = config['height']

    def on_image_data(self, sample):
        """Zenoh 订阅回调"""
        try:
            self.frame_count += 1
            if self.frame_count % self.skip_n != 0:
                return
            # 1. 解码图像 (假设是 CompressedImage 字节流) 或者 Image字节流
            # ROS 2 Bridge 传输的 CompressedImage 负载通常就是 JPEG 数据
            # 但注意：某些 Bridge 可能会包含 ROS 消息头，这里直接尝试 imdecode
            # 如果解码失败，可能需要跳过前几个字节的 ROS Header
            payload_bytes = sample.payload.to_bytes() 
        
            # print("🔹 收到新图像数据，大小:", len(payload_bytes), "bytes")
            
            # 1. 解码图像并获取时间戳
            frame, stamp = self.decode_ros2_image(payload_bytes, default_shape=(self.width, self.height, 3))
            if frame is None:
                print("⚠ 无法解码图像")
                return
            # print(f"🖼 图像解码成功: shape={frame.shape}, timestamp={stamp:.6f}")

            # 2. 推理检测
            uv_points, _ = self.detector.get_ground_contact_points(frame, render=True)
            print(f"🔍 推理完成，检测到 {len(uv_points)} 个接触点")
            # 3. 激光数据初始化
            scan_ranges = np.full(self.num_readings, np.inf)

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
            # 5. 条件发布
            if valid_points > 0:
                print(f"📡 投影完成，有效激光点: {valid_points}/{len(uv_points)}，正在发布数据...")
                self.publish_as_json(scan_ranges, stamp)
            else:
                # 这种情况直接跳过，不做任何网络传输
                # print(f"ℹ 帧内无有效接触点（valid_points=0），跳过发布。")
                pass
            
        except Exception as e:
            print(f"处理错误: {e}")

    def decode_ros2_image(self, payload, default_shape=(480, 640, 3)):
        # 关键修复 1: 确保进入函数的是 bytes 类型，或者是支持切片的视图
        if hasattr(payload, 'to_bytes'):
            payload = payload.to_bytes()

        stamp = time.time()
        frame = None

        # --- 1. 处理 ROS 2 消息头 (DDS 序列化通常会有额外开销) ---
        # ROS2 CompressedImage 的一般布局: 
        # [8字节 Stamp] [Frame_ID 长度 + 字符串] [Format 长度 + 字符串 "jpeg"] [数据]
        
        # 尝试寻找 JPEG 魔法数字 (0xFF, 0xD8)
        # 通常 JPEG 在 payload 中的偏移量在 40-100 字节之间
        idx = payload.find(b'\xff\xd8')

        if idx != -1:
            # 找到了 JPEG 开头，说明是压缩图像
            try:
                # 尝试提取时间戳：通常在消息最开始的 8 字节 (sec, nsec)
                # 注意：某些 Bridge 会在最前面加 4 字节的 CDR 封装头，如果是这样，偏移就是 4
                # 这里先尝试 0，如果时间戳看起来很离谱，可以尝试偏移 4 或 8
                sec, nsec = struct.unpack_from('<II', payload, 0)
                if 1e8 < sec < 2e9:  # 合法的时间戳范围检查
                    stamp = sec + nsec * 1e-9
                else:
                    # 尝试 CDR 偏移量
                    sec, nsec = struct.unpack_from('<II', payload, 4)
                    if 1e8 < sec < 2e9:
                        stamp = sec + nsec * 1e-9
            except Exception:
                pass

            # 解码 JPEG
            jpeg_data = payload[idx:]
            nparr = np.frombuffer(jpeg_data, np.uint8)
            frame = cv2.imdecode(nparr, cv2.IMREAD_COLOR)
            return frame, stamp

        # --- 2. 尝试 raw Image ---
        # 如果没找到 JPEG 头，可能是 raw 格式
        # 注意：Raw Image 也有 Header，payload 需要跳过 Header 才能正确 reshape
        # 假设 Header 长度约为 48 字节 (视 frame_id 长度而定)
        try:
            # 这是一个 Trick：从末尾向前取数据，规避前面变长的 Header
            raw_data = np.frombuffer(payload, np.uint8)
            num_pixels = default_shape[0] * default_shape[1] * default_shape[2]
            
            if len(raw_data) >= num_pixels:
                frame = raw_data[-num_pixels:].reshape(default_shape)
                return frame, stamp
        except Exception as e:
            print(f"⚠ raw Image reshape 失败: {e}")

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
        payload = json.dumps(msg).encode("utf-8")
        self.pub.put(payload=payload,
                        encoding="application/json")

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
        default='robot/config/imx219.json', 
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