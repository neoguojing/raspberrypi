import argparse
import zenoh
import numpy as np
import cv2
import json
import math
import time
import sys
import struct
import os
import glob
# from robot.robot.vision.detector import SegDetector # 假设你的 SegDetector 已经改造为 ONNX
from robot.robot.vision.segformer import SegFormerDetector 

class ZenohSegScan:
    def __init__(self, config_path='config.json'):
        
        self.frame_count = 0
        self.skip_n = 3 # 每 3 帧处理 1 帧
        
        # --- 1. 参数设置 (模拟 ROS 2 Parameter) ---
        self.camera_x_offset = 0.1
        self.camera_y_offset = 0.0
        self.camera_height = 0.071
        self.camera_pitch = 0.1484
        
        # 激光雷达模拟参数
        self.angle_min = -1.0
        self.angle_max = 1.0
        self.angle_increment = 0.017
        self.num_readings = int(round((self.angle_max - self.angle_min) / self.angle_increment)) + 1
        self.range_min = 0.05
        self.range_max = 4.0
        # 保存上一次定位的障碍
        self.scan_ranges = np.full(self.num_readings, float('inf'))

        # 加载相机内参
        self.load_sensor_config(config_path)

        # 初始化检测器
        # self.detector = SegDetector(conf=0.05)
        self.detector = SegFormerDetector()
        
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
        print(f"camera config:{config}")
        self.K = np.array(config['camera_matrix'], dtype=np.float32)
        self.dist_coeffs = np.array(config['dist_coeffs'], dtype=np.float32)
        self.cy = self.K[1, 2]
        self.width = config['width']
        self.height = config['height']

    def on_image_data(self, sample):
        """Zenoh 订阅回调"""
        try:
            self.frame_count += 1
            # 1. 解码图像 (假设是 CompressedImage 字节流) 或者 Image字节流
            # ROS 2 Bridge 传输的 CompressedImage 负载通常就是 JPEG 数据
            # 但注意：某些 Bridge 可能会包含 ROS 消息头，这里直接尝试 imdecode
            # 如果解码失败，可能需要跳过前几个字节的 ROS Header
            payload_bytes = sample.payload.to_bytes()           
            # 1. 解码图像并获取时间戳
            frame, stamp = self.decode_ros2_image(payload_bytes, default_shape=(self.height, self.width, 3))
            if frame is None:
                print("⚠ 无法解码图像")
                return
            # print(f"🖼 图像解码成功: shape={frame.shape}, timestamp={stamp:.6f}")

            # 3. 激光数据初始化
            valid_points = 0
            uv_points = []
            
            # 2. 推理检测
            if self.frame_count % self.skip_n == 0:
                uv_points, _ = self.detector.get_ground_contact_points(frame, render=True)
                # print(f"🔍 推理完成，检测到 {len(uv_points)} 个接触点")
                # 4. 投影逻辑 (逻辑与原代码一致)
                valid_points = 0
                for u, v in uv_points:
                    res = self.pixel_to_base(u, v)
                    if res:
                        self.scan_ranges = np.full(self.num_readings, float('inf'))
                        x, y = res
                        # 计算从坐标原点 $(0, 0)$ 到点 $(x, y)$ 的欧几里得距离
                        dist = math.hypot(x, y)
                        # if dist < self.range_min or dist > self.range_max:
                        #     print(f"on_image_data：距离太远或太近，{dist}")
                        #     continue
                        # 计算从原点指向点 $(x, y)$ 的射线与 正 X 轴 之间的夹角（弧度）
                        angle = math.atan2(y, x)
                        if not (self.angle_min <= angle <= self.angle_max):
                            print(f"on_image_data：角度偏离，{angle}")
                            continue
                            
                        idx = int(round((angle - self.angle_min) / self.angle_increment))
                        idx = max(0, min(idx, self.num_readings - 1))

                        for di in (-1, 0, 1):
                            j = idx + di
                            if 0 <= j < self.num_readings:
                                # scan_ranges[j] = min(scan_ranges[j], dist)
                                self.scan_ranges[j] = dist
                                
                        valid_points += 1
            # 5. 条件发布
            # if valid_points > 0:
            if valid_points >= 0:
                print(f"📡 投影完成，有效激光点: {valid_points}/{len(uv_points)}，正在发布数据...{self.scan_ranges}")
            self.publish_as_json(self.scan_ranges, stamp)

            
        except Exception as e:
            print(f"处理错误: {e}")

    # 输出统一为bgr
    def decode_ros2_image(self, payload, default_shape=(480, 640, 3)):
        # 关键修复 1: 确保进入函数的是 bytes 类型，或者是支持切片的视图
        if hasattr(payload, 'to_bytes'):
            payload = payload.to_bytes()

        payload_len = len(payload)
        h, w, c = default_shape
        num_pixels = h * w * c
    
        stamp = time.time()
        frame = None
        def save_image(decode_type, max_files=50):
            # --- 3. 保存验证 ---
            if frame is None:
                return

            debug_dir = 'debug_images'
            if not os.path.exists(debug_dir):
                os.makedirs(debug_dir)

            # 1. 数量限制检查
            files = sorted(glob.glob(os.path.join(debug_dir, "*.jpg")))
            if len(files) >= max_files:
                # 删除最早的一张 (按文件名排序)
                try:
                    os.remove(files[0])
                except Exception:
                    pass
            # 3. 执行保存
            filename = f"{debug_dir}/frame_{int(time.time()*1000)}_{decode_type}.jpg"
            cv2.imwrite(filename, frame)
        # print(f"✅ 已保存验证图片: {filename}")
        
        # --- 2. 尝试 raw Image ---
        # 如果没找到 JPEG 头，可能是 raw 格式
        # 注意：Raw Image 也有 Header，payload 需要跳过 Header 才能正确 reshape
        # 假设 Header 长度约为 48 字节 (视 frame_id 长度而定)
        if payload_len >= num_pixels:
            try:
                
                # 这是一个 Trick：从末尾向前取数据，规避前面变长的 Header
                raw_data = np.frombuffer(payload, np.uint8)
                num_pixels = default_shape[0] * default_shape[1] * default_shape[2]
        
                if len(raw_data) >= num_pixels:
                    frame = raw_data[-num_pixels:].reshape(default_shape)
                    # 默认为RGB
                    frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
                    # save_image('rgb')
                    return frame, stamp
            except Exception as e:
                print(f"⚠ raw Image reshape 失败: {e}")
        
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
                # 解码 JPEG
                jpeg_data = payload[idx:]
                nparr = np.frombuffer(jpeg_data, np.uint8)
                frame = cv2.imdecode(nparr, cv2.IMREAD_COLOR)
                # save_image('compressed')
                if frame is not None:
                    return frame, stamp
            except Exception as e:
                print(f"⚠ jpeg Image reshape 失败: {e}")
        return None, stamp
    
    def publish_as_json(self, ranges,stamp):
        """将雷达数据以 JSON 格式发布到 Zenoh"""
        # 替换 inf 为一个大数，因为标准 JSON 不支持 Infinity
        safe_value = self.range_max + 1
        # ranges_list = [float(r) if (np.isfinite(r) and r < self.range_max) else safe_value for r in ranges]
        ranges_list = [float(r) if np.isfinite(r) else safe_value for r in ranges]
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
        
        print(f"📡 发布的json {ranges_list}")
        payload = json.dumps(msg).encode("utf-8")
        self.pub.put(payload=payload,
                        encoding="application/json")


    def pixel_to_base(self, u, v):
        """
        数学原理：射线-平面相交模型 (Ray-Plane Intersection)
        目的：将图像坐标 (u, v) 映射到地面参考系 (X, Y, Z=0)
        """

        # --- 1. 消除畸变与归一化 (Undistortion & Normalization) ---
        # 数学原理：针孔相机逆模型
        # 通过相机内参矩阵 K 的逆运算和畸变系数，将像素坐标转换为归一化像平面坐标 (xn, yn)。
        # xn = (u - cx) / fx, yn = (v - cy) / fy (在无畸变理想状态下)
        # 此时 xn, yn 表示在焦距 f=1 处的物理尺寸。
        pts = np.array([[[u, v]]], dtype=np.float32)
        undist_pts = cv2.undistortPoints(pts, self.K, self.dist_coeffs)
        xn, yn = undist_pts[0][0]

        # --- 2. 坐标系重映射：光学系到本体系 (Optical Frame -> Base Frame) ---
        # 数学原理：欧式空间轴向对齐 (REP-103 标准)
        # 相机光学系 (Optical): Z向前, X向右, Y向下
        # 机器人本体系 (Base): X向前, Y向左, Z向上
        # 映射关系：Base_X = Opt_Z(1.0), Base_Y = -Opt_X(-xn), Base_Z = -Opt_Y(-yn)
        # v_base_raw 是从相机光心发出的、在机器人水平视角下的方向向量。
        v_base_raw = np.array([1.0, -xn, -yn]) 

        # --- 3. 俯仰角旋转处理 (Pitch Rotation) ---
        # 数学原理：绕 Y 轴的旋转变换 (Rotation Matrix)
        # 相机向下低头 (pitch > 0)，相对于机器人系是一个绕 Y 轴的旋转。
        # 旋转矩阵 R_y(p) 作用于向量：
        # [rb_x]   [ cos(p)  0  sin(p)] [v_raw_x]
        # [rb_y] = [   0     1     0   ] [v_raw_y]
        # [rb_z]   [-sin(p)  0  cos(p)] [v_raw_z]
        # 该步骤将“水平相机系”下的射线旋转至“实际安装倾角”下的射线方向向量。
        
        p = self.camera_pitch
        c, s = np.cos(p), np.sin(p)
        
        rb_x = v_base_raw[0] * c + v_base_raw[2] * s
        rb_y = v_base_raw[1]
        rb_z = -v_base_raw[0] * s + v_base_raw[2] * c

        # --- 4. 射线与地面求交 (Ray-Plane Intersection) ---
        # 数学原理：线性比例相似性 / 参数化直线方程
        # 假设地面方程为 Z = 0。相机光心在 Base 系下的坐标为 (camera_x_offset, 0, camera_height)。
        # 射线方程：P = P_camera + t * V_ray
        # 分解到 Z 轴：0 = camera_height + t * rb_z  =>  t = -camera_height / rb_z
        # 其中 t 是缩放因子，表示射线从光心到达地面所需的步长。
        
        
        # 物理约束：如果 rb_z >= 0，说明射线水平或向上射向天空，永远不会与地面相交。
        if rb_z >= -1e-6: return None 
        
        t = -self.camera_height / rb_z
        
        # --- 5. 平移补偿 (Translation Compensation) ---
        # 数学原理：刚体变换的平移部分
        # X = 射线在 X 轴的延伸 + 相机相对于机器人中心的安装偏移
        # Y = 射线在 Y 轴的延伸 (通常相机居中安装，偏移为 0)
        X = t * rb_x + self.camera_x_offset
        Y = t * rb_y
        
        return X, Y
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