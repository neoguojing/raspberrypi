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
import threading

import queue

class ZenohSegScan:
    def __init__(self, config_path='config.json'):
        
        self.frame_count = 0
        self.skip_n = 3 # 每 3 帧处理 1 帧
        
        # --- 1. 参数设置 (模拟 ROS 2 Parameter) ---
        self.camera_x_offset = 0.1
        self.camera_y_offset = 0.0
        self.camera_height = 0.071
        self.camera_pitch = math.radians(8.5)
        
        # 激光雷达模拟参数
        self.angle_min = -math.radians(40)  # -40°
        self.angle_max = math.radians(40)   # 40°
        self.angle_increment = math.radians(0.5)
        self.num_readings = int(round((self.angle_max - self.angle_min) / self.angle_increment)) + 1
        self.range_min = self.camera_height / math.tan(self.camera_pitch)
        self.range_max = 3.0
        # 物理模拟：假设激光光斑有 1.5 倍角分辨率的宽度，产生重叠
        self.beam_overlap_indices = 2

        # 加载相机内参
        self.load_sensor_config(config_path)

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
        self.point_cloud_topic = "rt/pointcloud"

        # 订阅图像
        self.sub = self.session.declare_subscriber(self.image_topic, self.on_image_data)
        self.sub_compress = self.session.declare_subscriber(self.image_topic_compress, self.on_image_data)

        # 定义发布者 (发送处理后的 JSON)
        self.pub = self.session.declare_publisher(self.scan_topic)
        self.pointcloud_pub = self.session.declare_publisher(self.point_cloud_topic)
        
        self.latest_sample_lock = threading.Lock()
        self.latest_sample = None

        self.last_valid_scan = np.full(self.num_readings, self.range_max)
        self.scan_alpha = 0.5
        

        self.inference_queue = queue.Queue(maxsize=1)


        # 2. Inference Thread
        inference_thread = threading.Thread(target=self.inference_loop, daemon=True)
        inference_thread.start()

        self.process_thread = threading.Thread(target=self.processing_loop, daemon=True)
        self.process_thread.start()

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
        """回调函数现在极快：只负责存下最新的数据包"""
        try:
            self.frame_count += 1
            if self.frame_count % self.skip_n != 0:  # 每3帧打印一次
                return 
            
            with self.latest_sample_lock:
                self.latest_sample = sample

        except Exception as e:
            print(f"⚠ 入队失败: {e}")
    

    # ---------------- Inference Thread ----------------
    def inference_loop(self):
        # 初始化检测器
        # self.detector = SegDetector(conf=0.05)
        # from robot.robot.vision.segformer import SegFormerDetector 
        # self.detector = SegFormerDetector()
        
        from robot.robot.vision.segformer_trt import SegFormerTRTDetector 
        self.detector = SegFormerTRTDetector(debug=True)
        while True:
            try:
                with self.latest_sample_lock:
                    sample = self.latest_sample
                    self.latest_sample = None  # optional, 防止重复处理
                    
                if sample is None:
                    time.sleep(0.01)
                    continue
                
                payload_bytes = sample.payload.to_bytes()
                frame, stamp = self.decode_ros2_image(payload_bytes, default_shape=(self.height, self.width, 3))
                if frame is None:
                    return                # 推理
                uv_points, _ = self.detector.get_ground_contact_points(frame, render=True)
                # 入队推理结果
                if self.inference_queue.full():
                    try:
                        self.inference_queue.get_nowait()
                    except queue.Empty:
                        pass
                self.inference_queue.put_nowait((uv_points, stamp))
            except queue.Empty:
                time.sleep(0.001)
            except Exception as e:
                print(f"Inference 错误: {e}")
                time.sleep(0.1)

    def processing_loop(self):
        """Zenoh 订阅回调"""
        while True:
            try:
                uv_points, stamp = self.inference_queue.get(timeout=0.1)
                scan_ranges = np.full(self.num_readings, float('inf'))

                if len(uv_points) > 0:
                    xyz_points = self.pixel_to_base_batch(uv_points)
                    valid_mask = ~np.isnan(xyz_points[:, 0])
                    valid_xyz = xyz_points[valid_mask]

                    if len(valid_xyz) > 0:
                        x = valid_xyz[:, 0]
                        y = valid_xyz[:, 1]

                        # 1. 计算距离
                        dist = np.round(np.hypot(x, y) / 0.02) * 0.02
                        mask_dist = (dist >= self.range_min) & (dist <= self.range_max)

                        # 2. 计算角度
                        angle = np.arctan2(y, x)
                        mask_angle = (angle >= self.angle_min) & (angle <= self.angle_max)

                        # 3. 有效点掩码
                        mask_valid = mask_dist & mask_angle
                        if np.any(mask_valid):
                            dist = dist[mask_valid]
                            angle = angle[mask_valid]

                            # 4. 计算 idx
                            idx = np.clip(np.round((angle - self.angle_min) / self.angle_increment).astype(int), 0, self.num_readings-1)

                            # 5. 扩散 [-1,0,1] 并更新 scan_ranges
                            for di in (-1, 0, 1):
                                idx_shift = idx + di
                                mask_in = (idx_shift >= 0) & (idx_shift < self.num_readings)
                                np.minimum.at(scan_ranges, idx_shift[mask_in], dist[mask_in])

                # 6. EMA 平滑
                if self.last_valid_scan is not None:
                    current_valid = np.isfinite(scan_ranges)
                    previous_valid = np.isfinite(self.last_valid_scan)
                    blend_mask = current_valid & previous_valid
                    scan_ranges[blend_mask] = (self.scan_alpha * scan_ranges[blend_mask] + 
                                            (1 - self.scan_alpha) * self.last_valid_scan[blend_mask])

                # 7. 发布
                self.last_valid_scan = scan_ranges.copy()
                self.publish_as_json(scan_ranges, stamp)

            except queue.Empty:
                time.sleep(0.001)
            except Exception as e:
                print(f"处理错误: {e}")
                time.sleep(0.1)

    def get_accurate_stamp(self,payload):
        try:
            # ROS 2 序列化后的前 4 字节是封装头 (Representation Identifier)
            # 紧接着就是消息内容。对于 Image/CompressedImage，第一个字段是 Header。
            # Header 的第一个字段是 Stamp (sec, nanosec)。
            
            # 尝试从偏移量 4 开始读取 (跳过 4 字节的 CDR Header)
            sec, nsec = struct.unpack_from('<II', payload, 4)
            stamp = sec + nsec * 1e-9
            # print(f"🕒 提取准确时间戳: {stamp:.6f}")
            return stamp
        except Exception:
            return time.time()
    
    # 输出统一为rgb
    def decode_ros2_image(self, payload, default_shape=(480, 640, 3)):
        # 关键修复 1: 确保进入函数的是 bytes 类型，或者是支持切片的视图
        if hasattr(payload, 'to_bytes'):
            payload = payload.to_bytes()

        payload_len = len(payload)
        h, w, c = default_shape
        num_pixels = h * w * c
    
        stamp = time.time()
        frame = None
        # frame统一为RGB
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
            cv2.imwrite(filename, frame[:, :, ::-1])
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
                    # frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
                    stamp = self.get_accurate_stamp(payload)
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
                # 解码 JPEG
                jpeg_data = payload[idx:]
                nparr = np.frombuffer(jpeg_data, np.uint8)
                frame = cv2.imdecode(nparr, cv2.IMREAD_COLOR)
                if frame is not None:
                    frame = frame[:, :, ::-1]   # BGR -> RGB
                    stamp = self.get_accurate_stamp(payload)
                    # save_image('compressed')
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

        # if np.any(np.isfinite(ranges)):
        print(f"📡 发布有效json {ranges_list}")
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
        if rb_z >= -1e-6: 
            # print(f"射线射向天空，无法与地面相交")
            return None 
        
        t = -self.camera_height / rb_z
        
        # --- 5. 平移补偿 (Translation Compensation) ---
        # 数学原理：刚体变换的平移部分
        # X = 射线在 X 轴的延伸 + 相机相对于机器人中心的安装偏移
        # Y = 射线在 Y 轴的延伸 (通常相机居中安装，偏移为 0)
        X = t * rb_x + self.camera_x_offset
        Y = t * rb_y
        
        return X, Y
    
    def pixel_to_base_batch(self, uv_points):
        """
        批量将像素坐标 (u, v) 映射到地面参考系 (X, Y, Z=0)
        Args:
            uv_points: np.ndarray, shape [N, 2], 元素为 [[u1, v1], [u2, v2], ...]
        Returns:
            points_base: np.ndarray, shape [N, 2], 元素为 [[X1, Y1], [X2, Y2], ...]
                        无法相交的点将被过滤或返回 NaN
        """
        if len(uv_points) == 0:
            return np.empty((0, 2))

        # --- 优化1：ROI过滤 (只看图像高度 85% 以上的部分，防止车头干扰) ---
        uv_points = np.atleast_2d(uv_points)
        mask_roi = uv_points[:, 1] < (self.height * 0.85)
        # --- 1. 批量消除畸变与归一化 ---
        # uv_points shape: [N, 2] -> reshape 为 cv2 要求的 [N, 1, 2]
        pts = np.array(uv_points, dtype=np.float32).reshape(-1, 1, 2)
        if self.dist_coeffs is None or self.dist_coeffs.size == 0:
            self.dist_coeffs = np.zeros(5, dtype=np.float64)
            
        undist_pts = cv2.undistortPoints(pts, self.K, self.dist_coeffs)
        # 得到归一化坐标 xn, yn (shape: [N, 2])
        n_coords = undist_pts.reshape(-1, 2)
        xn = n_coords[:, 0]
        yn = n_coords[:, 1]

        # --- 2. 坐标系重映射 (Optical -> Base) ---
        # v_base_raw shape: [N, 3]
        ones = np.ones_like(xn)
        v_base_raw = np.column_stack([ones, -xn, -yn])

        # --- 3. 批量俯仰角旋转处理 ---
        p = self.camera_pitch
        c, s = np.cos(p), np.sin(p)
        
        # 按照你提供的公式进行矩阵运算
        rb_x = v_base_raw[:, 0] * c + v_base_raw[:, 2] * s
        rb_y = v_base_raw[:, 1]
        rb_z = -v_base_raw[:, 0] * s + v_base_raw[:, 2] * c

        # --- 4. 射线与地面求交 ---
        # 物理约束过滤：只保留射向地面的点 (rb_z < -1e-6)
        # valid_mask = rb_z < -1e-6
        valid_mask = (rb_z < -0.01) & mask_roi
        
        # 初始化结果矩阵
        num_pts = len(uv_points)
        results = np.full((num_pts, 2), np.nan) # 默认填充 NaN 表示无效点

        if not np.any(valid_mask):
            return results

        # 只计算有效点
        t = -self.camera_height / rb_z[valid_mask]
        
        # --- 5. 平移补偿 ---
        X = t * rb_x[valid_mask] + self.camera_x_offset
        Y = t * rb_y[valid_mask]
        
        # --- 优化3：物理距离二次过滤 (过滤掉 0.2m 以内的抖动点) ---
        dist_sq = X**2 + Y**2
        phys_mask = dist_sq > (0.2 ** 2)
        
        # 定义最终能够写入 results 的布尔掩码（长度为 num_pts）
        valid_and_far = np.zeros(num_pts, dtype=bool)
        valid_and_far[valid_mask] = phys_mask 

        # 【关键修正】：使用 phys_mask 提取 X 和 Y 中真正有效的元素
        results[valid_and_far] = np.column_stack([X[phys_mask], Y[phys_mask]])

        return results
    
    def refine_wall_points(self, points_xyz):
        """
        利用直线拟合修正沿着墙方向的透视偏移
        """
        # 如果点太少，无法拟合直线，直接返回原数据
        if len(points_xyz) < 10: 
            return points_xyz
        
        try:
            # 1. 使用 cv2.fitLine 进行直线拟合 (最小二乘法)
            # 返回 [vx, vy, x0, y0]，其中 (vx, vy) 是方向向量，(x0, y0) 是直线上的一点
            # 这里的 points_xyz 需要是 float32 类型
            data = points_xyz[:, :2].astype(np.float32)
            [vx, vy, x0, y0] = cv2.fitLine(data, cv2.DIST_L2, 0, 0.01, 0.01)
            
            # 2. 几何投影：将原始散点投影到拟合出的直线上
            # 公式：P_new = P_anchor + <P_orig - P_anchor, V_dir> * V_dir
            refined_points = []
            vx, vy, x0, y0 = float(vx), float(vy), float(x0), float(y0)
            
            for x, y in data:
                # 计算向量 (x-x0, y-y0) 在方向向量 (vx, vy) 上的投影长度
                dot_product = (x - x0) * vx + (y - y0) * vy
                # 得到直线上的新点
                new_x = x0 + dot_product * vx
                new_y = y0 + dot_product * vy
                refined_points.append([new_x, new_y])
                
            return np.array(refined_points)
        except Exception as e:
            print(f"直线拟合失败: {e}")
            return points_xyz
        
    def check_and_refine_by_angle(self, points_xyz):
        """
        基于观测几何角度决定是否执行拟合
        """
        if len(points_xyz) < 15:
            return points_xyz

        try:
            # 1. 粗略拟合，获取墙的方向向量 (vx, vy)
            data = points_xyz[:, :2].astype(np.float32)
            [vx, vy, x0, y0] = cv2.fitLine(data, cv2.DIST_L2, 0, 0.01, 0.01)
            wall_vec = np.array([vx[0], vy[0]])

            # 2. 计算观测向量 (从原点到点云中心)
            center_x = np.mean(data[:, 0])
            center_y = np.mean(data[:, 1])
            view_vec = np.array([center_x, center_y])
            
            # 归一化向量
            wall_vec /= np.linalg.norm(wall_vec)
            view_vec /= np.linalg.norm(view_vec)

            # 3. 计算夹角余弦值 (取绝对值，因为方向向量是双向的)
            cos_theta = abs(np.dot(wall_vec, view_vec))
            
            # 4. 判定逻辑
            # cos_theta > 0.866 意味着夹角小于 30 度 (属于掠射角/小夹角)
            if cos_theta > 0.85:
                # print(f"📐 检测到小夹角 ({cos_theta:.2f})，启动强力 RANSAC 纠偏")
                return self.refine_wall_points(points_xyz) 
            else:
                # print(f"平视视角，数据置信度高，仅做轻微平滑")
                return points_xyz

        except Exception as e:
            return points_xyz
        
    def generate_and_publish_pointcloud(self, pseudo_pixels, stamp):
        """
        将像素点转换为 3D 点云并发布
        """
        if not pseudo_pixels:
            return

        pc_points = []
        for u, v in pseudo_pixels:
            # 复用你现有的投影逻辑 (像素 -> 机器人基座坐标系)
            # 假设你的 pixel_to_base 返回 (x, y)，对于地面点云，z 通常设为 0 或障碍物高度
            res = self.pixel_to_base(u, v)
            if res:
                x, y = res
                # 这里可以添加简单的过滤逻辑，比如距离太远的点不放入点云
                dist = math.hypot(x, y)
                if self.range_min <= dist <= self.range_max:
                    # 构造点云数据 [x, y, z]
                    # 注意：如果是地面边缘，z 通常接近 0
                    pc_points.append([float(x), float(y), 0.0])

        if pc_points:
            # 将点云发布出去
            # 如果你使用 Zenoh，可以将其序列化为 JSON 或二进制数组
            pc_data = {
                "timestamp": stamp,
                "frame_id": "base_link",
                "points": pc_points  # 格式为 [[x1,y1,z1], [x2,y2,z2], ...]
            }
            payload = json.dumps(pc_data).encode("utf-8")
            self.pointcloud_pub.put(payload=payload,
                            encoding="application/json")

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