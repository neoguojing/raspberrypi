import os
import time

import cv2
import numpy as np

from robot.robot.vision.trt_engine import TRTEngine

class SegFormerTRTDetector:
    def __init__(self, engine_path="models/segformer_b2_torch2.9.0_cu126_opset18.engine", alpha=0.6, conf_threshold=0.25, debug=False):
        self.id2label = {}
        self.ground_classes = [3, 6, 11, 13, 21, 26, 27, 28, 46, 52, 54, 60, 91, 94, 109, 131, 147]
        self.alpha = alpha
        self.conf_threshold = conf_threshold
        self.ema_ground_prob = None
        self.debug = debug
        self.trt_engine = TRTEngine(engine_path)

        self.kernel = np.ones((3, 3), np.uint8)

    def print_detected_categories(self, pred_map):
        unique_ids = np.unique(pred_map)
        print("\n🔍 当前帧检测到以下类型:")
        print("-" * 30)
        for cls_id in unique_ids:
            label = self.id2label.get(int(cls_id), f"Unknown({cls_id})")
            pixel_count = np.sum(pred_map == cls_id)
            percentage = (pixel_count / pred_map.size) * 100
            is_ground = " [地面✅]" if int(cls_id) in self.ground_classes else ""
            print(f"ID {cls_id:3} | {label:15} | 占比: {percentage:5.2f}% {is_ground}")

    # 输入统一为RGB
    def _predict_probs(self, frame_rgb):
        H, W = frame_rgb.shape[:2]
        TARGET_H, TARGET_W = 1232, 1640

        # 1. Resize only if needed
        if H != TARGET_H or W != TARGET_W:
            frame_processed = cv2.resize(frame_rgb, (TARGET_W, TARGET_H), interpolation=cv2.INTER_LINEAR)
        else:
            frame_processed = frame_rgb  # avoid copy if possible

        # 2. Convert to float32 [0, 255]
        if frame_processed.dtype != np.float32:
            frame_float = frame_processed.astype(np.float32)
        else:
            frame_float = frame_processed

        # 3. Add batch dim only if input is 3D
        if frame_float.ndim == 3:
            frame_input = np.expand_dims(frame_float, axis=0)  # (1, H, W, C)
        elif frame_float.ndim == 4:
            frame_input = frame_float
        else:
            raise ValueError(f"Unsupported input ndim: {frame_float.ndim}")

        # 4. Safety check (optional but recommended during debug)
        expected_shape = (1, TARGET_H, TARGET_W, 3)
        if frame_input.shape != expected_shape:
            raise ValueError(f"Input shape {frame_input.shape} does not match expected {expected_shape}")

        # 1. 零前处理推理 (6-8ms)
        # 内部已经包含了所有的 Resize, Norm, Argmax 和插值放大
        t0 = time.perf_counter()
        logits_small = self.trt_engine.infer(frame_input)
        if self.debug:
            print(f"🔍 [Debug] 输入推理的图像尺寸: {frame_input.shape} dtype: {frame_input.dtype}")
            print(f"⏱️ 推理耗时: {(time.perf_counter() - t0) * 1000:.2f} ms")

        return logits_small

    def _extract_boundary_points(self, ground_mask, step_x=8):
        """
        从图像底部向上扫描，寻找地面（1）到障碍物（0）的第一个转换点。
        """
        h, w = ground_mask.shape
        # 按步长采样列
        sampled = ground_mask[:, ::step_x]
        
        # 获取采样后的宽度
        sampled_w = sampled.shape[1]
        
        # 预设所有点都在顶部（表示整列都是地面，无障碍）
        # 或者预设在底部（表示整列都是障碍），取决于你的业务逻辑
        # 这里预设为 0，如果没有发现障碍，则认为路径延伸到无穷远
        boundary_y = np.zeros(sampled_w)

        # 核心逻辑：
        # 我们寻找从下往上第一个 1 -> 0 的跳变
        # 也就是在原矩阵中，上方是 0，下方是 1 的位置
        # diff = sampled[上方] - sampled[下方]
        diff = sampled[:-1, :].astype(np.int16) - sampled[1:, :].astype(np.int16)
        
        # diff == -1 表示：上面是 0 (障碍), 下面是 1 (地面) -> 这正是接触线！
        ys, xs = np.where(diff == -1)

        # 我们需要每一列中最靠下（Y值最大）的跳变点
        # 先初始化一个较小值
        res_y = np.full(sampled_w, 0) 
        
        # 因为 np.where 是按行优先扫描的（从上往下），
        # 所以对于每一列，最后一次更新的 y 必然是该列最靠下的交界点
        for y, x in zip(ys, xs):
            res_y[x] = y + 1 # +1 是为了指向地面的上边缘

        # 处理特殊情况：如果某一列完全没有跳变
        # 情况 A: 全是地面 -> res_y[x] 保持为 0
        # 情况 B: 全是障碍 -> 我们需要检测底部第一个像素是不是 0
        bottom_row = sampled[-1, :]
        for x in range(sampled_w):
            if bottom_row[x] == 0: # 底部就是障碍物
                res_y[x] = h - 1

        contact_pixels = [
            (float(x * step_x), float(res_y[x]))
            for x in range(sampled_w)
        ]
        return contact_pixels
    
    def _inference(self, frame):
        h_orig, w_orig = frame.shape[:2]
        
        frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        logits_small = self._predict_probs(frame_rgb)  # TRT输出尺寸（如512x512或1232x1640）
        
        if logits_small.ndim == 4:
            if logits_small.shape[1] > 100: # 假设 Channel 在第二维 (NCHW)
                logits_small = logits_small[0] # (C, H, W)
            elif logits_small.shape[-1] > 100: # 假设 Channel 在最后一维 (NHWC) - 较少见用于分割头
                logits_small = np.transpose(logits_small[0], (2, 0, 1)) # (H, W, C) -> (C, H, W)
            else:
                raise ValueError(f"Unexpected logits shape: {logits_small.shape}")
        else:
            raise ValueError(f"Logits must be 4D, got {logits_small.ndim}D")

        num_classes, current_h, current_w = logits_small.shape

        # 🔍 调试：打印尺寸
        if self.debug:
            print(f"🔍 logits_small shape: {logits_small.shape}, dtype: {logits_small.dtype}")
    
        # 3. 提取地面相关类别的最大概率值
        # 结果是一个 [H, W] 的矩阵，每个像素值代表“该点属于地面”的信心得分
        ground_prob = logits_small[self.ground_classes].max(axis=0)

        # 4. 指数移动平均 (EMA) 平滑处理
        # 作用：过滤掉由于单帧噪点、动态模糊或快速阴影漂移引起的“检测空洞”
        if self.ema_ground_prob is None:
            self.ema_ground_prob = ground_prob
        else:
            # 这里的平滑发生在概率空间，比在 Mask (0/1) 空间平滑更加细腻
            self.ema_ground_prob = (
                self.alpha * self.ema_ground_prob
                + (1 - self.alpha) * ground_prob
            )

        # ✅ 新增：对平滑后的概率图进行高斯模糊（空间平滑）
        # 目的：消除局部噪声引起的概率抖动，防止虚假边界
        blurred_ground_prob = cv2.GaussianBlur(
            self.ema_ground_prob.astype(np.float32),
            ksize=(3, 3),      # 核大小，可调（奇数）
            sigmaX=0.8         # X方向标准差，控制模糊强度
        )
    
        # 5. 二值化：只有当平滑后的地面概率超过阈值时，才判定为地面
        ground_mask = (blurred_ground_prob > self.conf_threshold).astype(np.uint8)

        # 6. 形态学闭运算 (Closing)
        # 作用：填充地面掩码中细小的黑色空洞（如地砖缝隙、细小阴影），同时保持边缘位置准确
        kernel = np.ones((5, 5), np.uint8)
        ground_mask = cv2.morphologyEx(ground_mask, cv2.MORPH_CLOSE, kernel)

        return ground_mask,(h_orig, w_orig), (current_h, current_w)
    
    def render(self, frame, mask, points):
        vis = frame.copy()
        vis[mask == 1] = [0, 255, 0]
        vis = cv2.addWeighted(vis, 0.3, frame, 0.7, 0)
        for x, y in points:
            cv2.circle(vis, (int(x), int(y)), 4, (0, 0, 255), -1)
        return vis

    def get_ground_contact_points(self, frame, render=False):
        t0 = time.perf_counter()
        mask, (h_orig, w_orig), (h_low, w_low)  = self._inference(frame)
        # 计算缩放比例
        # scale_x = w_orig / w_low
        # scale_y = h_orig / h_low
        if self.debug:
            print(f"[Timing] Inference: {(time.perf_counter() - t0) * 1000:.2f} ms")
        t1 = time.perf_counter()

        mask_full = cv2.resize(mask, (w_orig, h_orig), interpolation=cv2.INTER_NEAREST)

        points_orig = self._extract_boundary_points(mask_full)
        # 将坐标缩放回原图
        # points_orig = [(x * scale_x, y * scale_y) for x, y in points]
        if self.debug:
            print(f"[Timing] Boundary extraction: {(time.perf_counter() - t1) * 1000:.2f} ms")
        t2 = time.perf_counter()
        if render:
            vis = self.save_sample_image(frame, mask_full, points_orig)
        if self.debug:
            print(f"[Timing] Rendering: {(time.perf_counter() - t2) * 1000:.2f} ms")
        return points_orig, vis

    def save_sample_image(self, frame, mask, points, folder="samples", max_count=10, interval_seconds=10):
        if not hasattr(self, "last_save_time"):
            self.last_save_time = 0
        if not hasattr(self, "saved_images_count"):
            self.saved_images_count = 0
        current_time = time.time()
        if current_time - self.last_save_time >= interval_seconds:
            frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
            visual_frame = self.render(frame, mask, points)
            self.last_save_time = current_time
            save_index = (self.saved_images_count % max_count) + 1
            self.saved_images_count += 1
            os.makedirs(folder, exist_ok=True)
            file_path = os.path.join(folder, f"sample_{save_index}.jpg")
            cv2.imwrite(file_path, visual_frame)
            print(f"📸 [采样成功] 已保存至: {file_path} (累计保存: {self.saved_images_count} 张)")
            return visual_frame
        return None

if __name__ == "__main__":
    # --- 1. 配置参数 ---
    ENGINE_PATH = "models/segformer_b2_torch2.9.0_cu126_opset18.engine"  # 你的引擎文件路径
    IMAGE_PATH = "asset/test.png"              # 你要验证的测试图路径
    
    # 填充部分 ADE20K 标签，用于验证打印结果
    ADE_LABELS = {
        3: "floor", 6: "road", 11: "sidewalk", 13: "earth", 21: "mountain",
        26: "house", 46: "sand", 52: "path", 54: "field", 94: "case"
    }

    # --- 2. 初始化检测器 ---
    if not os.path.exists(ENGINE_PATH):
        print(f"❌ 找不到引擎文件: {ENGINE_PATH}")
    elif not os.path.exists(IMAGE_PATH):
        print(f"❌ 找不到测试图片: {IMAGE_PATH}")
    else:
        # 初始化（单图验证时 alpha 设为 0，因为不需要时间平滑）
        detector = SegFormerTRTDetector(alpha=0.0, conf_threshold=0.0, debug=True)
        detector.id2label = ADE_LABELS

        # --- 3. 执行验证 ---
        frame = cv2.imread(IMAGE_PATH)
        print(f"📷 正在处理图片: {IMAGE_PATH} ({frame.shape[1]}x{frame.shape[0]})")
        
        start_time = time.time()
        # 获取点位和可视化结果
        points, vis = detector.get_ground_contact_points(frame, render=True)
        end_time = time.time()

        print(f"⏱️ 推理总耗时: {(end_time - start_time)*1000:.2f} ms")
        
        # --- 4. 显示与保存 ---
        if vis is not None:      
            # 同时保存一份到本地
            save_path = "debug_result.jpg"
            cv2.imwrite(save_path, vis)
            print(f"💾 结果已保存至: {save_path}")