import os
import time

import cv2
import numpy as np

from robot.robot.vision.trt_engine import TRTEngine

class SegFormerTRTDetector:
    def __init__(self, engine_path="models/segformer_b2_torch2.9.0_cu126_opset18.engine", alpha=0.7, conf_threshold=0.5, debug=False):
        self.id2label = {}
        self.ground_classes = [3, 6, 11, 13, 21, 26, 27, 28, 46, 52, 54, 60, 91, 94, 109, 131, 147]
        self.alpha = alpha
        self.conf_threshold = conf_threshold
        self.ema_ground_prob = None
        self.debug = debug
        self.trt_engine = TRTEngine(engine_path)

        self.kernel = np.ones((5, 5), np.uint8)

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
        pred_map = self.trt_engine.infer(frame_input)
        if self.debug:
            print(f"🔍 [Debug] 输入推理的图像尺寸: {frame_input.shape} dtype: {frame_input.dtype}")
            print(f"⏱️ 推理耗时: {(time.perf_counter() - t0) * 1000:.2f} ms")

        return pred_map

    def _extract_boundary_points(self, ground_mask, step_x=1):
        h, w = ground_mask.shape
        sampled = ground_mask[:, ::step_x]
        sampled_w = sampled.shape[1]
        
        # 初始化为底部
        res_y = np.full(sampled_w, h - 1, dtype=np.float32)
        
        # 情况 A: 寻找跳变点 (0 -> 1)
        diff = sampled[:-1, :].astype(np.int16) - sampled[1:, :].astype(np.int16)
        ys, xs = np.where(diff == -1)  # 从地面(1)跳变到非地面(0)
        
        for y, x in zip(ys, xs):
            # ✅ 关键修正：边界点应在地面结束行的下一行 (y+1)
            if y + 1 < res_y[x]:
                res_y[x] = y + 1
        
        # 处理“全列都是地面”的情况：没有跳变点，说明地面一直延伸到顶部
        # 此时应设为一个合理的地平线高度，比如 h * 0.3
        for x in range(sampled_w):
            if res_y[x] == h - 1:  # 未找到跳变点
                # 检查是否整列都是地面
                if np.all(sampled[:, x] == 1):
                    res_y[x] = max(0, int(h * 0.3))  # 地平线假设在 30% 高度
                # 否则保留为 h-1（底部）
        
        return [(float(x * step_x), float(res_y[x])) for x in range(sampled_w)]

    def _inference(self, frame):
        h_orig, w_orig = frame.shape[:2]
        
        frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        pred_map_small = self._predict_probs(frame_rgb)  # TRT输出尺寸（如512x512或1232x1640）
    
        # 🔍 调试：打印尺寸
        if self.debug:
            print(f"🔍 pred_map_small shape: {pred_map_small.shape}, dtype: {pred_map_small.dtype}")
            print(f"🔍 唯一类别: {np.unique(pred_map_small)}")
        
        # 🔧 关键修复：如果存在 Batch 维度 (ndim==3)，去掉它
        if pred_map_small.ndim == 3 and pred_map_small.shape[0] == 1:
            pred_map_small = pred_map_small[0]  # 变为 (1232, 1640)
        
        # 再次确认形状
        current_h, current_w = pred_map_small.shape
        
        # 1. Resize 到原图尺寸（如果需要）
        if current_h != h_orig or current_w != w_orig:
            # cv2.resize 输入必须是 (H, W)，目标尺寸是 (width, height)
            # pred_map = cv2.resize(pred_map_small, (w_orig, h_orig), interpolation=cv2.INTER_NEAREST)
            pred_map = pred_map_small
        else:
            pred_map = pred_map_small
        
        # 2. 打印检测到的类别（调试）
        if self.debug:
            self.print_detected_categories(pred_map)
        
        # 3. 根据 ground_classes 生成地面 mask
        # 确保 ground_mask 也是 2D
        ground_mask = np.zeros_like(pred_map, dtype=np.uint8)
        for cls_id in self.ground_classes:
            ground_mask[pred_map == cls_id] = 1
        
        # 4. 闭运算处理
        ground_mask = cv2.morphologyEx(ground_mask, cv2.MORPH_CLOSE, self.kernel)
        
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
        scale_x = w_orig / w_low
        scale_y = h_orig / h_low
        if self.debug:
            print(f"[Timing] Inference: {(time.perf_counter() - t0) * 1000:.2f} ms")
        t1 = time.perf_counter()
        points = self._extract_boundary_points(mask)
        # 将坐标缩放回原图
        points_orig = [(x * scale_x, y * scale_y) for x, y in points]
        if self.debug:
            print(f"[Timing] Boundary extraction: {(time.perf_counter() - t1) * 1000:.2f} ms")
        t2 = time.perf_counter()
        if render:
            mask_full = cv2.resize(mask, (w_orig, h_orig), interpolation=cv2.INTER_NEAREST)
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