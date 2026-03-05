import numpy as np
import cv2
import os
import time
from robot.robot.vision.trt_engine import TRTEngine

class SegFormerTRTDetector:
    def __init__(self, engine_path="models/segformer_b2.engine", alpha=0.7, conf_threshold=0.25):
        self.id2label = {}
        self.ground_classes = [3, 6, 11, 13, 21, 26, 27, 28, 46, 52, 54, 60, 91, 94, 109, 131, 147]
        self.alpha = alpha
        self.conf_threshold = conf_threshold
        self.ema_ground_prob = None
        self.trt_engine = TRTEngine(engine_path)

        # 提前准备好 LUT，避免在推理循环中重复创建
        self.lut = np.zeros(256, dtype=np.uint8)
        self.lut[self.ground_classes] = 255  # 地面类设为 255 (白色)

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
        # frame_rgb: 已经是 RGB 格式的 (1232, 1640, 3) uint8 数组
    
        # 1. 零前处理推理 (6-8ms)
        # 内部已经包含了所有的 Resize, Norm, Argmax 和插值放大
        pred_map = self.trt_engine.infer(frame_rgb)[0]
        # 3. 后处理：仅剩查表 (约 1ms)
        # 此时 pred_map 已经是 (1232, 1640) 尺寸了
        ground_mask = self.lut[pred_map.astype(np.uint8)]
    
        return ground_mask, pred_map

    def _extract_boundary_points(self, ground_mask, step_x=10):
        h, w = ground_mask.shape
        sampled = ground_mask[:, ::step_x]
        sampled_w = sampled.shape[1]
        
        # 初始化为底部
        res_y = np.full(sampled_w, h - 1, dtype=np.float32)
        
        # 情况 A: 寻找跳变点 (0 -> 1)
        diff = sampled[:-1, :].astype(np.int16) - sampled[1:, :].astype(np.int16)
        ys, xs = np.where(diff == -1)
        for y, x in zip(ys, xs):
            if y + 1 < res_y[x]:
                res_y[x] = y + 1
        
        # 情况 B: 纠正“全列皆地面”的情况
        # 如果第一行(y=0)就是地面，且该列还没找到跳变点，说明地面一直延伸到地平线
        top_row = sampled[0, :]
        for x in range(sampled_w):
            if top_row[x] == 1 and res_y[x] == h - 1:
                res_y[x] = 0  # 或者设为一个合理的地平线高度，如 h*0.3
        
        return [(float(x * step_x), float(res_y[x])) for x in range(sampled_w)]

    def _inference(self, frame):
        h_orig, w_orig = frame.shape[:2]
        
        ground_logits_small, pred_map_small = self._predict_probs(frame)
        
        # 1. 处理分类图
        pred_map = cv2.resize(pred_map_small.astype(np.uint8), (w_orig, h_orig), interpolation=cv2.INTER_NEAREST)
        self.print_detected_categories(pred_map) # 调试时开启

        # 2. 处理地面概率 (Logits)
        ground_prob = cv2.resize(ground_logits_small, (w_orig, h_orig), interpolation=cv2.INTER_LINEAR)

        # 3. EMA 
        if self.ema_ground_prob is None:
            self.ema_ground_prob = ground_prob
        else:
            self.ema_ground_prob = self.alpha * self.ema_ground_prob + (1 - self.alpha) * ground_prob

        # 4. 生成 Mask (根据 Logit 阈值，通常 SegFormer Logit 阈值需根据实验调整)
        # 如果模型输出了 Softmax，阈值用 0.25；如果是原始 Logit，阈值可能是 0 左右
        _, ground_mask = cv2.threshold(self.ema_ground_prob, self.conf_threshold, 255, cv2.THRESH_BINARY)
        
        # 形态学操作
        kernel = np.ones((5, 5), np.uint8)
        ground_mask = cv2.morphologyEx(ground_mask.astype(np.uint8), cv2.MORPH_CLOSE, kernel)
        
        # 转回 0/1 格式供后续提取点位使用
        return (ground_mask > 0).astype(np.uint8)
    
    def render(self, frame, mask, points):
        vis = frame.copy()
        vis[mask == 1] = [0, 255, 0]
        vis = cv2.addWeighted(vis, 0.3, frame, 0.7, 0)
        for x, y in points:
            cv2.circle(vis, (int(x), int(y)), 4, (0, 0, 255), -1)
        return vis

    def get_ground_contact_points(self, frame, render=False):
        t0 = time.perf_counter()
        mask = self._inference(frame)
        print(f"[Timing] Inference: {(time.perf_counter() - t0) * 1000:.2f} ms")
        t1 = time.perf_counter()
        points = self._extract_boundary_points(mask)
        print(f"[Timing] Boundary extraction: {(time.perf_counter() - t1) * 1000:.2f} ms")
        t2 = time.perf_counter()
        vis = self.save_sample_image(frame, mask, points) if render else None
        print(f"[Timing] Rendering: {(time.perf_counter() - t2) * 1000:.2f} ms")
        return points, vis

    def save_sample_image(self, frame, mask, points, folder="samples", max_count=10, interval_seconds=10):
        if not hasattr(self, "last_save_time"):
            self.last_save_time = 0
        if not hasattr(self, "saved_images_count"):
            self.saved_images_count = 0
        current_time = time.time()
        if current_time - self.last_save_time >= interval_seconds:
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
        detector = SegFormerTRTDetector(ENGINE_PATH, alpha=0.0, conf_threshold=0.5)
        detector.id2label = ADE_LABELS

        # --- 3. 执行验证 ---
        frame = cv2.imread(IMAGE_PATH)
        print(f"📷 正在处理图片: {IMAGE_PATH} ({frame.shape[1]}x{frame.shape[0]})")
        
        start_time = time.time()
        # 获取点位和可视化结果
        points, vis = detector.get_ground_contact_points(frame, render=False)
        end_time = time.time()

        print(f"⏱️ 推理总耗时: {(end_time - start_time)*1000:.2f} ms")
        
        # --- 4. 显示与保存 ---
        if vis is not None:      
            # 同时保存一份到本地
            save_path = "debug_result.jpg"
            cv2.imwrite(save_path, vis)
            print(f"💾 结果已保存至: {save_path}")