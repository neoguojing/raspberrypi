import torch
import numpy as np
import cv2
import os
from collections import deque
import time
from transformers import SegformerImageProcessor, SegformerForSemanticSegmentation

class SegFormerDetector:
    def __init__(
        self,
        model_name="nvidia/segformer-b2-finetuned-ade-512-512",
        device=None,
        alpha=0.6,          # 时间域平滑系数，越大越平滑(延迟越高)
        conf_threshold=0.3  # 置信度阈值，低于此值不认为是地面
    ):
        self.device = device if device else ("cuda" if torch.cuda.is_available() else "cpu")
        
        # 1. 初始化模型与处理器
        self.processor = SegformerImageProcessor.from_pretrained(model_name)
        self.model = SegformerForSemanticSegmentation.from_pretrained(model_name).to(self.device)
        
        if self.device == "cuda":
            self.model.half()
        self.model.eval()

        # ADE20K 地面类别定义
        self.ground_classes = [3, 6, 11, 13, 28, 52, 91, 94, 21, 9, 60, 46, 52, 54, 26, 109, 27, 147, 131]
        
        # 稳定性增强相关参数
        self.alpha = alpha
        self.conf_threshold = conf_threshold
        self.ema_probs = None  # 存储概率图的指数移动平均
        self.clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))
        
        self.frame_counter = 0
        self.saved_images_count = 0
        self.last_save_time = 0

    def _preprocess_lighting(self, frame):
        """使用 CLAHE 增强对比度，抑制强光和暗阴影的影响"""
        lab = cv2.cvtColor(frame, cv2.COLOR_BGR2LAB)
        l, a, b = cv2.split(lab)
        cl = self.clahe.apply(l)
        limg = cv2.merge((cl, a, b))
        return cv2.cvtColor(limg, cv2.COLOR_LAB2RGB)

    def _inference(self, frame):
        """核心推理逻辑：包含概率平滑和置信度过滤"""
        # 1. 预处理
        img_rgb = self._preprocess_lighting(frame)
        inputs = self.processor(images=img_rgb, return_tensors="pt").to(self.device)
        
        if self.device == "cuda":
            inputs = {k: v.to(dtype=torch.float16) for k, v in inputs.items()}

        with torch.no_grad():
            outputs = self.model(**inputs)
            # 获取每个类别的概率 (Softmax)
            probs = torch.nn.functional.softmax(outputs.logits, dim=1)

        # 2. 上采样到原图尺寸 (B, C, H, W)
        upsampled_probs = torch.nn.functional.interpolate(
            probs, size=frame.shape[:2], mode='bilinear', align_corners=False
        )[0] # 取出第一张图
        
        current_probs = upsampled_probs.cpu().numpy()

        # 3. 时间域平滑 (EMA) - 在概率层面上平滑比在 Label 层面更稳
        if self.ema_probs is None:
            self.ema_probs = current_probs
        else:
            self.ema_probs = self.alpha * self.ema_probs + (1 - self.alpha) * current_probs

        # 4. 获取当前最高概率及其索引
        max_conf = np.max(self.ema_probs, axis=0)
        pred_map = np.argmax(self.ema_probs, axis=0)

        # 5. 置信度过滤：如果模型对自己没信心，就判定为非地面
        # 且 只有在 ground_classes 中的才判定为 1
        ground_mask = np.isin(pred_map, self.ground_classes)
        ground_mask = np.where((ground_mask) & (max_conf > self.conf_threshold), 1, 0).astype(np.uint8)

        # 6. 形态学后处理：去除细小噪点，填充阴影空洞
        kernel = np.ones((5, 5), np.uint8)
        ground_mask = cv2.morphologyEx(ground_mask, cv2.MORPH_CLOSE, kernel) # 填洞
        ground_mask = cv2.morphologyEx(ground_mask, cv2.MORPH_OPEN, kernel)  # 去噪

        return ground_mask

    def _extract_boundary_points_optimized(self, ground_mask, step_x=10):
        """高效提取地面边缘点"""
        h, w = ground_mask.shape
        sampled_mask = ground_mask[:, ::step_x]
        
        # 寻找 1 -> 0 的跳变 (从下往上扫描的逻辑简化版)
        diff = sampled_mask[:-1, :] - sampled_mask[1:, :]
        y_coords, x_idx = np.where(diff == 1)
        
        contact_dict = {}
        for x, y in zip(x_idx, y_coords):
            real_x = x * step_x
            # 保留每列最靠下的边界点
            if real_x not in contact_dict or y > contact_dict[real_x]:
                contact_dict[real_x] = y

        contact_pixels = []
        for i, x in enumerate(range(0, w, step_x)):
            if x in contact_dict:
                contact_pixels.append((float(x), float(contact_dict[x])))
            else:
                # 底部是地面则看作远处(0)，否则看作脚下(h-1)
                y_val = 0.0 if sampled_mask[-1, i] == 1 else float(h - 1)
                contact_pixels.append((float(x), y_val))

        return contact_pixels

    def get_ground_contact_points(self, frame, render=True):
        self.frame_counter += 1
        
        # 执行带有稳定性优化的推理
        smoothed_mask = self._inference(frame)
        
        # 提取交界点
        contact_pixels = self._extract_boundary_points_optimized(smoothed_mask)

        annotated_frame = None
        if render:
            annotated_frame = self._render_visualization(frame, smoothed_mask, contact_pixels)

        return contact_pixels, annotated_frame

    def _render_visualization(self, frame, ground_mask, contact_pixels):
        overlay = frame.copy()
        overlay[ground_mask == 1] = [0, 255, 0] # 绿色高亮地面
        canvas = cv2.addWeighted(overlay, 0.3, frame, 0.7, 0)

        for pt in contact_pixels:
            cv2.circle(canvas, (int(pt[0]), int(pt[1])), 4, (0, 0, 255), -1)
        return canvas

    def save_sample_image(self, visual_frame, folder="samples", max_count=10, interval=5):
        if visual_frame is None: return
        
        curr_time = time.time()
        if curr_time - self.last_save_time >= interval:
            self.last_save_time = curr_time
            save_index = (self.saved_images_count % max_count) + 1
            self.saved_images_count += 1
            
            os.makedirs(folder, exist_ok=True)
            path = os.path.join(folder, f"sample_{save_index}.jpg")
            cv2.imwrite(path, visual_frame)
            print(f"📸 稳定采样保存: {path}")

# =========================================================
# 运行主逻辑 (模拟视频流)
# =========================================================
def main():
    # 调高 alpha 可以让预测更迟钝但更稳（适合光照剧烈变化场景）
    detector = SegFormerDetector(alpha=0.7, conf_threshold=0.4)
    
    # 模拟处理：这里假设你有一个视频文件或摄像头
    # cap = cv2.VideoCapture("video.mp4")
    test_image_path = "asset/test.png"
    frame = cv2.imread(test_image_path)

    if frame is not None:
        # 在实际应用中，循环调用 get_ground_contact_points
        for i in range(5): # 模拟多帧输入以触发 EMA 平滑
            contact_pixels, visual_frame = detector.get_ground_contact_points(frame, render=True)
            detector.save_sample_image(visual_frame, interval=0) # 强制保存测试
            
        print(f"🔹 检测完成，当前帧接触点数量: {len(contact_pixels)}")

if __name__ == "__main__":
    main()