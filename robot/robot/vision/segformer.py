import torch
import numpy as np
import cv2
import os
from collections import deque
from transformers import SegformerImageProcessor, SegformerForSemanticSegmentation

class SegFormerDetector:
    def __init__(
        self,
        model_name="nvidia/segformer-b2-finetuned-ade-512-512",
        device=None
    ):
        self.device = device if device else ("cuda" if torch.cuda.is_available() else "cpu")
        
        # 1. 初始化模型与处理器
        self.processor = SegformerImageProcessor.from_pretrained(model_name)
        self.model = SegformerForSemanticSegmentation.from_pretrained(model_name).to(self.device)
        
        # [优化] 开启半精度推理，RTX 3090 性能翻倍
        if self.device == "cuda":
            self.model.half()
        self.model.eval()
        print(f"model classes: {self.get_labels()}")
        # ADE20K 地面类别定义
        # 核心通行类：地板、马路、人行道、小径、土地、地毯、土地
        self.ground_classes = [3, 6, 11, 13, 28, 52, 91, 94]
        # 新增：用于时域平滑的队列，存储最近 3 帧的 ground_mask
        self.mask_buffer = deque(maxlen=3)
        # 统计相关
        self.frame_counter = 0
        self.saved_images_count = 0

    def get_ground_contact_points(self, frame, render=True):
        self.frame_counter += 1
        
        # 1. 模型推理获取当前帧原始掩码 (Raw Mask)
        current_mask = self._inference(frame)

        # 2. [新增] Temporal Smoothing: 3 帧中值滤波
        self.mask_buffer.append(current_mask)
        
        if len(self.mask_buffer) < 3:
            # 缓冲区未满时，直接使用当前帧
            smoothed_mask = current_mask
        else:
            # 将队列中的 3 个 mask 堆叠并取中值
            # 对于二值(0,1)掩码，中值等同于“投票制”：2帧以上认为是地面，结果就是地面
            mask_stack = np.stack(self.mask_buffer, axis=0)
            smoothed_mask = np.median(mask_stack, axis=0).astype(np.uint8)

        # 3. 使用平滑后的掩码提取交界点
        contact_pixels = self._extract_boundary_points(smoothed_mask)

        # 4. 渲染
        annotated_frame = None
        if render:
            # 使用平滑后的结果进行可视化
            annotated_frame = self._render_visualization(frame, smoothed_mask, contact_pixels)
            self.save_sample_image(annotated_frame)

        return contact_pixels, annotated_frame

    def _inference(self, frame):
        """[内部函数] 处理模型推理"""
        # 颜色空间转换
        img_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)        
        # 预处理并转为半精度
        inputs = self.processor(images=img_rgb, return_tensors="pt").to(self.device)
        if self.device == "cuda":
            inputs = {k: v.to(dtype=torch.float16) for k, v in inputs.items()}

        with torch.no_grad():
            outputs = self.model(**inputs)
            logits = outputs.logits

        # 上采样回原图尺寸
        upsampled_logits = torch.nn.functional.interpolate(
            logits, size=frame.shape[:2], mode='bilinear', align_corners=False
        )
        
        # 获取分类预测图
        pred_map = upsampled_logits.argmax(dim=1)[0].cpu().numpy()
        # self.print_detected_categories(pred_map)
        # 生成二值化的地面掩码
        return np.isin(pred_map, self.ground_classes).astype(np.uint8)

    def print_detected_categories(self, pred_map):
        """
        输入推理得到的 pred_map [H, W]
        打印当前画面中出现的所有类别名称
        """
        # 1. 获取图中存在的所有唯一 ID
        unique_ids = np.unique(pred_map)
        
        # 2. 获取映射表
        id2label = self.model.config.id2label
        
        print("\n🔍 当前帧检测到以下类型:")
        print("-" * 30)
        for cls_id in unique_ids:
            label = id2label.get(cls_id, f"Unknown({cls_id})")
            # 统计该类别的像素占比，判断是否为主要特征
            pixel_count = np.sum(pred_map == cls_id)
            percentage = (pixel_count / pred_map.size) * 100
            
            # 标注该类别是否被你归类为“地面”
            is_ground = " [地面✅]" if cls_id in self.ground_classes else ""
            
            print(f"ID {cls_id:3} | {label:15} | 占比: {percentage:5.2f}% {is_ground}")
              
    def get_labels(self):
        """返回所有类别的字典 {id: "label_name"}"""
        return self.model.config.id2label
    
    def _extract_boundary_points(self, ground_mask):
        contact_pixels = []
        h, w = ground_mask.shape
        step_x = 10

        for x in range(0, w, step_x):
            col = ground_mask[:, x]

            found = False
            # 从底部向上扫描
            for y in range(h - 1, 0, -1):
                # 地面 → 非地面 的边界
                if col[y] == 1 and col[y - 1] == 0:
                    contact_pixels.append((float(x), float(y)))
                    found = True
                    break

            if not found:
                # 整列没有障碍，认为视野开阔
                contact_pixels.append((float(x), float(0)))

        return contact_pixels

    def _render_visualization(self, frame, ground_mask, contact_pixels):
        """[内部函数] 绘制可视化效果"""
        overlay = frame.copy()
        # 绿色高亮地面
        overlay[ground_mask == 1] = [0, 255, 0]
        canvas = cv2.addWeighted(overlay, 0.3, frame, 0.7, 0)

        # 绘制红色的交界点
        for pt in contact_pixels:
            cv2.circle(canvas, (int(pt[0]), int(pt[1])), 4, (0, 0, 255), -1)
            
        return canvas

    def save_sample_image(self, image, folder="samples", max_count=10, interval=20):
        """
        [独立函数] 外部调用此函数来决定是否保存采样图片
        """
        if self.saved_images_count >= max_count:
            return False
            
        if self.frame_counter % interval == 0:
            self.saved_images_count += 1
            os.makedirs(folder, exist_ok=True)
            path = os.path.join(folder, f"sample_{self.saved_images_count}.jpg")
            cv2.imwrite(path, image)
            print(f"📸 采样保存成功: {path} (Frame: {self.frame_counter})")
            return True
        return False

# =========================================================
# 运行主逻辑
# =========================================================
def main():
    detector = SegFormerDetector()
    
    test_image_path = "asset/test.png"
    frame = cv2.imread(test_image_path)
    
    if frame is not None:
        # 1. 推理与点提取
        contact_pixels, visual_frame = detector.get_ground_contact_points(frame, render=True)
        
        # 2. 独立调用保存逻辑
        detector.save_sample_image(visual_frame, max_count=5, interval=1)
        
        print(f"🔹 检测到 {len(contact_pixels)} 个接触点")
        # cv2.imshow("Optimized SegFormer", visual_frame)
        # cv2.waitKey(0)

if __name__ == "__main__":
    main()