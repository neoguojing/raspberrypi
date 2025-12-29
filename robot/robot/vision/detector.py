from ultralytics import YOLO
import numpy as np
import os
import cv2
class SegDetector:
    def __init__(
        self,
        model_name="yolo11n-seg.pt",
        model_dir="./models",
        conf=0.45
    ):
        self.conf = conf

        os.makedirs(model_dir, exist_ok=True)
        self.model_path = os.path.join(model_dir, model_name)

        # 1. 本地不存在 → 触发自动下载
        if not os.path.exists(self.model_path):
            print(f"[SegDetector] Model not found, downloading: {model_name}")
            self.model = YOLO(model_name)        # 触发 Ultralytics 下载
            self.model.save(self.model_path)     # 保存到指定目录
        else:
            print(f"[SegDetector] Loading local model: {self.model_path}")
            self.model = YOLO(self.model_path)

        # 2. 避障关心的 COCO 类别
        self.obstacle_ids = [
            0, 1, 2, 3, 5, 7, 24, 26, 32, 39, 41, 64, 67
        ]

    def get_ground_contact_points(self, frame, render=True):
        results = self.model(frame, verbose=False, conf=self.conf)[0]
        contact_pixels = []

        if results.masks is None:
            return contact_pixels

        # 获取所有类别的索引和掩码
        # 一次性获取所有 cls 以减少循环内计算
        classes = results.boxes.cls.cpu().numpy().astype(int)
        
        for i, mask in enumerate(results.masks.xy):
            # 1. 类别过滤
            if classes[i] not in self.obstacle_ids:
                continue

            # 2. 几何完整性过滤
            if mask.shape[0] < 20: # 稍微放宽，防止过滤掉远处的小障碍物
                continue

            y_min, y_max = np.min(mask[:, 1]), np.max(mask[:, 1])
            h = y_max - y_min
            
            # 忽略过扁的异常 Mask (可能是地面线)
            if h < 8:
                continue

            # 3. 提取底部带状区域
            # 0.15h 保证了采样鲁棒性，max(5, ...) 保证了小目标的采样厚度
            band_height = max(5, int(0.15 * h))
            mask_bottom_indices = mask[:, 1] > (y_max - band_height)
            bottom_points = mask[mask_bottom_indices]

            if len(bottom_points) < 3:
                continue

            # 4. 精准三点采样：根据 x 轴排序
            # 排序是为了找到物体的左边界和右边界
            sorted_indices = np.argsort(bottom_points[:, 0])
            left_idx = sorted_indices[0]
            right_idx = sorted_indices[-1]
            mid_idx = sorted_indices[len(sorted_indices) // 2]

            # 采样点：左边缘、中间点、右边缘
            # 保持为 numpy 数组或简单列表，方便后续 pixel_to_base 调用
            contact_pixels.append(bottom_points[left_idx])
            contact_pixels.append(bottom_points[mid_idx])
            contact_pixels.append(bottom_points[right_idx])
        
        annotated_frame = None
        if render:
            # 1. 先让 YOLO 帮你画好基础的 Mask 和 框
            # labels=True 显示类别, boxes=True 显示方框
            annotated_frame = results.plot(labels=True, boxes=True)

            # 2. 在 YOLO 画好的图上，叠加你自己的三个采样点
            # 假设你已经通过之前的逻辑算出了 contact_pixels
            for pt in contact_pixels:
                cv2.circle(annotated_frame, (int(pt[0]), int(pt[1])), 5, (0, 0, 255), -1)

        return (contact_pixels , annotated_frame)
