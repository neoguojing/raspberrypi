import numpy as np
import cv2
import os
import time
import torch
from transformers import SegformerImageProcessor, SegformerForSemanticSegmentation


class SegFormerTransformerDetector:
    def __init__(
        self,
        model_name="nvidia/segformer-b2-finetuned-ade-512-512",
        device=None,
        alpha=0.7,
        conf_threshold=0.25,
    ):
        self.id2label = {}
        self.ground_classes = [3, 6, 11, 13, 21, 26, 27, 28, 46, 52, 54, 60, 91, 94, 109, 131, 147]
        self.alpha = alpha
        self.conf_threshold = conf_threshold
        self.ema_ground_prob = None
        self.clahe = cv2.createCLAHE(2.0, (8, 8))

        self.device = device or ("cuda" if torch.cuda.is_available() else "cpu")
        self.processor = SegformerImageProcessor.from_pretrained(model_name)
        self.model = SegformerForSemanticSegmentation.from_pretrained(model_name).to(self.device)
        if self.device == "cuda":
            self.model.half()
        self.model.eval()
        self.id2label = self.model.config.id2label

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

    def _predict_probs(self, frame):
        inputs = self.processor(images=frame, return_tensors="pt")
        inputs = {k: v.to(self.device) for k, v in inputs.items()}
        if self.device == "cuda":
            inputs = {k: v.half() for k, v in inputs.items()}

        with torch.no_grad():
            logits = self.model(**inputs).logits
            probs = torch.softmax(logits, dim=1)

        probs = torch.nn.functional.interpolate(
            probs,
            size=frame.shape[:2],
            mode="bilinear",
            align_corners=False,
        )[0].cpu().numpy()
        return probs

    def _inference(self, frame):
        probs = self._predict_probs(frame)
        pred_map = probs.argmax(axis=0)
        self.print_detected_categories(pred_map)
        ground_prob = probs[self.ground_classes].max(axis=0)

        if self.ema_ground_prob is None:
            self.ema_ground_prob = ground_prob
        else:
            self.ema_ground_prob = self.alpha * self.ema_ground_prob + (1 - self.alpha) * ground_prob

        blurred_ground_prob = cv2.GaussianBlur(self.ema_ground_prob.astype(np.float32), (5, 5), 1.0)
        ground_mask = (blurred_ground_prob > self.conf_threshold).astype(np.uint8)
        kernel = np.ones((3, 3), np.uint8)
        return cv2.morphologyEx(ground_mask, cv2.MORPH_CLOSE, kernel)

    def _extract_boundary_points(self, ground_mask, step_x=10):
        h, _ = ground_mask.shape
        sampled = ground_mask[:, ::step_x]
        sampled_w = sampled.shape[1]
        diff = sampled[:-1, :].astype(np.int16) - sampled[1:, :].astype(np.int16)
        ys, xs = np.where(diff == -1)
        res_y = np.full(sampled_w, 0)
        for y, x in zip(ys, xs):
            res_y[x] = y + 1
        bottom_row = sampled[-1, :]
        for x in range(sampled_w):
            if bottom_row[x] == 0:
                res_y[x] = h - 1
        return [(float(x * step_x), float(res_y[x])) for x in range(sampled_w)]

    def render(self, frame, mask, points):
        vis = frame.copy()
        vis[mask == 1] = [0, 255, 0]
        vis = cv2.addWeighted(vis, 0.3, frame, 0.7, 0)
        for x, y in points:
            cv2.circle(vis, (int(x), int(y)), 4, (0, 0, 255), -1)
        return vis

    def get_ground_contact_points(self, frame, render=False):
        mask = self._inference(frame)
        points = self._extract_boundary_points(mask)
        vis = self.save_sample_image(frame, mask, points) if render else None
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
