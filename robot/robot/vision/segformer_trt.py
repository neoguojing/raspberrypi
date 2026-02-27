import numpy as np
import cv2
import os
import time
from robot.robot.vision.trt_engine import TRTEngine

class SegFormerTRTDetector:
    def __init__(self, engine_path, alpha=0.7, conf_threshold=0.25):
        self.id2label = {}
        self.ground_classes = [3, 6, 11, 13, 21, 26, 27, 28, 46, 52, 54, 60, 91, 94, 109, 131, 147]
        self.alpha = alpha
        self.conf_threshold = conf_threshold
        self.ema_ground_prob = None
        self.clahe = cv2.createCLAHE(2.0, (8, 8))
        self.trt_engine = TRTEngine(engine_path)

    def _resize_probs_to_frame(self, probs, frame_hw):
        h, w = frame_hw
        resized = np.zeros((probs.shape[0], h, w), dtype=np.float32)
        for i in range(probs.shape[0]):
            resized[i] = cv2.resize(probs[i], (w, h), interpolation=cv2.INTER_LINEAR)
        return resized

    def _softmax_np(self, x, axis=0):
        x = x - np.max(x, axis=axis, keepdims=True)
        e = np.exp(x)
        return e / np.sum(e, axis=axis, keepdims=True)

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
        n, c, h, w = self.trt_engine.input_shape
        rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        resized = cv2.resize(rgb, (w, h), interpolation=cv2.INTER_LINEAR)
        x = resized.astype(np.float32) / 255.0
        mean = np.array([0.485, 0.456, 0.406], dtype=np.float32)
        std = np.array([0.229, 0.224, 0.225], dtype=np.float32)
        x = (x - mean) / std
        x = np.transpose(x, (2, 0, 1))[None, ...]
        if x.shape != (n, c, h, w):
            raise ValueError(f"TensorRT input shape mismatch: expect {(n, c, h, w)}, got {x.shape}")

        logits = self.trt_engine.infer(x.astype(np.float32))
        probs = self._softmax_np(logits[0], axis=0)
        return self._resize_probs_to_frame(probs, frame.shape[:2])

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
