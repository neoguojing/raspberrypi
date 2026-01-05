from ultralytics import YOLO
import numpy as np
import os
import cv2
class SegDetector:
    def __init__(
        self,
        model_name="yolo11n-seg.pt",
        conf=0.45
    ):
        self.conf = conf
        self.model = YOLO(model_name)        # 触发 Ultralytics 下载
        self.obstacle_ids = [
            0, 1, 2, 3, 5, 7, 24, 26, 32, 39, 41, 64, 67
        ]

    def get_ground_contact_points(self, frame, render=True):
        results = self.model(frame, verbose=False, conf=self.conf)[0].cpu()
        num_objects = len(results.boxes)
        if num_objects > 0:
            # results.names 是类别字典，例如 {0: 'person', 1: 'car'}
            counts = results.verbose() 
            print(f"检测详情: {counts}")
        contact_pixels = []

        # 获取所有类别的索引和掩码
        # 一次性获取所有 cls 以减少循环内计算
        # classes = results.boxes.cls.cpu().numpy().astype(int)
        
        # ========== [新增] Box Ground Contact 兜底函数 ==========
        def box_ground_contact():
            if results.boxes is None:
                return
            for box in results.boxes:
                x1, y1, x2, y2 = box.xyxy[0]
                yb = int(y2)

                # 忽略极小 box（远处噪声）
                if (y2 - y1) < 8 or (x2 - x1) < 8:
                    continue

                # 在 box 底边均匀采样 3 个点
                num = max(3, int((x2 - x1) / 20))
                xs = np.linspace(x1, x2, num=num)
                for x in xs:
                    contact_pixels.append((float(x), float(yb)))

        def mask_ground_contact():
            if results.masks is None:
                return         
            for i, mask in enumerate(results.masks.xy):
                # 1. 类别过滤，无需过滤，地面不会被检测到
                # if classes[i] not in self.obstacle_ids:
                #     continue

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
        
        mask_ground_contact()
        # ========== [新增] 如果 mask 失败，使用 box ==========
        if len(contact_pixels) < 3:
            contact_pixels.clear()
            # print("⚠️ mask 点不足，使用 box 兜底")
            box_ground_contact()

        contact_pixels = list(set(
            (int(p[0]), int(p[1])) for p in contact_pixels
        ))
        
        annotated_frame = None
        if render:
            # 初始化自定义属性（如果尚未定义）
            if not hasattr(self, 'saved_images_count'):
                self.saved_images_count = 0  # 已保存的数量
                self.frame_counter = 0       # 经历的总帧数
                self.max_save_count = 10     # 最大保存上限
                self.save_interval = 20      # 采样间隔：每隔多少帧存一张

            self.frame_counter += 1
            annotated_frame = results.plot(labels=True, boxes=True)

            # 渲染采样点
            for pt in contact_pixels:
                cv2.circle(annotated_frame, (int(pt[0]), int(pt[1])), 5, (0, 0, 255), -1)

            # 判定条件：每隔固定帧数采样，且总数不超过 10 张
            if self.saved_images_count < self.max_save_count and self.frame_counter % self.save_interval == 0:
                self.saved_images_count += 1
                
                save_dir = "samples"
                if not os.path.exists(save_dir):
                    os.makedirs(save_dir)

                # 文件名：包含序列号
                output_path = os.path.join(save_dir, f"sample_{self.saved_images_count}.jpg")
                
                success = cv2.imwrite(output_path, annotated_frame)
                if success:
                    print(f"📸 已采样 ({self.saved_images_count}/{self.max_save_count}): {output_path}")


        return (contact_pixels , annotated_frame)
    
def main():
    # 1️⃣ 初始化检测器
    detector = SegDetector(model_name="yolo11n-seg.pt", conf=0.45)
    
    # 2️⃣ 读取测试图像
    test_image_path = "asset/test.png"  # 替换为你本地测试图片路径
    if not os.path.exists(test_image_path):
        print(f"❌ 测试图片不存在: {test_image_path}")
        return
    
    frame = cv2.imread(test_image_path)
    if frame is None:
        print(f"❌ 无法读取图像: {test_image_path}")
        return
    print(f"🖼 成功读取测试图像: {frame.shape}")
    
    # 3️⃣ 获取接触点并可视化
    contact_pixels, _ = detector.get_ground_contact_points(frame, render=True)
    
    print(f"🔹 检测到 {len(contact_pixels)} 个接触点:")
    for i, pt in enumerate(contact_pixels):
        print(f"  点 {i}: x={pt[0]:.2f}, y={pt[1]:.2f}")


if __name__ == "__main__":
    main()

