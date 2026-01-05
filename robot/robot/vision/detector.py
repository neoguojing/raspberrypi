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
        results = self.model(frame, verbose=False, conf=self.conf)[0]
        contact_pixels = []

        if results.masks is None:
            print(f"🖼 未检测到任何信息: {results}")
            return contact_pixels,None

        # 获取所有类别的索引和掩码
        # 一次性获取所有 cls 以减少循环内计算
        classes = results.boxes.cls.cpu().numpy().astype(int)
        
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
        
        annotated_frame = None
        if render:
            # 1. 先让 YOLO 帮你画好基础的 Mask 和 框
            # labels=True 显示类别, boxes=True 显示方框
            annotated_frame = results.plot(labels=True, boxes=True)

            # 2. 在 YOLO 画好的图上，叠加你自己的三个采样点
            # 假设你已经通过之前的逻辑算出了 contact_pixels
            for pt in contact_pixels:
                cv2.circle(annotated_frame, (int(pt[0]), int(pt[1])), 5, (0, 0, 255), -1)
            
            output_path = "annotated_result.jpg"  # 或者使用绝对路径
            success = cv2.imwrite(output_path, annotated_frame)
            if success:
                print(f"✅ 保存成功: {output_path}")
            else:
                print(f"❌ 保存失败: {output_path}")


        return (contact_pixels , annotated_frame)
    
def main():
    # 1️⃣ 初始化检测器
    detector = SegDetector(model_name="yolo11n-seg.pt", conf=0.45)
    
    # 2️⃣ 读取测试图像
    test_image_path = "asset/test.jpeg"  # 替换为你本地测试图片路径
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

