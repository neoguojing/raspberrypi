import torch
import numpy as np
import cv2
import os
import time
from transformers import SegformerImageProcessor, SegformerForSemanticSegmentation

class SegFormerDetector:
    def __init__(
        self,
        model_name="nvidia/segformer-b2-finetuned-ade-512-512",
        device=None,
        alpha=0.7,              # EMA（指数移动平均）系数。值越大，对历史帧依赖越强，画面越稳，但动态响应越慢
        conf_threshold=0.25     # 地面置信度阈值。过滤掉模型犹豫不决的预测区域
    ):
        # 自动选择硬件：优先使用 CUDA
        self.device = device or ("cuda" if torch.cuda.is_available() else "cpu")

        # 加载 SegFormer 处理器和预训练模型
        self.processor = SegformerImageProcessor.from_pretrained(model_name)
        self.model = SegformerForSemanticSegmentation.from_pretrained(model_name).to(self.device)

        # 如果使用 GPU，开启半精度推理 (FP16)，可显著提升 RTX 显卡的推理速度并减少显存占用
        if self.device == "cuda":
            self.model.half()
        self.model.eval()

        # ADE20K 协议中定义的地面、路面相关类别 ID
        # 聚合这些 ID 可以忽略具体的路面材质（地毯、草地或柏油路），统一视为“可行走区域”
        self.ground_classes = [
            3, 6, 11, 13, 21, 26, 27, 28,
            46, 52, 54, 60, 91, 94, 109, 131, 147
        ]

        self.alpha = alpha
        self.conf_threshold = conf_threshold
        self.ema_ground_prob = None  # 用于缓存上一帧平滑后的概率图

        # 初始化 CLAHE (对比度受限的自适应直方图均衡化)
        # 用于抑制图像中的强光闪烁，并增强暗部阴影中的细节纹理
        self.clahe = cv2.createCLAHE(2.0, (8, 8))

    # ----------------------------------------------------
    # 图像预处理：光照平衡
    # ----------------------------------------------------
    def _preprocess_lighting(self, frame):
        """
        通过 LAB 色彩空间对亮度通道(L)进行均衡化，
        解决室内外光照不均或阴影导致的路面漏检问题。
        """
        lab = cv2.cvtColor(frame, cv2.COLOR_BGR2LAB)
        l, a, b = cv2.split(lab)
        l = self.clahe.apply(l)  # 仅增强亮度，不破坏色彩平衡
        lab = cv2.merge((l, a, b))
        return cv2.cvtColor(lab, cv2.COLOR_LAB2RGB)

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

    # ----------------------------------------------------
    # 核心推理逻辑：时域平滑 + 概率过滤
    # ----------------------------------------------------
    def _inference(self, frame):
        # 1. 增强图像并准备模型输入
        img = self._preprocess_lighting(frame)
        inputs = self.processor(images=img, return_tensors="pt")
        inputs = {k: v.to(self.device) for k, v in inputs.items()}

        if self.device == "cuda":
            inputs = {k: v.half() for k, v in inputs.items()}

        with torch.no_grad():
            logits = self.model(**inputs).logits
            # 将输出转换为概率分布 (Softmax)
            probs = torch.softmax(logits, dim=1)

        # 2. 将低分辨率的 Logits 双线性插值回原始图像尺寸
        probs = torch.nn.functional.interpolate(
            probs,
            size=frame.shape[:2],
            mode="bilinear",
            align_corners=False
        )[0].cpu().numpy()

        pred_map = probs.argmax(dim=1)[0].cpu().numpy()
        self.print_detected_categories(pred_map)
        # 3. 提取地面相关类别的最大概率值
        # 结果是一个 [H, W] 的矩阵，每个像素值代表“该点属于地面”的信心得分
        ground_prob = probs[self.ground_classes].max(axis=0)

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

        # 5. 二值化：只有当平滑后的地面概率超过阈值时，才判定为地面
        ground_mask = (self.ema_ground_prob > self.conf_threshold).astype(np.uint8)

        # 6. 形态学闭运算 (Closing)
        # 作用：填充地面掩码中细小的黑色空洞（如地砖缝隙、细小阴影），同时保持边缘位置准确
        kernel = np.ones((3, 3), np.uint8)
        ground_mask = cv2.morphologyEx(ground_mask, cv2.MORPH_CLOSE, kernel)

        return ground_mask

    # ----------------------------------------------------
    # 边界提取：向量化扫描
    # ----------------------------------------------------
    def _extract_boundary_points(self, ground_mask, step_x=10):
        """
        从底部向上扫描，寻找地面与非地面（障碍物）的交界线点。
        使用向量化操作代替 Python 循环，极大提升效率。
        """
        h, w = ground_mask.shape
        # 按步长采样列，减少计算量
        sampled = ground_mask[:, ::step_x]

        # 计算垂直方向差分。当结果为 1 时，代表发生了 1(地面) -> 0(障碍物) 的跳变
        # 
        diff = sampled[:-1] - sampled[1:]
        ys, xs = np.where(diff == 1)

        # 预设所有列的交界点都在图像最底部 (h-1)
        bottom_y = np.full(sampled.shape[1], h - 1)

        # 对于每一列，记录最靠近图像下方的跳变点（即最近的障碍物接触点）
        for y, x in zip(ys, xs):
            # 因为扫描是从上往下的，我们需要找到该列最大的 y（最靠下）
            # 这里的逻辑通过遍历更新，保证 bottom_y 存储的是最靠下的边界点
            bottom_y[x] = min(bottom_y[x], y)

        # 还原回原始图像的 x 坐标并打包成坐标对
        contact_pixels = [
            (float(x * step_x), float(y))
            for x, y in enumerate(bottom_y)
        ]
        return contact_pixels

    # ----------------------------------------------------
    # 可视化渲染
    # ----------------------------------------------------
    def render(self, frame, mask, points):
        """
        在原图上叠加绿色透明地面蒙版和红色边缘触点。
        """
        vis = frame.copy()
        vis[mask == 1] = [0, 255, 0]  # 地面涂绿
        # 混合原图与蒙版
        vis = cv2.addWeighted(vis, 0.3, frame, 0.7, 0)

        # 绘制边界接触点
        for x, y in points:
            cv2.circle(vis, (int(x), int(y)), 4, (0, 0, 255), -1)
        return vis

    # ----------------------------------------------------
    # 外部统一接口
    # ----------------------------------------------------
    def get_ground_contact_points(self, frame, render=True):
        """
        输入 BGR 图像，返回边界点列表及（可选的）可视化结果。
        """
        mask = self._inference(frame)
        points = self._extract_boundary_points(mask)
        vis = self.save_sample_image(frame, mask, points) if render else None
        return points, vis

    # ----------------------------------------------------
    # [补充] 自动采样保存接口
    # ----------------------------------------------------
    def save_sample_image(self, frame, mask, points, folder="samples", max_count=10, interval_seconds=10):
        """
        按时间间隔自动保存检测结果图，用于离线分析稳定性。
        
        Args:
            visual_frame: render() 方法返回的可视化 BGR 图像
            folder: 保存文件夹路径
            max_count: 最大保存数量，达到后会循环覆盖（滚动记录）
            interval_seconds: 保存的时间间隔（秒），避免频繁写磁盘
        """

        # 初始化计数器和时间记录（仅在第一次调用时执行）
        if not hasattr(self, 'last_save_time'):
            self.last_save_time = 0
        if not hasattr(self, 'saved_images_count'):
            self.saved_images_count = 0

        current_time = time.time()

        # 检查是否满足保存的时间间隔
        if current_time - self.last_save_time >= interval_seconds:
            visual_frame = self.render(frame, mask, points)
            # 更新最后保存时间
            self.last_save_time = current_time
            
            # 计算滚动索引 (例如 1, 2, 3...10, 1, 2...)
            save_index = (self.saved_images_count % max_count) + 1
            self.saved_images_count += 1
            
            # 确保目录存在
            os.makedirs(folder, exist_ok=True)
            
            # 文件名拼接：包含索引以实现自动循环覆盖
            file_path = os.path.join(folder, f"sample_{save_index}.jpg")
            
            # 执行写入
            cv2.imwrite(file_path, visual_frame)
            
            print(f"📸 [采样成功] 已保存至: {file_path} (累计保存: {self.saved_images_count} 张)")

            return visual_frame
        
        return None
# ========================================================
# 测试入口
# ========================================================
def main():
    # 初始化检测器
    detector = SegFormerDetector(alpha=0.7, conf_threshold=0.25)

    frame = cv2.imread("asset/test.png")
    if frame is None:
        print("❌ 错误：无法加载测试图片，请检查路径。")
        return

    # 模拟视频流处理过程，观察 EMA 平滑效果
    print("🚀 正在模拟处理 5 帧连续图像...")
    for _ in range(5):
        pts, vis = detector.get_ground_contact_points(frame)

    print(f"✅ 成功！当前帧检测到 {len(pts)} 个边界引导点。")

if __name__ == "__main__":
    main()