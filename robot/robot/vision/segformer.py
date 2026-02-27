import cv2
import os
import time


class _TRTEngine:
    def __init__(self, engine_path):
        import tensorrt as trt
        import pycuda.autoinit  # noqa: F401
        import pycuda.driver as cuda

        self.trt = trt
        self.cuda = cuda
        self.logger = trt.Logger(trt.Logger.WARNING)
        self.runtime = trt.Runtime(self.logger)
        with open(engine_path, "rb") as f:
            self.engine = self.runtime.deserialize_cuda_engine(f.read())
        if self.engine is None:
            raise RuntimeError(f"TensorRT engine load failed: {engine_path}")

        self.context = self.engine.create_execution_context()
        self.stream = cuda.Stream()

        self.input_name = None
        self.output_name = None
        self.input_shape = None
        self.output_shape = None
        self.input_host = None
        self.output_host = None
        self.input_device = None
        self.output_device = None

        for i in range(self.engine.num_io_tensors):
            name = self.engine.get_tensor_name(i)
            mode = self.engine.get_tensor_mode(name)
            shape = tuple(self.context.get_tensor_shape(name))
            dtype = trt.nptype(self.engine.get_tensor_dtype(name))
            if -1 in shape:
                raise ValueError(f"TensorRT engine must be static shape, got {name}: {shape}")
            size = int(np.prod(shape))
            host = cuda.pagelocked_empty(size, dtype)
            device = cuda.mem_alloc(host.nbytes)
            self.context.set_tensor_address(name, int(device))
            if mode == trt.TensorIOMode.INPUT:
                self.input_name = name
                self.input_shape = shape
                self.input_host = host
                self.input_device = device
            else:
                self.output_name = name
                self.output_shape = shape
                self.output_host = host
                self.output_device = device

        if self.input_name is None or self.output_name is None:
            raise RuntimeError("TensorRT engine must contain exactly one input and one output tensor")

    def infer(self, input_tensor):
        np.copyto(self.input_host, input_tensor.ravel())
        self.cuda.memcpy_htod_async(self.input_device, self.input_host, self.stream)
        ok = self.context.execute_async_v3(stream_handle=self.stream.handle)
        if not ok:
            raise RuntimeError("TensorRT execute_async_v3 failed")
        self.cuda.memcpy_dtoh_async(self.output_host, self.output_device, self.stream)
        self.stream.synchronize()
        return self.output_host.reshape(self.output_shape)

class SegFormerDetector:
    """兼容入口：backend='transformers' 或 backend='tensorrt'。"""

    def __init__(
        self,
        model_name="nvidia/segformer-b2-finetuned-ade-512-512",
        device=None,
        backend="tensorrt",
        engine_path=None,
        alpha=0.7,              # EMA（指数移动平均）系数。值越大，对历史帧依赖越强，画面越稳，但动态响应越慢
        conf_threshold=0.25     # 地面置信度阈值。过滤掉模型犹豫不决的预测区域
    ):
        # 自动选择硬件：优先使用 CUDA
        self.device = device or ("cuda" if torch.cuda.is_available() else "cpu")

        self.backend = backend

        # 加载推理后端
        self.model = None
        self.trt_engine = None
        self.processor = None

        if self.backend == "tensorrt":
            if not engine_path:
                raise ValueError("backend=tensorrt 时必须提供 engine_path")
            self.trt_engine = _TRTEngine(engine_path)
        else:
            from transformers import SegformerImageProcessor, SegformerForSemanticSegmentation

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
        id2label = self.model.config.id2label if self.model is not None else {}
        
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
        # img = self._preprocess_lighting(frame)
        img = frame
        if self.backend == "tensorrt":
            n, c, h, w = self.trt_engine.input_shape
            rgb = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
            resized = cv2.resize(rgb, (w, h), interpolation=cv2.INTER_LINEAR)
            pixel_values = resized.astype(np.float32) / 255.0
            mean = np.array([0.485, 0.456, 0.406], dtype=np.float32)
            std = np.array([0.229, 0.224, 0.225], dtype=np.float32)
            pixel_values = (pixel_values - mean) / std
            pixel_values = np.transpose(pixel_values, (2, 0, 1))[None, ...]
            if pixel_values.shape != (n, c, h, w):
                raise ValueError(
                    f"TensorRT input shape mismatch: expect {(n, c, h, w)}, got {pixel_values.shape}"
                )

            logits = self.trt_engine.infer(pixel_values.astype(np.float32))
            logits = torch.from_numpy(logits)
            probs = torch.softmax(logits, dim=1)
        else:
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

        pred_map = probs.argmax(axis=0)
        self.print_detected_categories(pred_map)
        # 3. 提取地面相关类别的最大概率值
        # 结果是一个 [H, W] 的矩阵，每个像素值代表“该点属于地面”的信心得分
        ground_prob = probs[self.ground_classes].max(axis=0)

        # 4. 指数移动平均 (EMA) 平滑处理
        # 作用：过滤掉由于单帧噪点、动态模糊或快速阴影漂移引起的“检测空洞”
        if self.ema_ground_prob is None:
            self.ema_ground_prob = ground_prob
        else:
            self.impl = SegFormerTransformerDetector(
                model_name=model_name,
                device=device,
                alpha=alpha,
                conf_threshold=conf_threshold,
            )

    def __getattr__(self, item):
        return getattr(self.impl, item)


def main():
    detector = SegFormerDetector(alpha=0.7, conf_threshold=0.25)
    frame = cv2.imread("asset/test.png")
    if frame is None:
        print("❌ 错误：无法加载测试图片，请检查路径。")
        return

    print("🚀 正在模拟处理 5 帧连续图像...")
    for _ in range(5):
        pts, _ = detector.get_ground_contact_points(frame)

    print(f"✅ 成功！当前帧检测到 {len(pts)} 个边界引导点。")


if __name__ == "__main__":
    main()
