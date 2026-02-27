import cv2
from robot.robot.vision.segformer_trt import SegFormerTRTDetector
from robot.robot.vision.segformer_transformer import SegFormerTransformerDetector


class SegFormerDetector:
    """兼容入口：backend='transformers' 或 backend='tensorrt'。"""

    def __init__(
        self,
        model_name="nvidia/segformer-b2-finetuned-ade-512-512",
        device=None,
        backend="transformers",
        engine_path=None,
        alpha=0.7,
        conf_threshold=0.25,
    ):
        if backend == "tensorrt":
            if not engine_path:
                raise ValueError("backend=tensorrt 时必须提供 engine_path")
            self.impl = SegFormerTRTDetector(
                engine_path=engine_path,
                alpha=alpha,
                conf_threshold=conf_threshold,
            )
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
