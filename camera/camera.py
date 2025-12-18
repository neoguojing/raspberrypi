# sensor_mode
# IMX219 的模式是：
# mode	分辨率	fps	FOV
# 0	3280×2464	21 fps	full-res
# 1	1920×1080	30 fps	cropped
# 2	3280×2464	30 fps	full-res
# 3	1640×1232	30 fps	2×2 binning
# 4	1640×922	60 fps	2×2 binning
# 5	1280×720	120 fps	cropped
# exposure
# | 场景      | 曝光建议             |
# | ------- | ---------------- |
# | 室内光线一般  | 6000–10000 μs    |
# | 室外阳光强   | 100–2000 μs      |
# | 运动快（小车） | **3000–6000 μs** |
# awb_mode
# | 代码 | 模式           |
# | -- | ------------ |
# | 0  | 自动（默认）       |
# | 1  | 白炽灯 |
# | 2  | 钨丝灯     |
# | 3  | 荧光灯  |
# | 4  | Indoor       |
# | 5  | Daylight     |
# | 6  | Cloudy       |
import sys
import time
import queue
import threading
import cv2
# import libcamera
# from unittest.mock import MagicMock
# # 强行伪造 pykms 模块，防止 picamera2 报错
# # 必须在 from picamera2 import ... 之前执行
# mock_pykms = MagicMock()
# sys.modules["pykms"] = mock_pykms
from picamera2 import Picamera2
from picamera2.encoders import H264Encoder
from picamera2.outputs import FfmpegOutput


class RpiCamera:
    def __init__(self,
                 width=1280,
                 height=720,
                 fps=30,
                 exposure=8000,
                 auto_exposure=True,
                 awb_mode=0,
                 gain=1.0,
                 queue_size=10):
        self.width = width
        self.height = height
        self.fps = fps
        self.exposure = exposure
        self.auto_exposure = auto_exposure
        self.awb_mode = awb_mode
        self.gain = gain
        self.queue_size = queue_size
        self.frame_queue = queue.Queue(maxsize=queue_size)
        self._stop_flag = False
        self._thread = None
        self.picam2 = None  # 单一实例

    def start(self):
        if self._thread and self._thread.is_alive():
            return

        self._stop_flag = False
        self.picam2 = Picamera2()

        # 构建 controls
        controls = {"FrameRate": float(self.fps)}
        
        if not self.auto_exposure:
            controls.update({
                "AeEnable": False,
                "ExposureTime": self.exposure,
                "AnalogueGain": self.gain
            })
        else:
            controls["AeEnable"] = True

        # 白平衡
        if self.awb_mode == 0:
            controls["AwbEnable"] = True
        else:
            controls["AwbEnable"] = False
            AWB_MAP = {
                1: "Incandescent",
                2: "Fluorescent",
                3: "Cloudy",
                4: "Daylight",
                5: "Shade",
                6: "Twilight",
                7: "Custom",
                0: "Auto"  # 别忘了通常 0 是自动
            }
            controls["AwbMode"] = AWB_MAP.get(self.awb_mode, "Daylight")

        # 配置视频流（用于实时队列）
        video_config = self.picam2.create_video_configuration(
            main={"size": (self.width, self.height), "format": "BGR888"},
            controls=controls
        )
        self.picam2.configure(video_config)
        self.picam2.start()
        self.picam2.start(callback=self.frame_callback)

        # self._thread = threading.Thread(target=self._frame_loop, daemon=True)
        # self._thread.start()

    def stop(self):
        self._stop_flag = True
        if self._thread:
            self._thread.join(timeout=2)
            self._thread = None
        if self.picam2:
            self.picam2.stop()
            self.picam2.close()
            self.picam2 = None

    def frame_callback(self, request):
        try:
            # 1. 获取图像数组
            frame_bgr = request.make_array("main")
            
            # --- 防御性判断 ---
            # 检查是否为 None 或非数组对象
            if frame_bgr is None or not hasattr(frame_bgr, 'shape'):
                print("⚠️ 警告: 捕获到非法帧 (None 或非数组)")
                return

            # 检查维度是否完整 (H, W, C)
            if len(frame_bgr.shape) != 3:
                print(f"⚠️ 警告: 帧维度异常: {frame_bgr.shape}")
                return

            # 2. 获取硬件时间戳 (SensorTimestamp 是纳秒)
            # 获取不到时回退到系统时间
            ts_ns = request.metadata.get("SensorTimestamp")
            ts = ts_ns / 1e9 if ts_ns is not None else time.time()
            
            # --- 打印调试信息 (建议生产环境关闭或降级) ---
            # 打印：分辨率 (H, W), 像素类型, 时间戳(秒)
            print(f"📸 Frame Captured | Size: {frame_bgr.shape} | Type: {frame_bgr.dtype} | TS: {ts:.4f}")

            # 3. 更新队列
            if self.frame_queue.full():
                try:
                    self.frame_queue.get_nowait()
                except queue.Empty:
                    pass
            
            self.frame_queue.put((ts, frame_bgr))

        except Exception as e:
            print(f"❌ 回调处理发生错误: {e}")

    def _frame_loop(self):
        while not self._stop_flag:
            frame_bgr = self.picam2.capture_array("main")  # BGR
            # frame_bgr = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
            ts = time.time()
            
            if self.frame_queue.full():
                try:
                    self.frame_queue.get_nowait()
                except queue.Empty:
                    pass
            self.frame_queue.put((ts, frame_bgr))
            time.sleep(1.0 / self.fps)

    def get_frame(self, rgb=False):
        try:
            ts, frame_bgr = self.frame_queue.get(timeout=1)
            return (ts, frame_bgr) if not rgb else (ts, cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2RGB))
        except queue.Empty:
            return None, None

    # ✅ 修复：复用 self.picam2，动态切换配置
    def capture_photo(self, filename="photo.jpg"):
        if not self.picam2:
            print("❌ 摄像头未启动，请先调用 start()")
            return None

        try:
            # 保存当前配置
            old_config = self.picam2.camera_config

            # 切换到高分辨率拍照配置
            photo_config = self.picam2.create_still_configuration(
                main={"size": (1920, 1080)}
            )
            self.picam2.stop()          # 停止当前流
            self.picam2.configure(photo_config)
            self.picam2.start()
            
            frame_bgr = self.picam2.capture_array()
            # frame_bgr = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
            cv2.imwrite(filename, frame_bgr)
            
            # 恢复视频流配置
            self.picam2.stop()
            self.picam2.configure(old_config)
            self.picam2.start()
            
            print(f"✅ 照片已保存: {filename}")
            return frame_bgr

        except Exception as e:
            print(f"❌ 拍照失败: {e}")
            # 尝试恢复原配置
            try:
                self.picam2.stop()
                self.picam2.configure(old_config)
                self.picam2.start()
            except:
                pass
            return None

    # ✅ 修复：复用 self.picam2 录像
    def record_video(self, filename="video.mp4", duration=10, use_h265=False):
        if not self.picam2:
            print("❌ 摄像头未启动，请先调用 start()")
            return

        try:
            # 保存当前配置
            old_config = self.picam2.camera_config

            # 创建视频配置（与当前流一致）
            video_config = self.picam2.create_video_configuration(
                main={"size": (self.width, self.height)},
                controls={"FrameRate": float(self.fps)}
            )
            self.picam2.stop()
            self.picam2.configure(video_config)
            self.picam2.start()

            # 选择编码器
            # encoder = H265Encoder(10_000_000) if use_h265 else H264Encoder(10_000_000)
            encoder = H264Encoder(10_000_000)
            output = FfmpegOutput(filename, audio=False)
            
            self.picam2.start_recording(encoder, output)
            time.sleep(duration)
            self.picam2.stop_recording()

            # 恢复原配置
            self.picam2.stop()
            self.picam2.configure(old_config)
            self.picam2.start()
            
            print(f"✅ 视频已保存: {filename}")

        except Exception as e:
            print(f"❌ 录像失败: {e}")
            # 尝试恢复
            try:
                self.picam2.stop()
                self.picam2.configure(old_config)
                self.picam2.start()
            except:
                pass
            
if __name__ == "__main__":
    cam = RpiCamera(width=1280, height=720, fps=30, auto_exposure=False, exposure=10000, gain=1.5)
    cam.start()

    # 获取一帧用于 SLAM
    ts, frame = cam.get_frame(rgb=False)  # BGR
    if frame is not None:
        # 保存为 BGR 图像（OpenCV 默认格式）
        filename = f"frame_{ts:.6f}.jpg"
        cv2.imwrite(filename, frame)
    # 拍照
    cam.capture_photo("test.jpg")

    # 录像 5 秒
    cam.record_video("test.mp4", duration=5)

    cam.stop()