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
from picamera2 import Picamera2,MappedArray
from picamera2.encoders import H264Encoder
from picamera2.outputs import FfmpegOutput


class RpiCamera:
    def __init__(self,
                 width=1640,
                 height=1232,
                 fps=20,
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
        self.picam2 = None  # 单一实例

    def start(self):
        self.picam2 = Picamera2()

        # 构建 controls
        controls = {"FrameRate": float(self.fps),'FrameDurationLimits':(47000, 100000)}
        
        # 2. 曝光控制
        if not self.auto_exposure:
            controls.update({
                "AeEnable": False,
                "ExposureTime": int(self.exposure), # 确保是整数，单位微秒
                "AnalogueGain": float(self.gain)
            })
        else:
            controls["AeEnable"] = True

        # 3. 白平衡控制 (关键修正点)
        # libcamera 中：AwbEnable 应该始终为 True，除非你要手动指定 ColorGains
        if self.awb_mode == 0:
            controls["AwbEnable"] = True
            controls["AwbMode"] = 0  # Auto
        else:
            controls["AwbEnable"] = True 
            # 这里的 AWB_MAP 对应 libcamera 的索引
            # 0: Auto, 1: Incandescent, 2: Tungsten, 3: Fluorescent, 
            # 4: Indoor, 5: Daylight, 6: Cloudy
            AWB_MAP = {
                0: 0, # Auto
                1: 1, # Incandescent (白炽灯)
                2: 3, # Fluorescent (荧光灯)
                3: 6, # Cloudy (多云)
                4: 5, # Daylight (日光)
                5: 4, # Indoor (室内)
            }
            # 获取对应的索引数字
            controls["AwbMode"] = AWB_MAP.get(self.awb_mode, 0)

        full_mode = None
        print(self.picam2.sensor_modes)
        for mode in self.picam2.sensor_modes:
            # 找分辨率最大的 mode 即 full FOV
            if mode["size"] == (3280, 2464):
                full_mode = mode
                break

        # 2. 配置 sensor
        self.sensor_conf = {
            "output_size": full_mode["size"],
            "bit_depth": full_mode["bit_depth"]
        }

        # 配置视频流（用于实时队列）
        video_config = self.picam2.create_video_configuration(
            # main={"size": (self.width, self.height), "format": "BGR888"},
            main={"size": (self.width, self.height), "format": "RGB888"},
            # sensor=self.sensor_conf,
            raw={"size": (1640, 1232), "format": "SRGGB10_CSI2P"},
            controls=controls,
            use_case='video'
        )
        print("Camera Controls:", video_config)
        print(f"Camera Config: {self.picam2.camera_config}")
        self.picam2.configure(video_config)
        self.picam2.pre_callback = self._event_loop_callback        

        self.picam2.start()

    def stop(self):
        if self.picam2:
            self.picam2.stop()
            self.picam2.close()
            self.picam2 = None

    def _event_loop_callback(self, request):
        """
        按照文档 8.2.1 节建议的事件循环回调
        """
        try:
            # 使用 MappedArray 实现零拷贝访问 (In-place access)
            # 这比 request.make_array() 更快，因为它直接映射内存
            with MappedArray(request, "main") as m:
                # 此时 m.array 就是一个 numpy 数组
                # 由于你设置了 BGR888，这里拿到的直接就是 BGR
                frame_rgb = m.array.copy() # copy 是为了防止内存被底层回收
                # --- 防御性判断 ---
                # 检查是否为 None 或非数组对象
                if frame_rgb is None or not hasattr(frame_rgb, 'shape'):
                    print("⚠️ 警告: 捕获到非法帧 (None 或非数组)")
                    return

                # 检查维度是否完整 (H, W, C)
                if len(frame_rgb.shape) != 3:
                    print(f"⚠️ 警告: 帧维度异常: {frame_rgb.shape}")
                    return
                
            # 获取硬件时间戳
            metadata = request.get_metadata()
            ts = metadata.get("SensorTimestamp")
            ts = ts / 1e9 if ts else time.time()

            # print(f"📸 Frame Captured | Size: {frame_rgb.shape} | Type: {frame_rgb.dtype} | TS: {ts:.4f} | Meta: {metadata}")

            # 入队逻辑
            if self.frame_queue.full():
                try:
                    self.frame_queue.get_nowait()
                except queue.Empty:
                    pass
            self.frame_queue.put((ts, frame_rgb))

        except Exception as e:
            print(f"Callback error: {e}")

    def get_frame(self, rgb=True):
        try:
            ts, frame_rgb = self.frame_queue.get(timeout=1)
            return (ts, frame_rgb) if rgb else (ts, cv2.cvtColor(frame_rgb, cv2.COLOR_RGB2BGR))
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
                main={"size": (1920, 1080)},
                raw={"size": (3280, 2464), "format": "SRGGB10_CSI2P"},
            )
            self.picam2.stop()          # 停止当前流
            self.picam2.configure(photo_config)
            self.picam2.start()
            
            # 3. 手动强制设置参数 (关键点)
            # ExposureTime 单位是微秒 (us)
            # AnalogueGain 是模拟增益
            self.picam2.set_controls({"AeEnable": True})
            
            # 4. 等待 2 秒让硬件寄存器生效
            time.sleep(2)
            
            frame_rgb = self.picam2.capture_array()
            # frame_rgb = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
            cv2.imwrite(filename, frame_rgb)
            
            # 恢复视频流配置
            self.picam2.stop()
            self.picam2.configure(old_config)
            self.picam2.start()
            
            print(f"✅ 照片已保存: {filename}")
            return frame_rgb

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
                # sensor=self.sensor_conf,
                raw={"size": (3280, 2464), "format": "SRGGB10_CSI2P"},
                controls={"FrameRate": float(self.fps),"AeEnable": True,'FrameDurationLimits':(33333, 100000)}
            )
            self.picam2.stop()
            self.picam2.configure(video_config)
            self.picam2.start()
            
        # -----------------------
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
    cam = RpiCamera()
    cam.start()

    # 获取一帧用于 SLAM
    for _ in range(1):
        ts, frame = cam.get_frame()
        if frame is not None:
            filename = f"frame_{ts:.6f}.jpg"
            cv2.imwrite(filename, frame)
    # 拍照
    cam.capture_photo()

    # 录像 5 秒
    cam.record_video("test.mp4", duration=5)

    cam.stop()