import cv2
import time
import queue
import threading

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

class RpiCamera:
    """
    Raspberry Pi 5 CSI 摄像头控制类
    支持:
    - 拍照
    - 录像
    - 实时帧队列（用于模型/SLAM输入）
    - 保存/加载标定参数
    """

    def __init__(self,
                 width=1280,
                 height=720,
                 fps=30,
                 exposure=8000,
                 auto_exposure=False,
                 awb_mode=0,
                 gain=1.0,
                 queue_size=10,
                 use_gsm=False):
        self.width = width  #分辨率
        self.height = height #分辨率
        self.fps = fps #帧率
        self.exposure = exposure #曝光时间
        self.auto_exposure = auto_exposure #自动曝光开关
        self.awb_mode = awb_mode #自动白平衡模式
        self.gain = gain #模拟增益（亮度放大）

        self.queue_size = queue_size
        self.frame_queue = queue.Queue(maxsize=queue_size)

        self._stop_flag = False
        self._thread = None
        self.use_gsm = use_gsm

    # ----------------------------------------------------------
    # ① 启动解码线程（SLAM / 模型输入）
    # ----------------------------------------------------------
    def start(self):
        if self._thread:
            return
        self._stop_flag = False
        if self.use_gsm:
            self._thread = threading.Thread(target=self._frame_loop_gsm, daemon=True)
        else:
            self._thread = threading.Thread(target=self._frame_loop, daemon=True)

        self._thread.start()

    def stop(self):
        self._stop_flag = True
        if self._thread:
            self._thread.join()
            self._thread = None

    def _frame_loop_gsm(self):
        cap = cv2.VideoCapture(self._make_gst_pipeline(), cv2.CAP_GSTREAMER)
        if not cap.isOpened():
            print("❌ 摄像头无法打开")
            return

        while not self._stop_flag:
            # default is bgr
            ret, frame = cap.read()
            if not ret:
                continue

            # ORB-SLAM3 timestamp，单位秒（double）
            timestamp = time.time()

            # 如果队列满，丢弃旧帧（避免延迟堆积）
            if self.frame_queue.full():
                try:
                    self.frame_queue.get_nowait()
                except:
                    print("loose frames!!!")

            self.frame_queue.put((timestamp, frame))

        cap.release()

    def _make_gst_pipeline(self):
        # 自动曝光？则不设置 exposure
        exposure_part = "" if self.auto_exposure else f"exposure={self.exposure} "

        pipeline = (
            "libcamerasrc "
            f"sensor-mode=4 "
            f"{exposure_part}"
            f"ae-mode={'1' if self.auto_exposure else '0'} "
            f"awb-mode={self.awb_mode} "
            f"gain={self.gain} "
            "! video/x-raw,width={0},height={1},framerate={2}/1 "
            "! videoconvert "
            "! video/x-raw,format=BGR "
            "! appsink drop=true max-buffers=1 sync=false".format(
                self.width, self.height, self.fps
            )
        )
        return pipeline

    def _frame_loop(self):
        cap = cv2.VideoCapture(0)

        # 基本参数设置（可能不生效）
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
        cap.set(cv2.CAP_PROP_FPS, self.fps)

        if not self.auto_exposure:
            cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 0.75)  # 手动模式
            cap.set(cv2.CAP_PROP_EXPOSURE, float(self.exposure) / 1e6)

        if not cap.isOpened():
            print("❌ 摄像头无法打开（/dev/video0）")
            return

        while not self._stop_flag:
            ret, frame = cap.read()
            if not ret:
                continue

            ts = time.time()

            if self.frame_queue.full():
                try:
                    self.frame_queue.get_nowait()
                except:
                    pass

            self.frame_queue.put((ts, frame))

        cap.release()


    # ----------------------------------------------------------
    # ② 获取 SLAM / AI 输入帧
    # ----------------------------------------------------------
    def get_frame(self, rgb=False):
        """返回 (timestamp, img) ，用于SLAM/模型"""
        try:
            f = self.frame_queue.get(timeout=1)
            if rgb:
                # convert RGB -> BGR
                f[1] = cv2.cvtColor(f[1], cv2.COLOR_BGR2RGB)
            return f
        except queue.Empty:
            return None, None

    # ----------------------------------------------------------
    # ③ 拍照
    # ----------------------------------------------------------
    def capture_photo(self, filename="photo.jpg"):
        cap = cv2.VideoCapture(self._make_gst_pipeline(), cv2.CAP_GSTREAMER)
        ret, frame = cap.read()
        if ret:
            cv2.imwrite(filename, frame)
        cap.release()

    # ----------------------------------------------------------
    # ④ 录像（不经过队列）
    # ----------------------------------------------------------
    def record_video(self, filename="video.mp4", duration=10):
        cap = cv2.VideoCapture(self._make_gst_pipeline(), cv2.CAP_GSTREAMER)
        fourcc = cv2.VideoWriter_fourcc(*'mp4v')
        out = cv2.VideoWriter(filename, fourcc, self.fps, (self.width, self.height))

        start = time.time()
        while time.time() - start < duration:
            ret, frame = cap.read()
            if not ret:
                continue
            out.write(frame)

        out.release()
        cap.release()

