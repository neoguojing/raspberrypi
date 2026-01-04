import cv2
import threading
import queue
import time

class VideoReader:
    def __init__(self, source, to_rgb=True, queue_size=50):
        """
        :param source: 可以是文件路径 (str), RTSP地址 (str), 或摄像头索引 (int)
        :param to_rgb: 是否返回 RGB 格式
        :param queue_size: 缓冲队列大小
        """
        self.source = source
        self.to_rgb = to_rgb
        
        # 判断是否为实时流 (RTSP 或 摄像头)
        self.is_stream = str(source).startswith(('rtsp', 'rtmp', 'http')) or isinstance(source, int)
        
        self.cap = cv2.VideoCapture(source)
        if not self.cap.isOpened():
            raise ValueError(f"无法打开视频源: {source}")

        # 对于 RTSP 流，限制底层缓冲区以降低延迟
        if self.is_stream:
            self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
            # 实时流通常不需要太大的队列，1-5 即可，确保新鲜度
            queue_size = min(queue_size, 5)

        self.frame_queue = queue.Queue(maxsize=queue_size)
        self.running = True
        
        self.read_thread = threading.Thread(target=self._update, daemon=True)
        self.read_thread.start()

    def _update(self):
        """后台读取线程"""
        while self.running:
            ret, frame = self.cap.read()
            
            if not ret:
                if not self.is_stream:
                    # 文件读取完毕
                    self.running = False
                    break
                else:
                    # 流断开，尝试重连
                    print(f"流断开，正在尝试重连: {self.source}")
                    self.cap.open(self.source)
                    time.sleep(1)
                    continue

            # 核心策略：实时流丢弃旧帧，文件流阻塞等待
            if self.is_stream:
                if self.frame_queue.full():
                    try:
                        self.frame_queue.get_nowait() # 移除最旧的一帧
                    except queue.Empty:
                        pass
                self.frame_queue.put(frame)
            else:
                # 文件模式：队列满时阻塞，确保每一帧都处理到
                self.frame_queue.put(frame, block=True)

    def get_frame(self, rgb=None):
        """
        获取一帧
        :param rgb: 覆盖初始化的 to_rgb 设置
        :return: frame 或 None
        """
        use_rgb = rgb if rgb is not None else self.to_rgb
        try:
            # 等待时间根据是否是流来调整
            wait_time = 0.5 if self.is_stream else 5.0
            frame = self.frame_queue.get(timeout=wait_time)
            
            if use_rgb:
                frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            return frame
        except queue.Empty:
            return None

    def stop(self):
        self.running = False
        if self.read_thread.is_alive():
            self.read_thread.join()
        if self.cap.isOpened():
            self.cap.release()

    def __del__(self):
        self.stop()

# --- 使用示例 ---
if __name__ == "__main__":
    # 示例 1: 读取本地文件
    # reader = VideoReader("data/test.mp4")
    
    # 示例 2: 读取 RTSP 流
    reader = VideoReader("rtsp://admin:123456@192.168.1.10:554/h264")
    
    try:
        while True:
            frame = reader.get_frame(rgb=True)
            if frame is None:
                if not reader.running: break # 文件读完退出
                continue # 流等待中
                
            # 这里进行你的图像处理
            # cv2.imshow("Frame", cv2.cvtColor(frame, cv2.COLOR_RGB2BGR))
            # if cv2.waitKey(1) & 0xFF == ord('q'): break
    finally:
        reader.stop()