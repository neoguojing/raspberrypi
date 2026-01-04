import cv2
import threading
import queue
import time

class VideoReader:
    def __init__(self, video_path, to_rgb=True, queue_size=100):
        """
        :param video_path: 视频文件路径
        :param to_rgb: 默认初始化是否转换 RGB
        :param queue_size: 队列最大长度，防止内存溢出
        """
        self.cap = cv2.VideoCapture(video_path)
        if not self.cap.isOpened():
            raise ValueError(f"无法打开视频: {video_path}")

        self.to_rgb = to_rgb
        
        # 线程安全队列
        self.frame_queue = queue.Queue(maxsize=queue_size)
        self.running = True
        
        # 启动后台读取线程
        self.read_thread = threading.Thread(target=self._update, daemon=True)
        self.read_thread.start()

    def _update(self):
        """后台线程：不断读取帧并存入队列"""
        while self.running:
            if not self.frame_queue.full():
                ret, frame = self.cap.read()
                if not ret:
                    self.running = False
                    break
                
                # 放入队列（若队列满会阻塞直到有空位）
                self.frame_queue.put(frame)
            else:
                # 队列满时稍微休眠，减少 CPU 占用
                time.sleep(0.01)
        
        self.cap.release()

    def get_frame(self, rgb=True):
        """
        外部调用的接口
        :param rgb: 是否转换为 RGB (覆盖初始化设置)
        :return: Numpy 数组或 None (读取完毕时)
        """
        try:
            # 从队列获取一帧 (设置超时防止永久阻塞)
            frame = self.frame_queue.get(timeout=2)
            
            if rgb:
                frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            return frame
        except queue.Empty:
            return None

    def stop(self):
        """手动停止读取"""
        self.running = False
        if self.read_thread.is_alive():
            self.read_thread.join()

    def __del__(self):
        self.stop()

# --- 使用示例 ---
if __name__ == "__main__":
    reader = VideoReader("test_video.mp4")
    
    while True:
        frame = reader.get_frame(rgb=True)
        if frame is None:
            print("视频读取完毕或超时")
            break
            
        # 在这里进行你的处理
        # print(f"获取到帧，形状: {frame.shape}")
        
    reader.stop()