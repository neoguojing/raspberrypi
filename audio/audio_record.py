import threading
import collections

import pyaudio
import audioop
from typing import Deque
import asyncio
import webrtcvad
import logging
log = logging.getLogger(__name__)
from .config import (
    AUDIO_RATE,
    AUDIO_CHANNELS,
    AUDIO_CHUNK_SIZE,
    AUDIO_DEVICE_INDEX,
    RECORD_SECONDS_AFTER_WAKE,
    SILENT_CHUNKS_NEEDED,
    SILENCE_THRESHOLD
)
from tools.utils import pcm_to_wav


class ContinuousAudioListener:
    """
    持续监听麦克风，将音频帧写入环形缓冲，并提供录音方法。
    """
    def __init__(self):
        self.rate = AUDIO_RATE
        self.channels = AUDIO_CHANNELS
        self.chunk_size = AUDIO_CHUNK_SIZE
        self.device_index = AUDIO_DEVICE_INDEX

        # 环形缓冲：保存最近 BUFFER_SECONDS 秒的数据
        max_frames = int(self.rate / self.chunk_size * RECORD_SECONDS_AFTER_WAKE)
        self._buffer: Deque[bytes] = collections.deque(maxlen=max_frames)

        self._pa = pyaudio.PyAudio()
        self._stream = None
        self._stop_flag = threading.Event()
        
        self.vad_frame_duration_ms = 30
        self.vad_frame_bytes = int(self.rate * self.vad_frame_duration_ms / 1000) * 2
        self.vad = webrtcvad.Vad()
        self.vad.set_mode(2)

    def _audio_callback(self, in_data, frame_count, time_info, status):
        # 写入环形缓冲
        self._buffer.append(in_data)
        return (None, pyaudio.paContinue)

    def start(self):
        """启动持续监听音频流，带异常处理"""
        try:
            self._stream = self._pa.open(
                rate=self.rate,
                channels=self.channels,
                format=pyaudio.paInt16,
                input=True,
                frames_per_buffer=self.chunk_size,
                input_device_index=self.device_index,
                stream_callback=self._audio_callback
            )
            self._stream.start_stream()
            print(f"[INFO] Audio stream started: rate={self.rate}, device={self.device_index}")
        except OSError as e:
            print(f"[ERROR] Failed to start audio stream: {e}")
            if "Invalid sample rate" in str(e):
                print(f"[SUGGESTION] 尝试检查你的麦克风是否支持该采样率，或者换成 44100Hz 或 48000Hz {self.rate}")
            # 可选：记录错误日志或触发重试逻辑
        except Exception as e:
            print(f"[FATAL] Unexpected error while starting audio stream: {e}")
            # 可根据需要决定是否重新抛出异常或退出


    def stop(self):
        """停止监听并释放资源"""
        if self._stream:
            self._stream.stop_stream()
            self._stream.close()
        self._pa.terminate()
        self._stop_flag.set()

    def get_buffered_data(self, seconds: int) -> bytes:
        """返回最近指定秒数的原始音频数据"""
        num = int(self.rate / self.chunk_size * seconds)
        frames = list(self._buffer)[-num:]
        return b"".join(frames)
    
    def get_sampwidth(self):
        return self._pa.get_sample_size(pyaudio.paInt16)

    async def record(self, max_seconds: int = None) -> str:
        """
        会话模式录音，基于静音检测提前结束。
        返回临时 WAV 文件路径。
        """
        rate, chunk = self.rate, self.chunk_size
        silent = 0
        frames = []
        max_frames = int(rate / chunk * (max_seconds or RECORD_SECONDS_AFTER_WAKE))
        last_frame = None  # 防止重复帧（可选）
        for _ in range(max_frames):
            if not self._buffer:
                await asyncio.sleep(chunk / rate)
                continue
            # 获取最近一帧，1024 / 16000 * 1000 = 64 ms
            frame = self._buffer[-1]
            if frame == last_frame:
                await asyncio.sleep(chunk / rate)
                continue
            last_frame = frame
            
            # 过滤静音帧
            if self.is_silence(frame):
                silent += 1
                if silent >= SILENT_CHUNKS_NEEDED:
                    break
            else:
                silent = 0
                frames.append(frame)
            await asyncio.sleep(chunk / rate)

        if not frames:
            log.debug("用户没有输入")
            return None

        audio_byte = await pcm_to_wav(frames,channels=self.channels,rate=rate,
                                     sampwidth=self._pa.get_sample_size(pyaudio.paInt16))
        return audio_byte
    
    def is_silence(self,frame: bytes):
        if isinstance(frame,list):
            frame = b"".join(frame)
        
        # 先计算RMS能量，过低直接认为静音
        rms = audioop.rms(frame, 2)  # 2 bytes per sample (16bit PCM)
        if rms < SILENCE_THRESHOLD:
            return True
        
        # webrtcvad 仅支持 10/20/30ms 长度的帧
        # 例如 16000Hz，10ms = 160 samples = 320 bytes
        # 细分成30ms小帧用VAD判定
        # vad_frame_bytes = int(self.rate * 0.03) * 2  # 320 bytes for 10ms@16kHz
        num_subframes = len(frame) // self.vad_frame_bytes
        print(f"30ms:{self.vad_frame_bytes/AUDIO_CHUNK_SIZE},frame:{len(frame)/AUDIO_CHUNK_SIZE}")
        for i in range(num_subframes):
            subframe = frame[i * self.vad_frame_bytes:(i + 1) * self.vad_frame_bytes]
            if self.vad.is_speech(subframe, sample_rate=self.rate):
                return False  # 有语音，不是静音
        return True  # 所有子帧都没语音，判定静音         