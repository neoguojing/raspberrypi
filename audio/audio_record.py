import os
import tempfile
import threading
import collections

import pyaudio
import audioop
from typing import Deque
import asyncio
import time

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

    def _audio_callback(self, in_data, frame_count, time_info, status):
        # 写入环形缓冲
        self._buffer.append(in_data)
        return (None, pyaudio.paContinue)

    def start(self):
        """启动持续监听音频流"""
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

    async def record(self, max_seconds: int = None) -> str:
        """
        会话模式录音，基于静音检测提前结束。
        返回临时 WAV 文件路径。
        """
        rate, chunk = self.rate, self.chunk_size
        silent = 0
        frames = []
        max_frames = int(rate / chunk * (max_seconds or RECORD_SECONDS_AFTER_WAKE))

        for _ in range(max_frames):
            # 获取最近一帧
            frame = self._buffer[-1] if self._buffer else b"\x00" * (chunk * 2)
            frames.append(frame)
            # 静音帧数超过阈值，则判断用户输入完毕
            if self.is_silence(frame):
                silent += 1
                if silent >= SILENT_CHUNKS_NEEDED:
                    break
            else:
                silent = 0
            await asyncio.sleep(chunk / rate)

        if self.is_silence(frames):
            print("用户没有输入")
            return None

        audio_byte = pcm_to_wav(frames,channels=self.channels,rate=rate,
                                     sampwidth=self._pa.get_sample_size(pyaudio.paInt16))
        return audio_byte
    
    def is_silence(self,frames):
        if isinstance(frames,list):
            frames = b"".join(frames)
        return audioop.rms(frames, 2) < SILENCE_THRESHOLD