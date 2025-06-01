import os
import time
import base64
import asyncio
import tempfile
from pydub import AudioSegment
import pyaudio
import io
from tools.utils import load_wav_data
import requests
import aiohttp
import logging
import re

log = logging.getLogger(__name__)
END_TAG = b'\x00' * 8192
async def play_audio_bytes(source: str):
    # 通过 BytesIO 读取音频数据（假设是 WAV 格式）
    bio, _ = await load_wav_data(source)
    if bio is None:
        return
    audio = AudioSegment.from_file(bio, format="wav")

    p = pyaudio.PyAudio()
    stream = p.open(format=p.get_format_from_width(audio.sample_width),
                    channels=audio.channels,
                    rate=audio.frame_rate,
                    output=True)

    # 播放音频数据
    stream.write(audio.raw_data)

    stream.stop_stream()
    stream.close()
    p.terminate()

class AudioPlayer:
    def __init__(self, format=pyaudio.paInt16, channels=1, rate=24000, chunk_size=480):
        self.format = format
        self.channels = channels
        self.rate = rate
        self.chunk_size = chunk_size

        self.pyaudio_instance = pyaudio.PyAudio()
        self.stream = None
        self.running = True
        
        self.queue = asyncio.Queue(maxsize=1000)  # 防止爆内存

    def open_stream(self, format=None, channels=None, rate=None,):
        if self.stream is None:
            fmt = format if format is not None else self.format
            ch = channels if channels is not None else self.channels
            rt = rate if rate is not None else self.rate
            self.stream = self.pyaudio_instance.open(
                format=fmt,
                channels=ch,
                rate=rt,
                output=True,
                frames_per_buffer=self.chunk_size*2
            )

    def close_stream(self):
        if self.stream is not None:
            self.stream.stop_stream()
            self.stream.close()
            self.stream = None

    def terminate(self):
        self.running = False
        self.close_stream()
        if self.pyaudio_instance is not None:
            self.pyaudio_instance.terminate()
            self.pyaudio_instance = None
        
    async def play(self):
        """从队列读取 PCM 数据并播放，跟随 PyAudio 播放节奏并记录实际间隔偏差"""
        self.open_stream()
        frame_duration = self.chunk_size / self.rate  # e.g., 320 / 16000 = 0.02s
        last_play_time = time.time()

        while self.running:
            try:
                # 从队列中取出一帧
                wait_start = time.time()
                frame = await self.queue.get()
                wait_end = time.time()
                wait_time = wait_end - wait_start

                if wait_time > frame_duration * 2:
                    log.warning(f"[客户端] 播放等待时间过长 {wait_time:.4f}s，队列可能断流，重置计时参考")
                    last_play_time = time.time()  # 重置时间基准
                    continue  # 跳过本帧播放，防止节奏错误

                # 播放音频帧（阻塞，PyAudio 自控播放速率）
                self.stream.write(frame)

                # 计算实际播放间隔和漂移
                now = time.time()
                play_interval = now - last_play_time
                drift = play_interval - frame_duration
                last_play_time = now

                if abs(drift) > 0.005:
                    log.warning(f"[客户端] 播放一帧 {len(frame)} bytes，间隔偏差 {drift:+.4f}s")

            except Exception as e:
                log.error(f"[客户端] 播放异常: {e}", exc_info=True)
                break


    async def producer(self, url, retry_delay=5):
        while self.running:
            try:
                log.info(f"Connecting to {url} for PCM streaming...")
                async with aiohttp.ClientSession() as session:
                    async with session.get(url) as resp:
                        if resp.status != 200:
                            raise Exception(f"HTTP error: {resp.status}")
                        
                        # ✅ 解析 Content-Type
                        content_type = resp.headers.get('Content-Type', '')
                        match = re.search(r"rate=(\d+);.*channels=(\d+)", content_type)
                        if match:
                            self.rate = int(match.group(1))
                            self.channels  = int(match.group(2))
                            if self.rate == 16000:
                                self.chunk_size = 320
                        else:
                            self.rate  = 24000
                            self.channels = 1
                            self.chunk_size = 480
                        
                        print(f"[客户端] 采样率: {self.rate}, 通道数: {self.channels}")
                        
                        buffer = b''
                        while self.running:
                            chunk = await resp.content.read(self.chunk_size * 2)
                            if not chunk:
                                log.warning("empty data")
                                time.sleep(0.1)
                                continue
                            
                            if chunk == END_TAG:
                                log.warning("No more data. Stream ended.")
                                continue
                            
                            # 保证帧对其
                            buffer += chunk
                            while len(buffer) >= self.chunk_size * 2:
                                frame = buffer[:self.chunk_size * 2]
                                buffer = buffer[self.chunk_size * 2:]
                                try:
                                    await self.queue.put(frame)
                                except asyncio.QueueFull:
                                    log.warning("Playback queue full, dropping frame")

            except Exception as e:
                log.error(f"Producer: {e}")

            if self.running:
                log.info(f"Retrying connection in {retry_delay} seconds...")
                await asyncio.sleep(retry_delay)
            
    def __del__(self):
        self.terminate()


