import time
import asyncio
from pydub import AudioSegment
import pyaudio
from tools.utils import load_wav_data
import aiohttp
import logging
import re
import json

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

class PIDController:
    def __init__(self, kp, ki, kd, setpoint):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.setpoint = setpoint  # 目标间隔时间，比如 0.02 秒
        self.prev_error = 0
        self.integral = 0
        self.last_time = time.time()

    def compute(self, current_time):
        dt = current_time - self.last_time
        if dt <= 0:
            return 0
        
        error = self.setpoint - dt
        self.integral += error * dt
        derivative = (error - self.prev_error) / dt

        output = self.kp * error + self.ki * self.integral + self.kd * derivative

        self.prev_error = error
        self.last_time = current_time

        return output


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
        self.session = aiohttp.ClientSession()

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
        if self.session:
            self.session.close()
        self.close_stream()
        if self.pyaudio_instance is not None:
            self.pyaudio_instance.terminate()
            self.pyaudio_instance = None
        
    async def play(self):
        """使用 PID 控制播放节奏，处理 await 阻塞的影响"""
        self.open_stream()

        while self.running:
            try:
                frame = await self.queue.get()  # 这可能会阻塞

                self.stream.write(frame)

            except Exception as e:
                log.error(f"播放错误: {e}")
                break
            
    async def producer(self, url, retry_delay=5):
        while self.running:
            try:
                log.info(f"Connecting to {url} for PCM streaming...")
                async with self.session as session:
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
    
    async def producer_ws(self,ws_url:str, retry_delay=5):

        while self.running:
            try:
                logging.info(f"Connecting to {ws_url}")
                async with self.session.ws_connect(ws_url) as ws:
                    # 首次连接时发送音频参数请求
                    await ws.send_json({
                        "type": "config_request",
                        "supported_rates": [16000, 24000],
                        "supported_channels": [1, 2]
                    })
                    
                    async for msg in ws:
                        if msg.type == aiohttp.WSMsgType.TEXT:
                            try:
                                config = json.loads(msg.data)
                                if config.get("type") == "config":
                                    self.rate = config.get("rate", 16000)
                                    self.channels = config.get("channels", 1)
                                    self.chunk_size = 320 if self.rate == 16000 else 480
                                    log.info(f"Audio config: rate={self.rate}, channels={self.channels}")
                            except Exception as e:
                                log.warning(f"Failed to parse config: {e}")
                                
                        elif msg.type == aiohttp.WSMsgType.BINARY:
                            await self.queue.put(msg.data)

                        elif msg.type == aiohttp.WSMsgType.ERROR:
                            asyncio.sleep(retry_delay)
                            
            except Exception as e:
                logging.error(f"WebSocket error: {e}")
                if self.running:
                    logging.info(f"Retrying in {retry_delay} seconds...")
                    await asyncio.sleep(retry_delay)
                        
    def __del__(self):
        self.terminate()


