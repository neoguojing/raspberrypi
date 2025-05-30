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

log = logging.getLogger(__name__)

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
    def __init__(self, format=pyaudio.paInt16, channels=1, rate=24000, chunk_size=1024):
        self.format = format
        self.channels = channels
        self.rate = rate
        self.chunk_size = chunk_size

        self.pyaudio_instance = pyaudio.PyAudio()
        self.stream = None
        self.running = False

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
                frames_per_buffer=self.chunk_size
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

    async def play(self, url, retry_delay=5):
        self.running = True
        self.open_stream()

        while self.running:
            try:
                log.info(f"Connecting to {url} for PCM streaming...")
                async with aiohttp.ClientSession() as session:
                    async with session.get(url) as resp:
                        if resp.status != 200:
                            raise Exception(f"HTTP error: {resp.status}")

                        while self.running:
                            chunk = await resp.content.read(self.chunk_size * 2)
                            if not chunk:
                                log.warning("No more data. Stream ended.")
                                time.sleep(0.1)
                                continue
                            print(f"play:{len(chunk)}")
                            self.stream.write(chunk)

            except Exception as e:
                log.error(f"Error in PCM stream: {e}")

            if self.running:
                log.info(f"Retrying connection in {retry_delay} seconds...")
                await asyncio.sleep(retry_delay)

        self.close_stream()

    def __del__(self):
        self.terminate()


