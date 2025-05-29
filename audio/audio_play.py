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

async def play_audio_bytes(source: str):
    # 通过 BytesIO 读取音频数据（假设是 WAV 格式）
    bio, _ = await load_wav_data(source)
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

    def open_stream(self, format=None, channels=None, rate=None):
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
        self.close_stream()
        if self.pyaudio_instance is not None:
            self.pyaudio_instance.terminate()
            self.pyaudio_instance = None

    async def play_pcm_stream(self, url):
        self.open_stream()
        with requests.get(url, stream=True) as resp:
            resp.raise_for_status()
            for chunk in resp.iter_content(chunk_size=self.chunk_size * 2):
                if chunk:
                    self.stream.write(chunk)

    async def play(self, source: str):
        # 通过 BytesIO 读取音频数据（假设是 WAV 格式）
        bio, _ = await load_wav_data(source)
        audio = AudioSegment.from_file(bio, format="wav")
        self.open_stream()
        # 播放音频数据
        self.stream.write(audio.raw_data)

    def __del__(self):
        self.terminate()


# if __name__ == '__main__':
#     player = AudioPlayer()
#     try:
#         # 播放本地音频文件（wav/mp3等）
#         player.play("test.wav")

#         # 播放PCM流
#         # player.play_pcm_stream('http://localhost:5000/audio_stream')

#         # 播放内存PCM数据
#         # with open("sample.pcm", "rb") as f:
#         #     pcm_data = f.read()
#         # player.play_pcm_data(pcm_data)

#     finally:
#         player.terminate()

