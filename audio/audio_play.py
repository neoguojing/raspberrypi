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

# PCM参数要和服务端匹配
FORMAT = pyaudio.paInt16
CHANNELS = 1
RATE = 24000
CHUNK_SIZE = 1024  # 这里要和服务端 chunk_size 保持一致

def play_pcm_stream(url):
    p = pyaudio.PyAudio()
    stream = p.open(format=FORMAT,
                    channels=CHANNELS,
                    rate=RATE,
                    output=True,
                    frames_per_buffer=CHUNK_SIZE)

    with requests.get(url, stream=True) as r:
        r.raise_for_status()
        for chunk in r.iter_content(chunk_size=CHUNK_SIZE * 2):  # 16-bit=2字节
            if chunk:
                stream.write(chunk)

    stream.stop_stream()
    stream.close()
    p.terminate()

if __name__ == '__main__':
    play_pcm_stream('http://localhost:5000/audio_stream')
