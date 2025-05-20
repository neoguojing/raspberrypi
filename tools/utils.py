import base64
import asyncio
import io
import wave

def audio_to_base64(audio_path):
    
    with open(audio_path, "rb") as audio_file:
        encoded_audio = base64.b64encode(audio_file.read()).decode('utf-8')
    return encoded_audio

class AsyncSafeValue:
    def __init__(self, initial=0):
        self._value = initial
        self._lock = asyncio.Lock()

    async def set(self, value):
        async with self._lock:
            self._value = value

    async def get(self):
        async with self._lock:
            return self._value

def pcm_to_wav_bytes(pcm_data: bytes, channels: int, rate: int, sampwidth: int = 2) -> bytes:
    """将原始 PCM 音频数据封装为内存中的 WAV 格式"""
    wav_io = io.BytesIO()
    with wave.open(wav_io, 'wb') as wf:
        wf.setnchannels(channels)
        wf.setsampwidth(sampwidth)
        wf.setframerate(rate)
        wf.writeframes(pcm_data)
    return wav_io.getvalue()