import base64
import asyncio
import io
import wave
import os
import tempfile
import time
import aiohttp
import audioop
import traceback

async def audio_to_base64(source):
    bio = source
    if not isinstance(source,io.BytesIO):
        bio,_ = await load_wav_data(source)
    encoded_audio = None
    if bio:
        encoded_audio = base64.b64encode(bio.getvalue()).decode('utf-8')
        encoded_audio = f"data:audio/x-wav;base64,{encoded_audio}"
    return encoded_audio

async def decode_base64_audio(data: str) -> str:
    """将 base64 音频数据解码为临时文件"""
    if data.startswith("data:audio"):
        header, data = data.split(",", 1)
    audio_bytes = base64.b64decode(data)
    return audio_bytes
    
async def download_audio(url: str) -> str:
    """下载远程音频并保存为临时文件，返回路径"""
    async with aiohttp.ClientSession() as session:
        async with session.get(url) as response:
            if response.status != 200:
                raise Exception(f"音频下载失败: {response.status}")
            data = await response.read()
    return data

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

async def pcm_to_wav_bytes(pcm_data: bytes, channels: int = 1, rate: int = 16000, sampwidth: int = 2) -> io.BytesIO:
    """将原始 PCM 音频数据封装为内存中的 WAV 格式"""
    wav_io = io.BytesIO()
    with wave.open(wav_io, 'wb') as wf:
        wf.setnchannels(channels)
        wf.setsampwidth(sampwidth)
        wf.setframerate(rate)
        wf.writeframes(pcm_data)
    wav_io.seek(0)
    return wav_io

async def pcm_to_wav_file(pcm_data: bytes, channels: int = 1, rate: int = 16000, sampwidth: int = 2) -> str:
    path = os.path.join(tempfile.gettempdir(), f"wake_{int(time.time())}.wav")
    with wave.open(path, 'wb') as wf:
        wf.setnchannels(channels)
        wf.setsampwidth(sampwidth)
        wf.setframerate(rate)
        wf.writeframes(pcm_data)
    return path

async def pcm_to_wav(pcm_data: bytes, channels: int = 1, rate: int = 16000, sampwidth: int = 2,save_to_file=False):
    if isinstance(pcm_data,list):
        pcm_data = b"".join(pcm_data)
        
    ret = None
    if save_to_file:
        ret = await pcm_to_wav_file(pcm_data,channels,rate,sampwidth)
    else:
        ret = await pcm_to_wav_bytes(pcm_data,channels,rate,sampwidth)
        
    return ret

# 将wave源数据放入本地内存
async def load_wav_data(source:str,save_to_file=False):
    audio_data = None
    file_path = None
    is_file = True
    try:
        if source.startswith("http://") or source.startswith("https://"):
            audio_data = await download_audio(source)
            is_file = False
        elif source.strip().startswith("data:audio") or len(source.strip()) > 100:
            audio_data = await decode_base64_audio(source)
            is_file = False
        elif os.path.exists(source):
            with open(source, 'rb') as f:
                audio_data = f.read()
            file_path = source
        
        if save_to_file and not is_file:
            file_path = save_to_tmp(audio_data)
            
        return io.BytesIO(audio_data),file_path
    except Exception as e:
        print(f"Error during load_wav_data: {e}")
        print(traceback.format_exc())
        return None,""
    
        

async def wav_to_bytesio(file_path: str) -> io.BytesIO:
    """将WAV文件内容写入内存字节流
    
    Args:
        file_path: WAV文件路径
        
    Returns:
        包含完整WAV数据的BytesIO对象
    """
    with wave.open(file_path, 'rb') as wav:
        params = wav.getparams()
        frames = wav.readframes(params.nframes)
    byte_io = io.BytesIO()
    with wave.open(byte_io, 'wb') as mem_wav:
        mem_wav.setparams(params)
        mem_wav.writeframes(frames)
    byte_io.seek(0)
    return byte_io
    
async def save_to_tmp(audio_bytes):
    tmp_file = tempfile.NamedTemporaryFile(delete=False, suffix=".wav")
    tmp_file.write(audio_bytes)
    tmp_file.close()
    return tmp_file.name

def calculate_rms_with_audioop(wav_file):
    # 打开 WAV 文件
    with wave.open(wav_file, 'rb') as wf:
        # 读取音频文件的参数
        sample_width = wf.getsampwidth()  # 每个样本的字节数
        num_channels = wf.getnchannels()  # 声道数
        frame_rate = wf.getframerate()  # 采样率
        num_frames = wf.getnframes()  # 总帧数
        
        # 读取音频样本数据
        audio_data = wf.readframes(num_frames)

        # 计算 RMS 值
        rms_value = audioop.rms(audio_data, sample_width)
    
    return rms_value
