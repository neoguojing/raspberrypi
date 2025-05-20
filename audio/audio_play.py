import os
import time
import base64
import asyncio
import aiohttp
import tempfile
from pygame import mixer

# 初始化 pygame mixer（只需初始化一次）
mixer.init()

async def _wait_for_playback_end():
    """等待当前音频播放完成（非阻塞等待）"""
    while mixer.get_busy():
        await asyncio.sleep(0.1)

async def _download_audio(url: str) -> str:
    """下载远程音频并保存为临时文件，返回路径"""
    async with aiohttp.ClientSession() as session:
        async with session.get(url) as response:
            if response.status != 200:
                raise Exception(f"音频下载失败: {response.status}")
            data = await response.read()
    
    tmp_file = tempfile.NamedTemporaryFile(delete=False, suffix=".wav")
    tmp_file.write(data)
    tmp_file.close()
    return tmp_file.name

async def _decode_base64_audio(data: str) -> str:
    """将 base64 音频数据解码为临时文件"""
    if data.startswith("data:audio"):
        header, data = data.split(",", 1)
    audio_bytes = base64.b64decode(data)
    tmp_file = tempfile.NamedTemporaryFile(delete=False, suffix=".wav")
    tmp_file.write(audio_bytes)
    tmp_file.close()
    return tmp_file.name

async def play_sound(source: str):
    """
    异步播放音频（支持本地文件、远程URL、base64）
    
    参数:
        source: 可以是本地路径、远程URL、或base64音频字符串
    """
    temp_path = None
    try:
        if source.startswith("http://") or source.startswith("https://"):
            temp_path = await _download_audio(source)
            sound = mixer.Sound(temp_path)
        elif source.strip().startswith("data:audio") or len(source.strip()) > 100:
            temp_path = await _decode_base64_audio(source)
            sound = mixer.Sound(temp_path)
        elif os.path.exists(source):
            sound = mixer.Sound(source)
        else:
            print(f"无法识别或找到音频源: {source}")
            return
        
        sound.play()
        await _wait_for_playback_end()
    except Exception as e:
        print(f"播放音频失败: {e}")
    finally:
        if temp_path and os.path.exists(temp_path):
            os.remove(temp_path)
