import os
import time
import base64
import asyncio

import tempfile
# from pygame import mixer
from pydub import AudioSegment
import pyaudio
import io
from tools.utils import load_wav_data

# 初始化 pygame mixer（只需初始化一次）
# mixer.quit() 
# # mixer.init()
# mixer.init(devicename="bluez_sink.58_EA_1F_26_88_47.a2dp_sink")

# async def _wait_for_playback_end():
#     """等待当前音频播放完成（非阻塞等待）"""
#     while mixer.get_busy():
#         await asyncio.sleep(0.1)

# async def play_sound(source: str):
#     """
#     异步播放音频（支持本地文件、远程URL、base64）
    
#     参数:
#         source: 可以是本地路径、远程URL、或base64音频字符串
#     """
#     mixer.init()
#     try:     
#         _, temp_path = load_wav_data(source,save_to_file=True)
#         sound = mixer.Sound(temp_path)
#         sound.play()
#         await _wait_for_playback_end()
#     except Exception as e:
#         print(f"播放音频失败: {e}")
#     finally:
#         if temp_path and os.path.exists(temp_path):
#             os.remove(temp_path)
#         mixer.quit() 

async def play_audio_bytes(source: str):
    # 通过 BytesIO 读取音频数据（假设是 WAV 格式）
    bio, _ = load_wav_data(source)
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

    