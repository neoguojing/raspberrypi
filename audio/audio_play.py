from pygame import mixer # 用于播放提示音和TTS生成的语音
import os
import time

mixer.init() # 初始化pygame mixer

def play_sound(sound_path):
    """播放指定的提示音"""
    if os.path.exists(sound_path):
        try:
            sound = mixer.Sound(sound_path)
            sound.play()
            while mixer.get_busy(): # 等待播放完成
                time.sleep(0.1)
        except Exception as e:
            print(f"播放声音 {sound_path} 失败: {e}")
    else:
        print(f"提示音文件 {sound_path} 未找到。")

    