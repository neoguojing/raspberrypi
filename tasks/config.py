import os
# 提示音 (可选)
WAKE_SOUND_PATH = "asset/wakeup.wav" # 唤醒成功提示音
WAKE_CHECK_SECONDS = 3  #唤醒检测截取的音频的时间长度，从缓存截取
WAKE_CHECK_STEP = 3 #唤醒检测的周期
POST_WAKE_COOLDOWN = 180 #唤醒后激活的保持时间 秒


# voice_chat 直接和语音模型对话，否则speech to text -> llm -> tts
AUDIO_FEATURE_TYPE = os.getenv("AUDIO_FEATURE_TYPE", "voice_chat")
# AUDIO_FEATURE_TYPE = os.getenv("AUDIO_FEATURE_TYPE", "")
