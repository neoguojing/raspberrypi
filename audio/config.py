# config.py

import os
from dotenv import load_dotenv

load_dotenv(override=True)  # 加载 .env 文件中的环境变量
# Picovoice Porcupine AccessKey (从 Picovoice Console 获取)
PICOVOICE_ACCESS_KEY = "YOUR_PICOVOICE_ACCESS_KEY"

# Porcupine 模型文件路径
PORCUPINE_LIBRARY_PATH = None # SDK会自动选择合适的库，一般无需设置
PORCUPINE_MODEL_PATH = "porcupine_models/porcupine_params.pv"

# 唤醒词 .ppn 文件路径 (可以有多个)
# 你可以从 Picovoice Console 下载预置的，或训练自己的
PORCUPINE_KEYWORD_PATHS = ["porcupine_models/raspberry-pi_raspberry-pi.ppn"] # 示例："Raspberry Pi"
PORCUPINE_SENSITIVITIES = [0.5] # 对应每个关键词的灵敏度 (0.0 to 1.0)

# 录音参数
AUDIO_FORMAT = "wav" # 或者 "flac" 等，取决于你的API
AUDIO_CHANNELS = 1
AUDIO_RATE = 16000 # 采样率，Hz
AUDIO_CHUNK_SIZE = 1024 # 每次读取的帧数
AUDIO_DEVICE_INDEX = None # 麦克风设备索引，None代表默认。运行 list_audio_devices.py 查找。
RECORD_SECONDS_AFTER_WAKE = 10 # 唤醒后最长录音时间
SILENCE_THRESHOLD = 500 # 静音阈值 (需要根据麦克风和环境调整)
SILENT_CHUNKS_NEEDED = 30 # 连续多少个静音块后判断为静音 (RATE / CHUNK_SIZE * seconds) e.g., 16000/1024 * 1.5 = ~

# 远端LLM API
REMOTE_LLM_URL = "YOUR_REMOTE_LLM_API_ENDPOINT" # 例如 "http://your-server.com/process-audio"
# 如果你的API需要认证，可以在headers中添加
REMOTE_LLM_HEADERS = {
    # "Authorization": "Bearer YOUR_API_TOKEN"
}

# TTS 语言 (用于 gTTS)
TTS_LANG = 'zh-cn' # 中文
