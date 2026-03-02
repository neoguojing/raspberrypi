# audio 模块说明

`audio/` 负责本地音频采集、播放与基础信号判断，是语音助手的底层 I/O 层。

## 文件结构

- `audio_record.py`：持续监听与缓存录音数据。
- `audio_play.py`：音频播放、播放队列与 TTS 音频输出。
- `audio_controller.py`：统一管理录音器和播放器（异步接口）。
- `config.py`：音频相关默认配置。

## 主要能力

1. 持续录音并支持按时间窗口回溯（用于唤醒词检测）。
2. 在播放阶段避免抢占录音，减少回声干扰。
3. 提供静音判断能力，减少无效识别请求。

## 被谁使用

- `tasks/audio_chat_app.py` 会通过 `AudioControllerAsync` 调用本模块。
