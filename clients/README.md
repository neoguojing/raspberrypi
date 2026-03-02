# clients 模块说明

`clients/` 封装外部 AI 服务调用，避免业务层直接耦合 SDK 细节。

## 文件结构

- `agi.py`：`OpenAIClient`，包含：
  - 音频唤醒词识别（Whisper 转写）
  - 音频发送到 LLM（流式/非流式）

## 环境变量（建议）

- `OPENAI_API_KEY`：服务访问密钥。
- 其余服务地址通常在 `tasks/config.py` 内集中配置。
