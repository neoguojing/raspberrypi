# tasks 模块说明

`tasks/` 用于定义可执行任务及其运行生命周期。

## 文件结构

- `base.py`：任务基类（统一 `run/stop` 语义）。
- `tasks.py`：任务注册表（`TASKS`）。
- `audio_chat_app.py`：语音助手主任务（唤醒、录音、调用 LLM、播报）。
- `config.py`：任务级配置（唤醒检测窗口、接口地址等）。

## 当前默认任务

- `chat`：通过 `cli.py` 启动的语音助手任务。

## 启动方式

```bash
python cli.py chat
```
