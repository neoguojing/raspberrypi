# raspberrypi

树莓派端语音交互 + 视觉感知 + ROS2 机器人实验仓库。

## 模块概览

- `audio/`：录音、播放、静音检测与音频控制。
- `tasks/`：任务编排与语音助手主流程。
- `clients/`：对接 OpenAI/AGI 的异步客户端。
- `tools/`：设备检测与音频工具函数。
- `robot/`：ROS2 机器人包、launch 与配置。
- `yolo/`：视觉分割与伪激光扫描实验脚本。
- `world/`：Gazebo 仿真场景文件。

## 快速开始

```bash
pip install -r requirements.txt
python cli.py chat
```

如需 ROS2 / Gazebo 相关能力，请先完成 `robot/README` 中的环境准备。
