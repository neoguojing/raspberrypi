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

## 操作手册（基于 Makefile）

> 以下命令默认在仓库根目录执行：`/workspace/raspberrypi`

### 1) Python 环境与语音助手

```bash
# 安装系统依赖（需要 sudo）
make init

# 创建虚拟环境并安装 Python 依赖
make setup

# 启动语音交互主程序
make run
```

常用辅助命令：

```bash
# 仅创建虚拟环境
make venv

# 仅安装 requirements
make requirements

# 清理虚拟环境与缓存
make clean
```

### 2) Docker 镜像与容器

```bash
# 构建 robot 主镜像
make image

# 构建 ORB-SLAM3 镜像
make slam3

# 构建 YOLO 分割镜像
make yolo

# 构建 SegFormer 镜像
make seg
```

运行方式：

```bash
# 启动运行时容器（后台）
make runtime

# 启动开发容器（挂载当前工程目录）
make dev

# 使用 docker compose 启动/停止整套服务
make up
make down
```

### 3) ROS2 工作流

```bash
# 安装 ROS2 依赖
make ros2_install

# 清理并编译 robot 包
make ros2_build
```

启动不同模式：

```bash
# 机器人系统
make ros2_robot

# 算法系统（SLAM/感知）
make ros2_algo

# 仿真全流程
make ros2_sim

# 离线流程
make ros2_offline

# 生产全流程
make ros2_full
```

其他 ROS2 工具：

```bash
# 清理 colcon 产物
make ros2_clean

# 键盘遥控
make ros2_ctl

# 查看 Nav2 生命周期节点状态
make status

# 生成 TF 树
make tf
```

### 4) 仿真、感知与数据录制

```bash
# 启动 Gazebo 场景（tugbot_depot）
make sim

# 启动 yolo 分割脚本
make yolo_run

# 录制 / 回放 rosbag
make record
make play
```

### 5) 模型导出与 TensorRT

```bash
# 导出 ONNX
make onnx

# 生成 TensorRT engine
make trt
```

### 6) 自查

```bash
# 查看 Makefile 提供的帮助信息
make help
```

如需 ROS2 / Gazebo 相关更详细说明，请结合 `robot/README.md` 一起使用。
