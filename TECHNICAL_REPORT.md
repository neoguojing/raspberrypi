# Raspberry Pi 智能移动机器人项目技术报告

> **文档范围**：基于当前仓库代码与配置的静态梳理；描述的是已实现或已编排的能力，运行时实际启用的节点仍以启动参数、硬件接入和容器镜像为准。  
> **更新时间**：2026-09-18

## 1. 项目定位与目标

本仓库是面向树莓派端的实验型智能机器人系统，融合两条相对独立、可在同一设备协作的主线：

1. **语音助手线**：持续采集麦克风音频，进行静音过滤和云端转写唤醒；唤醒后录制一轮语音、调用 OpenAI 兼容服务，并通过 WebSocket 接收 TTS 音频流播放。
2. **ROS 2 机器人线**：提供四轮小车控制、树莓派相机发布、ICM-20948 IMU 读取、视觉语义分割伪激光、定位建图、Nav2 导航以及 Gazebo 仿真。

仓库采用 Python 为主要实现语言，ROS 2 Jazzy 为机器人中间件，Docker Compose 将 ROS 2 与推理服务拆分为容器。视觉路径支持 RTAB-Map、ORB-SLAM3 和 slam_toolbox 三种 SLAM 后端，并为单目、双目和伪激光三种传感器模式提供启动编排。

## 2. 仓库结构与职责边界

| 路径 | 职责 | 关键内容 |
|---|---|---|
| `cli.py`、`tasks/` | 语音任务入口与编排 | `chat` 任务、唤醒状态机、环境变量配置 |
| `audio/` | 本地音频 I/O | 连续录音环形缓存、VAD/静音判断、TTS 队列播放 |
| `clients/` | 云端服务适配 | 基于 `AsyncOpenAI` 的转写、对话流接口 |
| `tools/` | 共用工具 | 音频转换、设备检查等辅助能力 |
| `robot/` | ROS 2 ament Python 包 | 节点、launch、传感器/导航/SLAM 配置、Dockerfile |
| `yolo/` | 视觉感知实验 | Zenoh 图像订阅、SegFormer TensorRT 推理、伪 `LaserScan` 生成 |
| `world/` | 仿真资产 | Gazebo SDF 场景 |
| `asset/` | 运行时资源 | 唤醒提示音、测试媒体 |
| `Makefile`、`docker-compose.yml` | 开发、构建与部署入口 | Python 环境、ROS 构建、镜像构建、容器编排 |

建议将根目录视为 ROS 2 工作空间根目录：`robot/` 是唯一由 `colcon build --packages-select robot` 构建的包；语音助手则使用根目录 `.venv` 和 `python cli.py chat` 运行。二者目前没有代码级命令桥接，协作主要发生在同一主机与运行环境层。

## 3. 总体架构

```text
                         ┌──────────────── 语音交互子系统 ────────────────┐
USB 麦克风 ──> PyAudio ─>│ 环形缓冲 / RMS + WebRTC VAD                    │
                         │    └─> Whisper（唤醒词“小派”）                 │
                         │           └─> 录音 WAV/Base64                 │
                         │                 └─> OpenAI 兼容 LLM 服务      │
                         │                         └─> TTS WebSocket      │
扬声器 <─────────────────│<── PyAudio 播放队列                            │
                         └───────────────────────────────────────────────┘

                         ┌──────────────── ROS 2 机器人子系统 ───────────┐
相机 ─> /camera/image_raw[(/compressed)] ─> 图像处理 ─> SLAM / 感知       │
IMU  ─> /imu/data_raw ───────────────────────────────> EKF               │
视觉分割 ─> Zenoh rt/scan ─> seg_to_scan_node ─> /seg/scan               │
轮速/视觉里程计 ─────────────────────────────────────> EKF -> /ekf/odom  │
RTAB-Map / ORB-SLAM3 / slam_toolbox ───────────────────> /map、TF        │
Nav2 ─> /cmd_vel ─> car_driver_node ─> GPIO 电机 + 舵机                  │
                         └───────────────────────────────────────────────┘
```

### 3.1 分层说明

- **硬件与驱动层**：相机节点可使用树莓派相机驱动或视频文件；IMU 节点通过 SPI0/CE0 访问 ICM-20948；车辆驱动将 `Twist` 映射到四个电机和一个转向舵机。
- **消息与传输层**：机器人内部使用 ROS 2/DDS；重型图像分割使用 Zenoh 的 `rt/...` 键空间，桥接节点将 JSON 形式的扫描结果恢复为 ROS 2 `sensor_msgs/LaserScan`。容器采用 host network，适合本机 DDS 发现与设备直通。
- **感知与状态估计层**：`image_proc`/`stereo_image_proc` 可做图像校正，分割路径把地面接触点投影到车体坐标系并合成二维扫描；EKF 在二维模式下融合轮速、视觉里程计和 IMU。
- **建图、导航与执行层**：SLAM 后端生成地图/位姿；Nav2 根据地图、扫描和里程计规划，输出 `/cmd_vel`；底盘节点最终控制 GPIO。

## 4. 语音交互子系统

### 4.1 执行链路

`cli.py` 从 `tasks.TASKS` 选择 `chat`，调用 `VoiceAssistant.run()`。助手启动时先验证输入、输出音频设备，然后并发启动播放器与唤醒检测协程。唤醒检测周期性从连续录音器的缓存提取最近 3 秒 PCM：先以 RMS 与 WebRTC VAD 排除静音，再转 WAV 调用 Whisper 兼容端点；转写文本包含“小派”后置位事件。主循环播放 `asset/wakeup.wav`，再以静音结束条件录制用户语音并将其 Base64 化，发送给 `agi-model`。

服务地址由 `.env` 覆盖，默认值如下：

| 配置 | 默认值 | 用途 |
|---|---|---|
| `AGI_URL` | `http://localhost:8000/v1` | OpenAI 兼容对话服务 |
| `AGI_TTS_URL` | `http://localhost:8002/v1` | TTS WebSocket 服务基址 |
| `AGI_WHISPER_URL` | `http://localhost:8003/v1` | Whisper 转写服务 |
| `AGI_API_KEY` | `123` | 服务认证密钥（开发默认值） |
| `AUDIO_FEATURE_TYPE` | 空字符串 | 控制普通流式/语音聊天模式 |

### 4.2 并发与资源管理

- `ContinuousAudioListener` 使用 PyAudio callback 持续向固定长度 `deque` 写入音频帧；交互录音读取最新帧，连续静音达到阈值即结束。
- `VoiceAssistant` 用 `interaction_lock` 避免唤醒检查和一轮交互同时消费音频；播放队列未清空时会延后录音以降低回声。
- TTS 播放器由 HTTP/WebSocket 生产协程和 PyAudio 消费协程组成，队列上限为 1000，避免无界内存增长。
- 临时 WAV 文件在转写/交互结束后删除；部署时仍应确保异常路径、权限和可用磁盘空间受到监控。

### 4.3 外部依赖与安全边界

Python 依赖包括 `pyaudio`、`webrtcvad`、`aiohttp`、`openai`、`pydub`、`picamera2` 和 OpenCV。音频系统依赖正确的 ALSA/PulseAudio 设备；网络语音链路依赖三个可达的 HTTP 服务。生产环境不应使用默认 API key，应通过未提交的 `.env` 或秘密管理系统注入凭据，并限制本地服务监听范围。

## 5. ROS 2 机器人子系统

### 5.1 包与可执行节点

`robot` 是 `ament_python` 包，注册的主要 ROS 2 可执行节点如下。

| 节点 | 输入 | 输出 | 功能 |
|---|---|---|---|
| `car_driver_node` | `cmd_vel` (`Twist`) | GPIO 执行 | 线速度控制四轮正反转，角速度控制舵机，零速度停车 |
| `camera_publisher_node` | 相机或视频源 | `/camera/image_raw` 或 `/camera/image_raw/compressed`、`/camera/camera_info` | 以默认 15 Hz 发布 JPEG 压缩图像或原始图像及标定信息 |
| `icm20948_spi_node` | SPI0/CE0 | `/imu/data_raw` (`Imu`) | 读取加速度/陀螺仪，换算为 SI 单位，默认 100 Hz |
| `static_tf_pub_node` | `tf_data` 参数 | `/tf_static` | 将 JSON 的欧拉角外参转换为静态 TF |
| `seg_to_scan_node` | Zenoh `rt/scan` | `/seg/scan`、`/seg/scan/local` | 将分割端 JSON 扫描桥接为 ROS 2 `LaserScan` |
| `manual_nav_commander`、`explore_node` 等 | Nav2 action/topic | 导航目标/探索控制 | 人工或自动探索辅助 |

### 5.2 实车基础启动

`robot.launch.py` 读取 `config/robot_config.json` 与 `config/imx219.json`，当前返回的启动描述**实际启用**底盘驱动和相机发布节点；静态 TF 与 ICM 节点虽已定义，但在返回列表中被注释。因此，实车联调如需 TF 与 IMU，须先核验并按需求启用相应节点，或由其他 launch 提供等价数据。

相机节点支持参数化：`camera_frequency`、`is_camera`、`source`、`compressed` 与 `camera_config`。标定 JSON 被转换为 `CameraInfo` 的 `K`、`D`、`R`、`P` 字段；压缩模式发布 JPEG 到 `/camera/image_raw/compressed`，原始模式发布 RGB8 到 `/camera/image_raw`。

### 5.3 坐标系与传感器标定

`robot_config.json` 明确采用 REP-103 右手系，定义 `base_footprint -> base_link -> {imu_link, camera_link -> camera_link_optical}`。相机外参为前向 0.1 m、高度 0.13 m、下俯约 0.1484 rad；光学坐标系额外采用 `roll=-π/2`、`yaw=-π/2`。这份 JSON 同时被静态 TF 发布器和视觉伪激光程序读取，是几何一致性的单一配置源。

目标 TF 拓扑为：

```text
map -> odom -> base_footprint -> base_link -> {imu_link, camera_link -> camera_link_optical}
```

其中 `odom -> base_footprint` 应由 EKF 等动态估计器发布，不能由静态 TF 固化。

### 5.4 定位、建图与导航

`algo.launch.py` 是算法编排入口，参数为：

- `sensor_mode`：`mono`、`stereo` 或 `laser`；默认 `stereo`。
- `slam_backend`：`orbslam3`、`rtabmap` 或 `slam_toolbox`；默认 `rtabmap`。
- `use_sim_time` 与 `compressed`：控制仿真时钟与图像传输类型。

该 launch 会根据组合条件引入图像校正、EKF、对应 SLAM 后端、视觉伪激光桥接、Nav2 和导航控制节点。`full.launch.py` 直接组合实车基础节点与算法节点；`full.sim.launch.py` 根据传感器模式选择单目/激光或双目仿真，并将 `use_sim_time=true`、`compressed=false` 传给算法链路。

EKF 配置使用 `two_d_mode: true` 和 50 Hz 更新，融合 `/wheel_odom` 的平面速度、`/visual_odom` 的平面位姿/偏航以及 `/imu/data_raw` 的角速度与平面加速度，输出坐标系配置为 `world_frame=odom`、`base_link_frame=base_footprint`。该配置决定了上游里程计话题与 TF 的可用性是导航成功的前提。

RTAB-Map 的激光模式订阅 `/seg/scan`、`/ekf/odom`、`/imu/data_raw`，以二维 ICP 配置生成 5 cm 栅格地图，最大有效扫描距离为 3 m；其参数关闭深度输入并启用射线追踪。Nav2 使用 `nav2_bringup` 的配置和独立组件容器，接收地图/里程计/扫描等标准 ROS 2 接口，输出 `/cmd_vel`。

## 6. 视觉分割伪激光链路

该链路用单目语义分割在没有物理激光雷达时构造近距离二维障碍扫描，属于实验性能力。

1. `camera_publisher_node` 发布相机图像；Zenoh ROS 2-DDS bridge 将其映射到 `rt/camera/image_raw` 或 `rt/camera/image_raw/compressed`。
2. `yolo/zen_seg.py` 每三帧保留一帧，在独立推理线程中使用 `SegFormerTRTDetector` 提取地面接触像素点。
3. 程序按相机内参和 `robot_config.json` 外参将像素批量投影至车体平面，量化距离到 2 cm；在 ±40° 视场以 0.5° 步进形成 161 束扫描，量程上限为 3 m。
4. 处理线程对同一角度保留最近障碍物、对相邻束扩散，并对连续有效值施加指数滑动平均（`alpha=0.3`）。结果携带时间戳、角度和距离，以 JSON 发布至 `rt/scan`。
5. `seg_to_scan_node` 订阅该键，将空值、过远值和过近值转换为 `inf`/`nan`，以 10 Hz 发布 `/seg/scan`，供 RTAB-Map 与 Nav2 使用。

该方案的关键假设是相机高度、俯仰角和内参准确，且分割模型可稳定识别障碍物与地面接触点。它不等价于真实激光测距：应在成本地图中设置保守膨胀半径、限制速度，并使用 rosbag 和实测障碍物对距离误差、时延和漏检率做验收。

## 7. 部署、构建与运行

### 7.1 本地 Python 工作流

```bash
make init          # 安装 PyAudio 等系统依赖（需要 sudo）
make setup         # 创建 .venv 并安装 requirements.txt
make run           # 等价于 .venv/bin/python cli.py chat
```

### 7.2 ROS 2 工作流

```bash
make ros2_install  # rosdep 安装 robot 包依赖
make ros2_build    # 清理后构建 robot 包
make ros2_robot    # 实车基础节点
make ros2_algo     # 算法链路
make ros2_full     # 实车完整链路
make ros2_sim      # 仿真完整链路
```

构建脚本假定 ROS 2 发行版为 Jazzy，且外部 ORB-SLAM3 ROS 2 工作空间位于 `/home/ros_user/orbslam3`（容器内路径）。启动算法、仿真和完整链路前，需要 source 本项目与外部 ORB-SLAM3 工作空间的 `install/setup.bash`。

### 7.3 容器与硬件访问

`docker-compose.yml` 包含 `ros2` 与 `yolo` 服务。两者使用 `network_mode: host`；ROS 2 服务还使用 privileged、host IPC、`/dev` 与 udev 挂载，并挂载 X11 以便可视化。默认镜像标签面向 Pi 5（`...:pi5`）；GPU profile 请求 GPU 设备。建议在部署前确认目标平台驱动、Docker GPU runtime、X11 安全策略及 `/dev/spidev*`、相机、音频设备的可见性。

镜像职责如下：

| 镜像入口 | 基础能力 |
|---|---|
| `robot/Dockerfile` | ROS 2 Jazzy、libcamera/rpicam、Nav2、Gazebo、Zenoh bridge、CycloneDDS、SPI/IMU 依赖 |
| `robot/Dockerfile.slam` | RTAB-Map Jazzy 基础上编译 Pangolin 与 ORB-SLAM3 |
| `yolo/Dockerfile.yolo` | Ultralytics YOLO 与 Zenoh Python 客户端 |
| `yolo/Dockerfile.segformer_trt` | NVIDIA TensorRT、OpenCV、Zenoh 和 SegFormer engine 运行环境 |

## 8. 可观测性与验证建议

| 验证目标 | 建议命令 | 预期 |
|---|---|---|
| ROS 图 | `ros2 node list`、`ros2 topic list -t` | 节点/接口与所选启动模式一致 |
| 图像链路 | `ros2 topic hz /camera/image_raw` 或 compressed 话题 | 频率接近配置值，时间戳连续 |
| IMU | `ros2 topic echo /imu/data_raw --once` | frame 为 `imu_link`，单位为 m/s² 与 rad/s |
| TF | `ros2 run tf2_tools view_frames`、`ros2 run tf2_ros tf2_monitor` | 无断链、无重复发布者、时间有效 |
| 伪激光 | `ros2 topic hz /seg/scan`、RViz | 扫描约 10 Hz、量程和坐标系正确 |
| Nav2 | `make status` | 生命周期节点处于预期活跃状态 |
| 网络/DDS | `ros2 multicast receive/send`、`ros2 doctor --report` | 多机发现正常，RMW 与部署预期一致 |
| 记录回放 | `make record`、`make play` | 可离线复现图像、IMU、TF、里程计问题 |

建议把每次实车测试的版本、相机标定文件、机器人外参、模型/engine 哈希、容器标签和 rosbag 路径记录到测试日志，避免因配置漂移导致结论不可复现。

## 9. 当前风险、约束与整理建议

### 9.1 已识别的运行约束

1. **启动配置与实际节点需核验**：`robot.launch.py` 中静态 TF 与 ICM 节点当前被注释；而 EKF、SLAM 与导航配置依赖 IMU 和完整 TF。因此完整实车启动前必须明确这些数据由何处发布。
2. **外部组件不是仓库内闭环**：ORB-SLAM3 ROS 2 工作空间、LLM/Whisper/TTS 服务、Zenoh router、TensorRT engine 和模型文件均需独立准备。`Makefile` 与 Compose 中的路径、镜像标签应作为部署清单维护。
3. **硬件执行需有安全措施**：`/cmd_vel` 直接驱动电机/舵机。实车应增加急停、命令超时归零、速度/转角限幅、启动互锁与看门狗；在这些措施验证前不要让 Nav2 无人值守控制实车。
4. **伪激光的不确定性**：单目投影对标定、俯仰、地面形态、光照和语义误检敏感，不能假定与物理激光雷达同等可靠。
5. **容器权限较高**：host 网络、privileged 和 `/dev` 挂载便于硬件调试，也扩大了隔离边界；生产设备应最小化权限、挂载和暴露端口。

### 9.2 推荐的后续整理优先级

1. **固化运行矩阵**：为 `mono/stereo/laser × rtabmap/orbslam3/slam_toolbox × real/sim` 建立表格，列明 launch 命令、必需话题、必需容器、预期 TF 发布者及验收命令。
2. **收敛配置源**：为机器人外参、相机内参、Nav2、EKF 和模型路径建立环境/配置覆盖规则；避免 Makefile、Compose、Python 默认值中重复硬编码路径。
3. **补齐自动验证**：新增无硬件单元测试（参数、JSON、launch 导入）、ROS 图集成测试和伪激光几何回归测试；针对 `robot_config.json` 改动执行投影一致性检查。
4. **明确接口契约**：为 `/camera/*`、`/imu/data_raw`、`/wheel_odom`、`/visual_odom`、`/seg/scan`、`/ekf/odom`、`/cmd_vel` 记录消息类型、频率、frame、QoS、时间基准和责任组件。
5. **分离实验与生产代码**：将含空格的备份 launch/脚本、调试输出和实验模型入口归档或移入 `experiments/`，为推荐路径保持单一、可测试的入口。

## 10. 结论

该仓库已具备从树莓派相机/IMU/底盘到 ROS 2 建图导航、从麦克风到云端语音交互，以及以 Zenoh 解耦重型视觉推理的完整实验骨架。其核心优势是覆盖实车、仿真、容器化和多种 SLAM 方案；当前工程化重点应放在**启动配置一致性、外部依赖可复现性、实车安全闭环和伪激光质量验证**。完成这些整理后，项目可从“多能力实验仓库”演进为具有清晰运行矩阵和可验收接口的机器人系统。
