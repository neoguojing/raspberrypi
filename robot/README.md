# Robot 模块说明（ROS2 + 单目视觉导航）

本目录是机器人侧（ROS2）的核心工程，包含底盘控制、相机/IMU、SLAM、定位融合、导航与仿真相关能力。

## 项目视频（保留 2 个位置）

- 视频 1（本地演示文件）：`../asset/fpv-demo-50-2.mp4`
- 视频 2（预留位置，后续补充）：`<TODO: 在此补充第二个演示视频链接或路径>`

---

## 1. 目录结构

- `launch/`：系统启动编排（实车/仿真/算法组合）
- `config/`：URDF、EKF、Nav2、相机、RTAB-Map 等参数
- `robot/`：ROS2 Python 节点实现（底盘、相机、感知、控制）
- `docs/`：专题文档（IMU / Nav2）
- `Dockerfile*`：容器化构建入口

---

## 2. 快速开始（Start）

> 以下命令以 ROS2 Jazzy 为参考，按你的环境调整。

### 2.1 安装依赖（示例）

```bash
sudo apt update
sudo apt install -y \
  ros-jazzy-desktop \
  ros-jazzy-nav2-bringup \
  ros-jazzy-robot-localization \
  ros-jazzy-rtabmap-ros \
  ros-jazzy-tf2-tools
```

### 2.2 构建

在仓库根目录执行：

```bash
colcon build --packages-select robot
source install/setup.bash
```

### 2.3 常用启动命令

```bash
# 仅机器人基础节点（底盘 / 相机 / TF 等）
ros2 launch robot robot.launch.py

# 完整链路（示例，按你当前配置选择）
ros2 launch robot full.launch.py

# 仿真链路
ros2 launch robot full.sim.launch.py
```

---

## 3. 系统架构（简化）

### 3.1 ORB-SLAM3 节点（视觉里程计 + 稀疏地图）

- 订阅：
  - `/camera/image_raw` 或 `/camera/image_raw/compressed`
  - `/camera/camera_info`
  - `/imu/data_raw`
- 发布：
  - `/slam3/pose` (`geometry_msgs/PoseStamped`)
  - `/slam3/odom` (`nav_msgs/Odometry`)
  - `/slam3/map_points` (`sensor_msgs/PointCloud2`)
  - `/slam3/debug_image` (`sensor_msgs/Image`，默认可关闭)
- TF（可选）：`map -> odom`

### 3.2 robot_localization（EKF）

- 订阅：`/imu/data_raw`、`/slam3/odom`（或 `/wheel_odom`）
- 发布：`/odometry/filtered`（或 `/ekf/odom`）
- TF：`odom -> base_footprint`

### 3.3 Nav2 导航栈

- 依赖输入：
  - `/map`（来自 RTAB-Map）
  - `/ekf/odom`
  - `/scan`（由视觉分割伪造或其他传感器）
  - `/cloud_obstacles` / `/slam3/map_points`（局部避障增强）
- 关键输出：
  - `cmd_vel`
  - `/plan`
  - `/local_plan`

### 3.4 RTAB-Map（建图）

- 订阅：`/camera/image_raw`、`/camera/camera_info`、`/ekf/odom`、`/imu/data_raw`
- 发布：`/map`、`/cloud_obstacles`
- TF：`map -> odom`

---

## 4. TF 目标结构

```text
map
 └── odom                (EKF)
      └── base_footprint (EKF)
           └── base_link (Static TF)
                ├── imu_link
                └── camera_link
                     └── camera_link_optical
```

参考安装位姿（示例）：

| Link | X | Y | Z | 说明 |
|---|---:|---:|---:|---|
| base_footprint | 0 | 0 | 0 | 车体中心地面投影 |
| base_link | 0 | 0 | 0.021 | 车体中心 |
| imu_link | 0 | 0 | 0.041 | IMU 中心 |
| camera_link | 0.08 | 0 | 0.071 | 相机外壳 |
| camera_link_optical | 0.08 | 0 | 0.071 | 光学坐标系（旋转后） |

---

## 5. 联调排障清单（网络 / DDS / 话题 / TF）

### 5.1 网络发现

```bash
# 服务器端
ros2 multicast receive

# 树莓派端
ros2 multicast send
```

### 5.2 图与话题检查

```bash
ros2 node list
ros2 node info /your_slam_node_name
ros2 topic list -t
ros2 topic info /camera/image_raw --verbose
ros2 topic echo /camera/image_raw --qos-profile sensor_data
```

### 5.3 传输质量

```bash
ros2 topic hz /camera/image_raw
ros2 topic hz /imu/data
ros2 topic bw /camera/image_raw
ros2 topic delay /camera/image_raw
```

### 5.4 CycloneDDS 检查

```bash
ros2 doctor --report | grep rmw
# sudo apt install ros-jazzy-cyclonedds-tools
ros2 run cyclonedds_cpp_tools cyclone_find_topic /camera/image_raw
ros2 run cyclonedds_cpp_tools cyclone_list_nodes
```

### 5.5 TF 检查

```bash
ros2 run tf2_tools view_frames
ros2 run tf2_ros tf2_monitor
```

---

## 6. 仿真环境（Gazebo Harmonic）

```bash
sudo apt update && sudo apt install -y curl gnupg lsb-release
sudo curl https://packages.osrfoundation.org/gazebo.gpg \
  --output /usr/share/keyrings/pkgs-osr-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osr-archive-keyring.gpg] \
http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" \
| sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null

sudo apt update
sudo apt install -y gz-harmonic

export GZ_PARTITION=sim
gz sim -v 4 world/bed_room.sdf
```

---

## 7. 技术路线结论（当前）

- ✅ 采用：RTAB-Map（单目 + 运动估计）生成导航栅格地图。
- ❌ 暂不采用：仅依赖 ORB-SLAM3 稀疏点云直接供 Nav2 做建图。
- ✅ 在做：YOLOv8-Seg -> 几何投影 -> 伪 `/scan`（局部代价地图）。
- ❌ 暂不采用：单目深度大模型（Depth-Anything / MiDaS）实时链路（算力压力大）。

---

## 8. 优秀开源项目参考

以下项目是本模块设计和实现的重要参考：

1. **Nav2**（ROS2 官方导航栈）
   - 仓库：<https://github.com/ros-navigation/navigation2>
   - 价值：路径规划、局部控制、行为树导航框架。

2. **RTAB-Map / rtabmap_ros**
   - 仓库：<https://github.com/introlab/rtabmap>
   - ROS2 封装：<https://github.com/introlab/rtabmap_ros>
   - 价值：视觉里程计、回环检测、占据栅格地图。

3. **robot_localization**
   - 文档：<https://docs.ros.org/en/rolling/p/robot_localization/>
   - 价值：IMU/里程计多源融合，稳定发布 `odom -> base_*`。

4. **ORB-SLAM3**
   - 仓库：<https://github.com/UZ-SLAMLab/ORB_SLAM3>
   - 价值：高质量视觉里程计与稀疏地图能力。

5. **slam_toolbox**
   - 仓库：<https://github.com/SteveMacenski/slam_toolbox>
   - 价值：2D SLAM 和地图保存/加载工具链（可作为对照方案）。

6. **Ultralytics YOLO**
   - 仓库：<https://github.com/ultralytics/ultralytics>
   - 价值：分割模型用于障碍物提取与伪激光生成。

---

## 9. 其他调试入口

```bash
curl http://127.0.0.1:25000/@/local/ros2/node/**
```
