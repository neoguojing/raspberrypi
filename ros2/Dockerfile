# 使用 ROS 2 Humble 或 Jazzy 的 ARM64 基础镜像
# 推荐使用 Humble，因为它更稳定，且有官方 ARM64 支持
ARG ROS_DISTRO=humble
FROM ros:$ROS_DISTRO-ros-base-jammy

# 设置工作目录
WORKDIR /home/ros_user

# 安装 ROS 2 依赖、构建工具和 RTAB-Map 依赖
RUN apt update && apt install -y \
    python3-pip \
    build-essential \
    cmake \
    git \
    libsqlite3-dev \
    ros-$ROS_DISTRO-rtabmap-ros \
    ros-$ROS_DISTRO-vision-opencv \
    ros-$ROS_DISTRO-depth-image-proc \
    && rm -rf /var/lib/apt/lists/*

# 设置 ROS 2 环境
ENV LD_LIBRARY_PATH="/opt/ros/$ROS_DISTRO/lib:${LD_LIBRARY_PATH}"
ENV PATH="/opt/ros/$ROS_DISTRO/bin:${PATH}"

# 设置容器启动时的默认命令
CMD ["bash"]