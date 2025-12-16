# 指定基础镜像
FROM orb_slam3_jazzy:pi5

# 切换为 root 用户以安装系统软件包和构建依赖
USER root

# 1. 安装系统依赖和 ROS 2 环境设置所需的基本工具
#    - libcap-dev: 解决 python-prctl (picamera2 依赖) 的编译问题
#    - python3-libcamera: 提供 libcamera C++ 库的 Python 绑定（必须系统安装）
#    - python3-picamera2: 推荐使用这个高层库，它依赖 libcamera
RUN apt-get update && \
    apt-get install -y --no-install-recommends \
    python3-pip \
    python3-venv \
    libcap-dev \
    python3-lgpio \
    python3-libcamera \
    v4l-utils \
    libcamera-dev \
    libcamera-tools && \
    rm -rf /var/lib/apt/lists/*

# 设置工作目录
WORKDIR /home/ros_user/ros2_ws/src

# === 使用 Venv 并继承系统包 ===

# 1. 创建一个虚拟环境，关键是使用 --system-site-packages 继承 libcamera 和 picamera2
ENV VIRTUAL_ENV=/home/ros_user/ros2_ws/venv
RUN python3 -m venv $VIRTUAL_ENV --system-site-packages

# 2. 复制并安装依赖
# 注意：确保 requirementrobot.txt中不包含 libcamera 或 picamera2，
# 因为它们已经通过 apt 安装并由 --system-site-packages 继承。
RUN $VIRTUAL_ENV/bin/pip install --no-cache-dir \
    # 在这里列出您的项目所需的包，例如：
    picamera2 \
    # ... 其他包 ...
    && echo "Python packages installed successfully."

RUN echo "source /opt/ros/jazzy/setup.sh" >> /root/.bashrc && \
    echo "source \$VIRTUAL_ENV/bin/activate" >> /root/.bashrc

RUN git clone https://github.com/neoguojing/ORB_SLAM3_ROS2.git orbslam3

ENV ROS_DOMAIN_ID=42