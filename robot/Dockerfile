# 指定基础镜像
FROM guojingneo/orb_slam3_jazzy:pi5

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
    v4l-utils \

    meson \
    ninja-build \
    
    # libcamera 编译还需要这些
    libgnutls28-dev \
    libboost-dev \
    libyaml-dev \

    # 解决 libudev 缺失
    libudev-dev \
    # 解决 glib-2.0 和 GStreamer 缺失
    libglib2.0-dev \
    libgstreamer1.0-dev \
    libgstreamer-plugins-base1.0-dev \

    python3-jinja2 \
    python3-ply \
    python3-yaml \
    pybind11-dev \

    #ffmpeg
    libavdevice-dev \
    ffmpeg \
    # 网页可视化
    ros-jazzy-foxglove-bridge \
    ros-jazzy-tf-transformations \
    # IMU
    python3-spidev \
    ros-jazzy-imu-tools \
    # DDS
    ros-jazzy-rmw-cyclonedds-cpp \
    # 仿真
    ros-jazzy-ros-gz \
    ros-jazzy-ros-gz-bridge \
    ros-jazzy-ros-gz-image \
    ros-jazzy-ros-gz-sim \
    ros-jazzy-xacro \
    # nv2
    ros-jazzy-nav2-bringup \
    && rm -rf /var/lib/apt/lists/*


WORKDIR /usr/local/
RUN git clone https://github.com/raspberrypi/libcamera.git --branch v0.5.0+rpt20250429
WORKDIR /usr/local/libcamera/
RUN meson setup build --buildtype=release \
    -Dpipelines=rpi/vc4,rpi/pisp \
    -Dipas=rpi/vc4,rpi/pisp \
    -Dv4l2=true \
    -Dgstreamer=enabled \
    -Dtest=false \
    -Dlc-compliance=disabled \
    -Dcam=disabled \
    -Dqcam=disabled \
    -Ddocumentation=disabled \
    -Dpycamera=enabled && \
    ninja -C build && \
    sudo ninja -C build install && \
    ldconfig && \
    cd .. && rm -rf libcamera

# rpicam-apps
WORKDIR /usr/local/
RUN git clone https://github.com/raspberrypi/rpicam-apps.git --branch v1.10.1
WORKDIR /usr/local/rpicam-apps/
RUN meson setup build \
    # -Denable_libav=enabled \
    -Denable_libav=disabled \
    -Denable_drm=enabled \
    -Denable_egl=enabled \
    -Denable_qt=enabled \
    -Denable_opencv=disabled \
    -Denable_tflite=disabled && \
    meson compile -C build && \
    meson install -C build && \
    ldconfig && \
    cd .. && rm -rf rpicam-apps

# kmsxx
WORKDIR /usr/local/
RUN git clone https://github.com/tomba/kmsxx.git
WORKDIR /usr/local/kmsxx
RUN git reset --hard 2efdd2583da9575242091bb53c57a311c3eacbc6 && \
    git submodule update --init && \
    meson build -Dpykms=enabled && \
    sudo ninja -C build install && \
    ldconfig && cd .. && rm -rf kmsxx

# 设置工作目录
WORKDIR /home/ros_user/ros2_ws

# === 使用 Venv 并继承系统包 ===

# 1. 创建一个虚拟环境，关键是使用 --system-site-packages 继承 libcamera 和 picamera2
ENV VIRTUAL_ENV=/home/ros_user/venv

RUN python3 -m venv $VIRTUAL_ENV --system-site-packages && \
    echo "source /opt/ros/jazzy/setup.sh" >> /root/.bashrc && \
    echo "source \$VIRTUAL_ENV/bin/activate" >> /root/.bashrc

    # 确保 PYTHONPATH 初始化，并包含 libcamera 所在的路径
# 注意：我们先放 venv 的路径（如果有自定义的话），再放系统路径
ENV PYTHONPATH="/home/ros_user/venv/lib/python3.12/site-packages:/usr/lib/aarch64-linux-gnu/python3.12/site-packages:/usr/lib/python3/dist-packages:/usr/local/lib/aarch64-linux-gnu/python3.12/site-packages:$PYTHONPATH"

# picamera2 and rpi-libcamera
# upgrade pip required to passed setup-args
RUN $VIRTUAL_ENV/bin/pip install --upgrade pip
RUN $VIRTUAL_ENV/bin/pip install --no-cache-dir picamera2==0.3.33 \
    rpi-libcamera==0.1a10 -C setup-args="-Dwerror=false"
    # ultralytics
# 安装仿真
RUN curl https://packages.osrfoundation.org/gazebo.gpg --output /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg \
    && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" \
       | tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null \
    && apt-get update && apt-get install -y --no-install-recommends \
    libgz-transport13-dev \
    libgz-sim8-dev \
    libsdformat14-dev \
    # 仿真控制
    ros-jazzy-teleop-twist-keyboard \
    gdb \
    && apt-get clean && rm -rf /var/lib/apt/lists/*

# --- 添加 Zenoh 官方 GPG key ---
RUN mkdir -p /etc/apt/keyrings && \
    curl -fsSL https://download.eclipse.org/zenoh/debian-repo/zenoh-public-key \
    | gpg --dearmor -o /etc/apt/keyrings/zenoh-public-key.gpg

# --- 添加 Zenoh APT 源 ---
RUN echo "deb [signed-by=/etc/apt/keyrings/zenoh-public-key.gpg] \
https://download.eclipse.org/zenoh/debian-repo/ /" \
    > /etc/apt/sources.list.d/zenoh.list

# --- 安装 zenoh bridge ---
RUN apt-get update && apt-get install -y \
    zenoh-bridge-ros2dds \
 && rm -rf /var/lib/apt/lists/*

ENV ROS_DOMAIN_ID=42
ENV ROS_DISTRO=jazzy
# 设置环境变量指向该文件
# ENV CYCLONEDDS_URI=file:///etc/ros/cyclonedds/config.xml
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
ENV GZ_PARTITION=sim

# 放到你真正要执行的目录
ADD robot_starter.sh /home/ros_user/ros2_ws/starter.sh

# 确保可执行（双保险）
RUN chmod +x /home/ros_user/ros2_ws/starter.sh

CMD ["/home/ros_user/ros2_ws/starter.sh"]

# CMD ["ros2", "launch", "foxglove_bridge", "foxglove_bridge_launch.xml"]


