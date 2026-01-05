# Makefile

# --- Variables ---
PYTHON = python3
# 推荐使用 .venv 作为虚拟环境目录名
VENV_DIR = .venv
PIP = $(VENV_DIR)/bin/pip
PYTHON_EXEC = $(VENV_DIR)/bin/python
PROJECT_ROOT := $(shell pwd)
ROBOT_DIR = $(PROJECT_ROOT)/robot
YOLO_DIR = $(PROJECT_ROOT)/yolo
SLAM3_APP_ROOT = /home/ros_user/orbslam3
ROS_DISTRO := jazzy
# APT packages needed for the project
# python3-pyaudio is sometimes better installed via apt on Raspberry Pi
# python3-pygame can also be installed via apt
APT_PACKAGES = \
    python3-pip \
    python3-venv \
    python3-pyaudio \
    python3-dev \
	portaudio19-dev \
	cmake \
    # espeak-ng # 如果你想使用espeak-ng作为本地TTS备选方案
    # mpg123    # 如果你想使用mpg123播放音频（pygame可以处理，但这是另一种选择）

# Files to clean
CLEAN_FILES = recorded_audio.wav response.mp3
CLEAN_DIRS = __pycache__ $(VENV_DIR) *.egg-info dist build

.PHONY: all init venv requirements run clean help slam3 image dev runtime ros2_install ros2_build ros2_algo ros2_robot ros2_clean ros2_sim ros2_full sim ros2_ctl yolo up down

all: help

help:
	@echo "Available targets:"
	@echo "  init          - Install system-level APT packages (requires sudo)."
	@echo "  venv          - Create a Python virtual environment in $(VENV_DIR)."
	@echo "  requirements  - Install Python packages from requirements.txt into the virtual environment."
	@echo "  setup         - Alias for 'venv' then 'requirements'."
	@echo "  run           - Run the main application (main.py)."
	@echo "  clean         - Remove virtual environment, cache files, and generated audio."
	@echo "  fullclean     - Alias for 'clean'."
	@echo "  list-audio    - Run the script to list audio devices."

# Target to install system-level dependencies
init:
	@echo "Installing system-level APT packages (requires sudo)..."
	sudo apt-get update
	sudo apt-get install -y $(APT_PACKAGES)
	@echo "System dependencies installed."

# Target to create virtual environment
venv:
	@if [ -d "$(VENV_DIR)" ]; then \
		echo "Virtual environment $(VENV_DIR) already exists."; \
	else \
		echo "Creating Python virtual environment in $(VENV_DIR)..."; \
		$(PYTHON) -m venv $(VENV_DIR) --system-site-packages; \
		echo "Virtual environment created. Activate with: source $(VENV_DIR)/bin/activate"; \
	fi
	@echo "Upgrading pip in venv..."
	$(PIP) install --upgrade pip

# Target to install Python requirements
requirements: venv # Ensure venv exists
	@echo "Installing Python packages from requirements.txt..."
	$(PIP) install -r requirements.txt
	@echo "Python packages installed."

# Convenience target to setup everything python related
setup: venv requirements

# Target to run the application
run: venv# Ensure requirements are installed
	@echo "Running the application (main.py)..."
	$(PYTHON_EXEC) cli.py chat 

slam3:
	@echo "--- 🛠️ 正在构建 Docker 镜像 $(FULL_IMAGE_NAME) ---"
	docker build -t guojingneo/orb_slam3_jazzy:pi5 -f $(ROBOT_DIR)/Dockerfile.slam .
	@echo "--- ✅ 镜像构建完成 ---"

image:
	docker build -t guojingneo/robot_rtabmap_slam3_jazzy:pi5 -f $(ROBOT_DIR)/Dockerfile .

yolo:
	docker build -t guojingneo/yolo_zenoh:pi5 -f $(YOLO_DIR)/Dockerfile .

runtime:
	docker run -d --rm \
		--name ros2_container \
		--privileged \
		--network host \
		guojingneo/robot_rtabmap_slam3_jazzy:pi5

HAS_NVIDIA := $(shell which nvidia-smi 2>/dev/null)
dev:
	-docker rm -f ros2_container 2>/dev/null || true
	docker run -d \
		--name ros2_container \
		--network host \
		--privileged \
		--ipc=host \
		-v /dev:/dev \
		-v /run/udev:/run/udev:ro \
		-v ${PWD}:/home/ros_user/ros2_ws \
		-v "${PWD}/../ORB_SLAM3_ROS2":/home/ros_user/orbslam3 \
		-e DISPLAY=${DISPLAY} \
		-e QT_X11_NO_MITSHM=1 \
		-v /tmp/.X11-unix:/tmp/.X11-unix \
		-v ~/.Xauthority:/root/.Xauthority:ro \
		$(if $(HAS_NVIDIA),--gpus all --env="NVIDIA_DRIVER_CAPABILITIES=all",) \
		guojingneo/robot_rtabmap_slam3_jazzy:pi5 

up: down
	$(if $(HAS_NVIDIA),docker compose --profile gpu up -d,docker compose up -d)

down:
	$(if $(HAS_NVIDIA), \
		docker compose --profile gpu down, \
		docker compose down)
# Target to list audio devices
list-audio: requirements
	@echo "Listing audio devices..."
	$(PYTHON_EXEC) list_audio_devices.py # 假设你有这个辅助脚本

# Target to clean up the project
clean:
	@echo "Cleaning up project files..."
	@if [ -d "$(VENV_DIR)" ]; then \
		echo "Removing virtual environment $(VENV_DIR)..."; \
		rm -rf $(VENV_DIR); \
	fi
	@echo "Removing other generated files and directories..."
	rm -f $(CLEAN_FILES)
	rm -rf $(CLEAN_DIRS)
	@find . -name "*.pyc" -exec rm -f {} +
	@find . -name "__pycache__" -exec rm -rf {} +
	@echo "Cleanup complete."

fullclean: clean

ros2_install:
	@echo "安装依赖..."
	rosdep update && \
	rosdep install -i --from-path $(ROBOT_DIR) --rosdistro $(ROS_DISTRO) -y

ros2_build: ros2_clean
	@echo "编译 ROS 2 节点..."
	colcon build --packages-select robot

ros2_robot: ros2_build
	@echo "启动robot系统..."
	# 使用 . 代替 source，并确保在项目根目录执行
	cd $(PROJECT_ROOT) && . install/setup.bash && \
	ros2 launch robot robot.launch.py

ros2_algo: ros2_build
	@echo "启动算法系统..."
	# 使用 . 代替 source，并确保在项目根目录执行
	cd $(PROJECT_ROOT) && . install/setup.bash && \
	cd $(SLAM3_APP_ROOT) && . install/setup.bash && \
	ros2 launch robot algo.launch.py

ros2_sim: ros2_build
	@echo "启动模拟系统..."
	# 使用 . 代替 source，并确保在项目根目录执行
	cd $(PROJECT_ROOT) && . install/setup.bash && \
	cd $(SLAM3_APP_ROOT) && . install/setup.bash && \
	ros2 launch robot full.sim.launch.py

ros2_offline: ros2_build
	@echo "启动模拟系统..."
	# 使用 . 代替 source，并确保在项目根目录执行
	cd $(PROJECT_ROOT) && . install/setup.bash && \
	cd $(SLAM3_APP_ROOT) && . install/setup.bash && \
	ros2 launch robot offline.launch.py

ros2_full: ros2_build
	@echo "启动生产系统..."
	# 使用 . 代替 source，并确保在项目根目录执行
	cd $(PROJECT_ROOT) && . install/setup.bash && \
	cd $(SLAM3_APP_ROOT) && . install/setup.bash && \
	ros2 launch robot full.launch.py

ros2_clean:
	@echo "进入 $(PROJECT_ROOT)"
	rm -rf build/ install/ log/ && \
	cd $(PROJECT_ROOT)/robot/ && rm -rf build/ install/ log/

# 用于启动键盘控制仿真模型
ros2_ctl:
	ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -p speed:=0.1 -p turn:=0.5

sim:
	export GZ_PARTITION=sim && gz sim -r world/bed_room.sdf
# 	export GZ_PARTITION=sim && gz sim -r world/living_room.sdf
# 	export GZ_PARTITION=sim && gz sim -r world/harmonic.sdf
# 	export GZ_PARTITION=sim && gz sim -r world/tugbot_depot.sdf

yolo_run:
	python3 -m yolo.zen_seg --config robot/config/imx219.json
