# Makefile

# --- Variables ---
PYTHON = python3
# 推荐使用 .venv 作为虚拟环境目录名
VENV_DIR = .venv
PIP = $(VENV_DIR)/bin/pip
PYTHON_EXEC = $(VENV_DIR)/bin/python

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

.PHONY: all init venv requirements run clean help ros2 ros2_run ros2_dev slam3 slam3_run

# --- Targets ---

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
	docker build -t orb_slam3_jazzy:amd64 -f ros2/Dockerfile.slam .
	@echo "--- ✅ 镜像构建完成 ---"

slam3_run:
	@echo "--- 🚀 正在运行容器 $(CONTAINER_NAME) (带 Pangolin 图形支持) ---"
	docker run $(RUN_FLAGS) \
		--name orbslam3_container\
		--privileged \
		$(FULL_IMAGE_NAME) bash

ros2:
	docker build -t rtabmap_ros_jazzy:pi5 ./ros2/

ros2_run:
	docker run -it --rm \
		--name rtabmap_container \
		--privileged \
		--network host \
		rtabmap_ros_jazzy:pi5
ros2_dev:
	-docker rm -f rtabmap_dev_container 2>/dev/null || true
	docker run -d \
		--name rtabmap_dev_container \
		--network host \
		--privileged \
		-v ${PWD}:/home/ros_user/ros2_ws/src \
		rtabmap_ros_jazzy:pi5 \
		sleep infinity

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