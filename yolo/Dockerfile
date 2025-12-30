# =========================================================
# Base: Official Ultralytics YOLO image
# =========================================================
FROM ultralytics/ultralytics:latest

# =========================================================
# Environment
# =========================================================
ENV DEBIAN_FRONTEND=noninteractive
ENV PYTHONUNBUFFERED=1

# =========================================================
# System dependencies
# =========================================================
RUN apt-get update && apt-get install -y \
    curl \
    gnupg \
    ca-certificates \
    iproute2 \
    net-tools \
 && rm -rf /var/lib/apt/lists/*

# =========================================================
# Install Zenoh (Router)
# =========================================================
RUN mkdir -p /etc/apt/keyrings \
 && curl -L https://download.eclipse.org/zenoh/debian-repo/zenoh-public-key \
    | gpg --dearmor --yes --output /etc/apt/keyrings/zenoh-public-key.gpg \
 && echo "deb [signed-by=/etc/apt/keyrings/zenoh-public-key.gpg] \
    https://download.eclipse.org/zenoh/debian-repo/ /" \
    > /etc/apt/sources.list.d/zenoh.list \
 && apt-get update \
 && apt-get install -y zenoh \
 && rm -rf /var/lib/apt/lists/*

# =========================================================
# Install Zenoh Python client
# =========================================================
RUN pip install --no-cache-dir eclipse-zenoh

# =========================================================
# (Optional) Copy your YOLO + Zenoh application
# =========================================================
# 示例：你可以后续再加
# COPY yolo_zenoh_app.py /app/yolo_zenoh_app.py
# WORKDIR /app

# =========================================================
# Expose Zenoh default port (optional, host mode不用)
# =========================================================
EXPOSE 7447

# =========================================================
# Start Zenoh Router
# =========================================================
# zenohd 必须是 PID 1，绑定容器生命周期
CMD ["zenohd"]