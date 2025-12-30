# =========================================================
# Base: Official Ultralytics YOLO image
# =========================================================
FROM ultralytics/ultralytics:latest

ENV DEBIAN_FRONTEND=noninteractive
ENV PYTHONUNBUFFERED=1

# Ultralytics 模型缓存目录
ENV YOLO_MODELS_DIR=/root/.cache/ultralytics

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
# Download YOLO11 Segmentation model at build time
# =========================================================
RUN python3 -c "from ultralytics.utils.downloads import attempt_download_asset; \
                attempt_download_asset('yolo11n-seg.pt')"


# =========================================================
# Expose Zenoh port (optional)
# =========================================================
EXPOSE 7447

WORKDIR /home/zenoh

# =========================================================
# Start Zenoh Router (PID 1)
# =========================================================
# CMD ["zenohd]
CMD ["sh", "-c", "while true; do sleep 1000; done"]