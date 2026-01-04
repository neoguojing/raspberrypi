# =========================================================
# Base: Official Ultralytics YOLO image
# =========================================================
FROM ultralytics/ultralytics:latest

ENV DEBIAN_FRONTEND=noninteractive
ENV PYTHONUNBUFFERED=1

# Ultralytics 模型缓存目录
ENV YOLO_MODELS_DIR=/root/.cache/ultralytics

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