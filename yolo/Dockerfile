# =========================================================
# Base: 使用集成了 CUDA/PyTorch 的 HuggingFace 官方镜像
# =========================================================
FROM huggingface/transformers-pytorch-gpu:latest

ENV DEBIAN_FRONTEND=noninteractive
ENV PYTHONUNBUFFERED=1

# 设置模型缓存目录
ENV TRANSFORMERS_CACHE=/root/.cache/huggingface

# =========================================================
# 安装依赖：Zenoh, OpenCV, timm (SegFormer 依赖)
# =========================================================
RUN pip install --no-cache-dir \
    eclipse-zenoh \
    opencv-python-headless \
    timm \
    scikit-image

# =========================================================
# 预下载 SegFormer 模型 (ADE20K 预训练)
# 在构建时下载，避免运行阶段受网络波动影响
# =========================================================
RUN python3 -c "from transformers import SegformerImageProcessor, SegformerForSemanticSegmentation; \
    model_name = 'nvidia/segformer-b2-finetuned-ade-512-512'; \
    SegformerImageProcessor.from_pretrained(model_name); \
    SegformerForSemanticSegmentation.from_pretrained(model_name)"

WORKDIR /home/zenoh

ENV HF_HOME=/root/.cache/huggingface
# 推荐使用脚本启动，以便同时管理 zenohd 和 推理程序
# 如果你只需要跑推理，直接运行 python
CMD ["python3", "-m", "yolo.zen_seg"]