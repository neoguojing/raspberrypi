import torch
import argparse
from transformers import SegformerForSemanticSegmentation, SegformerImageProcessor
import os

def make_onnx_filename(model_name_or_path: str, opset: int) -> str:
    """
    根据模型名、opset、设备和环境自动生成 ONNX 文件名。
    
    示例输出:
      segformer-nvidia_segformer-b2-finetuned-ade-512-512_torch2.3.0_cu121_opset17.onnx
      segformer-my_local_model_torch2.2.0_cpu_opset16.onnx
    """
    # 获取 PyTorch 版本（去掉 +cuXXX 等后缀）
    torch_ver = torch.__version__.split('+')[0]
    
    # 获取 CUDA 或 CPU 标识
    if torch.cuda.is_available():
        cuda_ver = torch.version.cuda
        if cuda_ver:
            # 转为 cuXXX 格式，如 cu121
            cuda_str = "cu" + cuda_ver.replace('.', '')
        else:
            cuda_str = "cpu"
    else:
        cuda_str = "cpu"
    
    # 清理模型名（只保留字母、数字、下划线、连字符）
    base_name = os.path.basename(model_name_or_path)
    clean_name = "".join(c if c.isalnum() or c in ('_', '-') else '_' for c in base_name)
    
    # 构造文件名
    filename = f"{clean_name}_torch{torch_ver}_{cuda_str}_opset{opset}.onnx"
    return filename

def export_segformer_to_onnx(
    model_name_or_path="nvidia/segformer-b2-finetuned-ade-512-512",
    output_path="./models/segformer_b2.onnx",
    opset=18,
    use_gpu=True,
    dynamic_axes=True
):
    """
    将 HuggingFace SegFormer 模型导出为 ONNX
    
    Args:
        model_name_or_path (str): HF model name 或本地路径
        output_path (str): ONNX 文件保存路径
        opset (int): ONNX opset 版本 (17 推荐)
        use_gpu (bool): 是否使用 GPU 进行 forward
        dynamic_axes (bool): 是否启用动态 batch 和尺寸
    """
    # 1. 加载模型
    device = "cuda" if use_gpu and torch.cuda.is_available() else "cpu"
    print(f"[INFO] Loading model '{model_name_or_path}' on {device} ...")
    model = SegformerForSemanticSegmentation.from_pretrained(model_name_or_path).to(device)
    model.eval()
    
    output_path = os.path.join(os.path.dirname(output_path), make_onnx_filename("segformer_b2", opset))

    # 2. 获取输入尺寸（默认模型大小）
    processor = SegformerImageProcessor.from_pretrained(model_name_or_path)
    dummy_input = torch.randn(
        1, 3, processor.size["height"], processor.size["width"],
        device=device, dtype=torch.float32
    )

    # 3. 确保输出目录存在
    os.makedirs(os.path.dirname(output_path), exist_ok=True)

    # 4. 正确调用 torch.onnx.export（不要把 model 和 args 放进 dict！）
    print(f"[INFO] Exporting ONNX model to {output_path} ...")
    torch.onnx.export(
        model,  # 模型（位置参数 0）
        (dummy_input,),  # 输入示例（位置参数 1）
        output_path,  # 输出文件路径（位置参数 2 或 f=output_path）
        export_params=True,
        opset_version=opset,
        do_constant_folding=True,
        input_names=['input'],
        output_names=['logits'],
        verbose=False,
        dynamic_axes={
            'input': {0: 'batch', 2: 'height', 3: 'width'},
            'logits': {0: 'batch', 2: 'height', 3: 'width'}
        } if dynamic_axes else None
    )

    print(f"[SUCCESS] ONNX model saved to {output_path}")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Export SegFormer PyTorch model to ONNX")
    parser.add_argument(
        "--model", type=str,
        default="nvidia/segformer-b2-finetuned-ade-512-512",
        help="HF model name or path"
    )
    
    args = parser.parse_args()

    export_segformer_to_onnx(
        model_name_or_path=args.model,
    )