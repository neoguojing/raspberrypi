import torch
import argparse
from transformers import SegformerForSemanticSegmentation, SegformerImageProcessor
import os

def export_segformer_to_onnx(model_name_or_path="nvidia/segformer-b2-finetuned-ade-512-512", 
                             output_path="./models/segformer_b2.onnx", opset=17, use_gpu=True, dynamic_axes=True):
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

    # 2. 获取输入尺寸（默认模型大小）
    processor = SegformerImageProcessor.from_pretrained(model_name_or_path)
    dummy_input = torch.randn(
        1, 3, processor.size["height"], processor.size["width"], device=device, dtype=torch.float32
    )

    # 3. ONNX 导出配置
    onnx_export_args = dict(
        f= model,
        args=(dummy_input,),
        f=output_path,
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

    print(f"[INFO] Exporting ONNX model to {output_path} ...")
    torch.onnx.export(**onnx_export_args)

    print(f"[SUCCESS] ONNX model saved to {output_path}")

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Export SegFormer PyTorch model to ONNX")
    parser.add_argument("--model", type=str, default="nvidia/segformer-b2-finetuned-ade-512-512", help="HF model name or path")
    parser.add_argument("--output", type=str, default="./models/segformer_b2.onnx", help="ONNX output file path")
    parser.add_argument("--opset", type=int, default=17, help="ONNX opset version")
    parser.add_argument("--gpu", action="store_true", help="Use GPU for export")
    args = parser.parse_args()

    export_segformer_to_onnx(args.model, args.output, opset=args.opset, use_gpu=args.gpu)