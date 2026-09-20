#!/usr/bin/env python3
"""
fix_acoustic_onnx.py - 修复 Matcha-TTS 声学模型的 Range 节点

地平线 BPU 不支持 Range 算子。本脚本通过以下方式解决:
1. 用 onnxruntime 做一次真实推理，收集所有 Range 节点的输出值
2. 将 Range 节点替换为 Constant 节点（预计算值）

前提: 输入 shape 必须固定 (max_len=50, length_scale=1.0)

用法:
    python3 fix_acoustic_onnx.py [--max-len 50] [--length-scale 1.0]
"""

import os
import sys
import argparse
import numpy as np
import onnx
from onnx import numpy_helper, TensorProto, helper
import onnxruntime as ort

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
DEFAULT_MODEL = os.path.join(
    os.path.dirname(SCRIPT_DIR), "matcha-icefall-zh-baker", "model-steps-3.onnx"
)
DEFAULT_OUTPUT = os.path.join(SCRIPT_DIR, "output", "onnx", "model-steps-3.onnx")


def get_range_outputs(model_path: str, max_len: int, length_scale: float):
    """通过运行模型获取 Range 节点的实际输出值"""
    # 先找到所有 Range 节点的输出名
    model = onnx.load(model_path)
    range_nodes = [n for n in model.graph.node if n.op_type == "Range"]
    range_output_names = [n.output[0] for n in range_nodes]

    print(f"找到 {len(range_nodes)} 个 Range 节点:")
    for rn in range_nodes:
        print(f"  {rn.name} -> {rn.output[0]}")

    # 修改模型，将 Range 输出加入 graph output
    model_with_intermediates = onnx.load(model_path)
    for name in range_output_names:
        # 添加为额外输出
        output_value_info = helper.make_tensor_value_info(name, TensorProto.FLOAT, None)
        model_with_intermediates.graph.output.append(output_value_info)

    # 保存临时模型
    tmp_model_path = "/tmp/matcha_with_range_outputs.onnx"
    onnx.save(model_with_intermediates, tmp_model_path)

    # 运行推理获取 Range 输出
    sess = ort.InferenceSession(tmp_model_path)

    # 构造固定输入
    x = np.zeros((1, max_len), dtype=np.int64)
    # 填一些真实 token IDs
    sample_ids = [1117, 597, 1518, 703, 650, 1851, 841, 313, 1963, 877]
    x[0, : len(sample_ids)] = sample_ids

    x_length = np.array([max_len], dtype=np.int64)
    noise_scale = np.array([0.667], dtype=np.float32)
    length_scale_arr = np.array([length_scale], dtype=np.float32)

    # 获取所有输出（包括 mel + Range 节点输出）
    output_names = [o.name for o in sess.get_outputs()]
    results = sess.run(output_names, {
        "x": x,
        "x_length": x_length,
        "noise_scale": noise_scale,
        "length_scale": length_scale_arr,
    })

    # 提取 Range 节点的值
    range_values = {}
    for name, result in zip(output_names, results):
        if name in range_output_names:
            range_values[name] = result
            print(f"  {name}: shape={result.shape}, dtype={result.dtype}, values={result[:5]}...")

    # 清理临时文件
    os.remove(tmp_model_path)

    return range_values, model


def replace_range_with_constant(model: onnx.ModelProto, range_values: dict) -> onnx.ModelProto:
    """将 Range 节点替换为 Constant 节点"""
    nodes_to_remove = []
    nodes_to_add = []

    for node in model.graph.node:
        if node.op_type == "Range" and node.output[0] in range_values:
            value = range_values[node.output[0]]
            # 创建 Constant 节点替代 Range
            const_tensor = numpy_helper.from_array(value, name=node.output[0] + "_const")
            const_node = helper.make_node(
                "Constant",
                inputs=[],
                outputs=[node.output[0]],
                name=node.name + "_replaced",
                value=const_tensor,
            )
            nodes_to_remove.append(node)
            nodes_to_add.append(const_node)
            print(f"  替换: {node.name} -> Constant (shape={value.shape})")

    # 移除旧节点，添加新节点
    for node in nodes_to_remove:
        model.graph.node.remove(node)
    for node in nodes_to_add:
        model.graph.node.insert(0, node)  # 插入到开头

    return model


def main():
    parser = argparse.ArgumentParser(description="修复 Matcha-TTS ONNX 的 Range 算子")
    parser.add_argument("--model", default=DEFAULT_MODEL, help="输入 ONNX 模型路径")
    parser.add_argument("--output", default=DEFAULT_OUTPUT, help="输出修复后的模型路径")
    parser.add_argument("--max-len", type=int, default=50, help="固定 token 序列长度")
    parser.add_argument("--length-scale", type=float, default=1.0, help="语速缩放")
    args = parser.parse_args()

    print("=" * 60)
    print("修复 Matcha-TTS ONNX 模型 - 替换 Range 算子为 Constant")
    print("=" * 60)
    print(f"  输入模型: {args.model}")
    print(f"  输出模型: {args.output}")
    print(f"  固定参数: max_len={args.max_len}, length_scale={args.length_scale}")
    print()

    # 1. 获取 Range 节点的实际输出值
    print("步骤 1: 运行模型获取 Range 节点输出值...")
    range_values, model = get_range_outputs(args.model, args.max_len, args.length_scale)

    if not range_values:
        print("错误: 未找到 Range 节点或未能获取输出值")
        return 1

    # 2. 替换 Range 为 Constant
    print("\n步骤 2: 替换 Range 节点为 Constant...")
    model = replace_range_with_constant(model, range_values)

    # 3. 验证修改后的模型
    print("\n步骤 3: 验证修改后的模型...")
    try:
        onnx.checker.check_model(model, full_check=False)
        print("  OK ONNX 模型结构验证通过")
    except Exception as e:
        print(f"  WARN 结构验证警告: {e}")
        print("  (可能是模型太大，跳过完整检查)")

    # 4. 保存
    os.makedirs(os.path.dirname(args.output), exist_ok=True)
    print(f"\n步骤 4: 保存修复后的模型到 {args.output}...")
    onnx.save(model, args.output)
    size_mb = os.path.getsize(args.output) / 1024 / 1024
    print(f"  OK 已保存 ({size_mb:.1f} MB)")

    # 5. 验证推理结果一致性
    print("\n步骤 5: 对比推理结果...")
    sess_orig = ort.InferenceSession(args.model)
    sess_fixed = ort.InferenceSession(args.output)

    x = np.zeros((1, args.max_len), dtype=np.int64)
    x[0, :10] = [1117, 597, 1518, 703, 650, 1851, 841, 313, 1963, 877]
    inputs = {
        "x": x,
        "x_length": np.array([args.max_len], dtype=np.int64),
        "noise_scale": np.array([0.667], dtype=np.float32),
        "length_scale": np.array([args.length_scale], dtype=np.float32),
    }

    mel_orig = sess_orig.run(None, inputs)[0]
    mel_fixed = sess_fixed.run(None, inputs)[0]

    # 计算余弦相似度
    cos_sim = np.dot(mel_orig.flatten(), mel_fixed.flatten()) / (
        np.linalg.norm(mel_orig.flatten()) * np.linalg.norm(mel_fixed.flatten())
    )
    max_diff = np.abs(mel_orig - mel_fixed).max()
    print(f"  原始 mel shape: {mel_orig.shape}")
    print(f"  修复 mel shape: {mel_fixed.shape}")
    print(f"  余弦相似度: {cos_sim:.8f}")
    print(f"  最大绝对差: {max_diff:.8e}")

    if cos_sim > 0.9999:
        print("  OK 推理结果一致性验证通过!")
    else:
        print("  WARN 推理结果有差异，请检查!")
        return 1

    print("\n" + "=" * 60)
    print("修复完成! 请使用修复后的模型重新编译:")
    print(f"  docker exec xgs_OE_v3.7.0 hb_compile --config /workspace/acoustic_config.yaml")
    print("=" * 60)
    return 0


if __name__ == "__main__":
    sys.exit(main())
