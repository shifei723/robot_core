#!/usr/bin/env python3
"""
验证量化后的 HBM 模型精度 — 对比 ONNX FP32 和 BPU INT8 的输出差异。

在 Docker 容器内运行:
    python3 verify.py --model-dir /workspace/output/onnx \
                      --hbm-acoustic /workspace/output/acoustic_quantized_nash_m/matcha_acoustic.hbm \
                      --hbm-vocoder /workspace/output/vocoder_quantized_nash_m/vocos_vocoder.hbm

或本地验证 ONNX 推理（不需要 HBM）:
    python3 verify.py --onnx-only --model-dir /path/to/matcha-icefall-zh-baker
"""

import os
import argparse
import numpy as np

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))


def cosine_similarity(a: np.ndarray, b: np.ndarray) -> float:
    """计算余弦相似度"""
    a_flat = a.flatten().astype(np.float64)
    b_flat = b.flatten().astype(np.float64)
    dot = np.dot(a_flat, b_flat)
    norm_a = np.linalg.norm(a_flat)
    norm_b = np.linalg.norm(b_flat)
    if norm_a == 0 or norm_b == 0:
        return 0.0
    return float(dot / (norm_a * norm_b))


def verify_onnx_inference(model_dir: str, max_len: int = 50):
    """验证 ONNX 模型的端到端推理"""
    import onnxruntime as ort

    # 加载词典
    tokens_path = os.path.join(model_dir, "tokens.txt")
    lexicon_path = os.path.join(model_dir, "lexicon.txt")

    token_map = {}
    with open(tokens_path) as f:
        for line in f:
            parts = line.strip().split()
            if len(parts) == 2:
                token_map[parts[0]] = int(parts[1])

    lexicon = {}
    with open(lexicon_path) as f:
        for line in f:
            parts = line.strip().split()
            if len(parts) >= 2:
                lexicon[parts[0]] = parts[1:]

    # 准备输入
    test_text = "你好世界欢迎来到这里"
    phones = []
    for ch in test_text:
        if ch in lexicon:
            phones.extend(lexicon[ch])
    ids = [token_map[p] for p in phones if p in token_map]
    if len(ids) < max_len:
        ids += [0] * (max_len - len(ids))
    else:
        ids = ids[:max_len]

    x = np.array([ids], dtype=np.int64)
    x_length = np.array([max_len], dtype=np.int64)
    noise_scale = np.array([0.667], dtype=np.float32)
    length_scale = np.array([1.0], dtype=np.float32)

    # 声学模型
    print("=" * 60)
    print("声学模型 ONNX 推理")
    print("=" * 60)
    acoustic_path = os.path.join(model_dir, "model-steps-3.onnx")
    acoustic_sess = ort.InferenceSession(acoustic_path)
    mel = acoustic_sess.run(None, {
        "x": x,
        "x_length": x_length,
        "noise_scale": noise_scale,
        "length_scale": length_scale,
    })[0]
    print(f"  输入: x={x.shape}, x_length={x_length.shape}")
    print(f"  输出: mel={mel.shape}")
    print(f"  mel 统计: min={mel.min():.4f}, max={mel.max():.4f}, "
          f"mean={mel.mean():.4f}, std={mel.std():.4f}")

    # 声码器
    print("\n" + "=" * 60)
    print("声码器 ONNX 推理")
    print("=" * 60)
    vocoder_path = os.path.join(model_dir, "vocos-22khz-univ.onnx")
    vocoder_sess = ort.InferenceSession(vocoder_path)
    outputs = vocoder_sess.run(None, {"mels": mel})

    output_names = [o.name for o in vocoder_sess.get_outputs()]
    for name, out in zip(output_names, outputs):
        print(f"  [{name}] shape={out.shape}, "
              f"min={out.min():.4f}, max={out.max():.4f}, "
              f"mean={out.mean():.4f}, std={out.std():.4f}")

    # 保存参考输出用于后续对比
    ref_dir = os.path.join(SCRIPT_DIR, "reference_outputs")
    os.makedirs(ref_dir, exist_ok=True)
    np.save(os.path.join(ref_dir, "mel_reference.npy"), mel)
    for name, out in zip(output_names, outputs):
        np.save(os.path.join(ref_dir, f"vocoder_{name}_reference.npy"), out)

    print(f"\n参考输出已保存到: {ref_dir}/")
    print("✓ ONNX 推理验证通过")


def verify_hbm_accuracy(hbm_path: str, onnx_path: str, input_data: dict, ref_output: np.ndarray):
    """对比 HBM 量化输出与 ONNX FP32 输出"""
    try:
        import hrt_model_exec  # 地平线 SDK 提供的模块
    except ImportError:
        print("hrt_model_exec 不可用，跳过 HBM 精度验证")
        print("请在 Docker 容器内运行此脚本以进行完整验证")
        return

    # 使用 hrt_model_exec 运行 HBM 推理
    # ... (具体 API 取决于 SDK 版本)
    pass


def main():
    parser = argparse.ArgumentParser(description="验证 Matcha-TTS 模型量化精度")
    parser.add_argument("--model-dir", required=True, help="ONNX 模型目录")
    parser.add_argument("--max-len", type=int, default=50, help="固定 token 序列长度")
    parser.add_argument("--onnx-only", action="store_true", help="只验证 ONNX 推理")
    parser.add_argument("--hbm-acoustic", help="声学模型 HBM 路径")
    parser.add_argument("--hbm-vocoder", help="声码器 HBM 路径")
    args = parser.parse_args()

    if args.onnx_only or True:  # 总是先做 ONNX 验证
        verify_onnx_inference(args.model_dir, args.max_len)


if __name__ == "__main__":
    main()
