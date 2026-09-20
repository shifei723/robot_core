#!/usr/bin/env python3
"""
为 Matcha-TTS 两个 ONNX 模型生成地平线 BPU 校准数据。

声学模型: model-steps-3.onnx  (token IDs → mel spectrogram)
声码器:   vocos-22khz-univ.onnx (mel spectrogram → waveform)

用法:
    python3 gen_calibration_data.py [--model-dir DIR] [--num-samples N] [--max-len L]
"""

import os
import argparse
import time
import numpy as np

# ── 路径 ────────────────────────────────────────────────
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
DEFAULT_MODEL_DIR = os.path.join(os.path.dirname(SCRIPT_DIR), "matcha-icefall-zh-baker")
DEFAULT_OUT_DIR = os.path.join(SCRIPT_DIR, "calibration_data")
MAX_SEQ_LEN = 50        # 固定输入 token 序列长度（约一句中文）
NOISE_SCALE = 0.667     # Matcha-TTS 默认噪声缩放
LENGTH_SCALE = 1.0      # 语速缩放（1.0 = 正常）
NUM_SAMPLES = 10        # 校准样本数
MAX_MEL_LEN = 500       # 声码器固定 mel 帧数 (覆盖 50 token 输入的最大输出)


def load_tokens(path: str) -> dict:
    """加载 tokens.txt → {phone: id}"""
    mapping = {}
    with open(path, "r", encoding="utf-8") as f:
        for line in f:
            parts = line.strip().split()
            if len(parts) == 2:
                mapping[parts[0]] = int(parts[1])
    return mapping


def load_lexicon(path: str) -> dict:
    """加载 lexicon.txt → {char: [phone1, phone2, ...]}"""
    lex = {}
    with open(path, "r", encoding="utf-8") as f:
        for line in f:
            parts = line.strip().split()
            if len(parts) >= 2:
                lex[parts[0]] = parts[1:]
    return lex


def text_to_token_ids(text: str, lexicon: dict, token_map: dict, max_len: int) -> np.ndarray:
    """将中文字符串转为固定长度的 token ID 数组"""
    phones = []
    for ch in text:
        if ch in lexicon:
            phones.extend(lexicon[ch])
    ids = []
    for p in phones:
        if p in token_map:
            ids.append(token_map[p])
    # 填充或截断到 max_len
    if len(ids) < max_len:
        ids += [0] * (max_len - len(ids))
    else:
        ids = ids[:max_len]
    return np.array(ids, dtype=np.int64)


# ── 校准用中文样本句子 ──────────────────────────────────
SAMPLE_SENTENCES = [
    "你好世界欢迎来到这里",
    "今天天气非常好我们出去玩",
    "语音合成技术越来越成熟了",
    "请问明天有什么安排呢",
    "我喜欢听歌也喜欢看书",
    "早上好祝你一天愉快",
    "这个故事非常有趣让人深思",
    "科技改变生活创新引领未来",
    "学习是一件快乐的事情",
    "让我们一起努力创造美好的明天",
    "春风送暖万物复苏花开满园",
    "大海波澜壮阔令人心旷神怡",
]


def run_acoustic_model(sess, token_ids: np.ndarray):
    """运行声学模型，返回 mel spectrogram"""
    x = token_ids.reshape(1, -1)
    x_length = np.array([x.shape[1]], dtype=np.int64)
    noise_scale = np.array([NOISE_SCALE], dtype=np.float32)
    length_scale = np.array([LENGTH_SCALE], dtype=np.float32)

    outputs = sess.run(
        None,
        {
            "x": x,
            "x_length": x_length,
            "noise_scale": noise_scale,
            "length_scale": length_scale,
        },
    )
    return outputs[0]  # mel: (1, 80, T)


def generate_acoustic_calibration(
    sess, lexicon: dict, token_map: dict, max_len: int, num_samples: int, out_dir: str
):
    """生成声学模型校准数据 — 每个输入单独一个子目录"""
    # 为每个输入创建独立子目录（与 YAML cal_data_dir 对应）
    sub_dirs = {
        "x": os.path.join(out_dir, "x"),
        "x_length": os.path.join(out_dir, "x_length"),
        "noise_scale": os.path.join(out_dir, "noise_scale"),
        "length_scale": os.path.join(out_dir, "length_scale"),
    }
    for d in sub_dirs.values():
        os.makedirs(d, exist_ok=True)

    print(f"\n{'='*60}")
    print(f"生成声学模型校准数据 ({num_samples} 个样本, max_len={max_len})")
    print(f"{'='*60}")

    for i in range(num_samples):
        sentence = SAMPLE_SENTENCES[i % len(SAMPLE_SENTENCES)]
        token_ids = text_to_token_ids(sentence, lexicon, token_map, max_len)

        t0 = time.time()
        mel = run_acoustic_model(sess, token_ids)
        elapsed = time.time() - t0

        # 每个输入保存到独立子目录
        np.save(os.path.join(sub_dirs["x"], f"acoustic_x_{i:04d}.npy"),
                token_ids.reshape(1, -1))
        np.save(os.path.join(sub_dirs["x_length"], f"acoustic_x_length_{i:04d}.npy"),
                np.array([max_len], dtype=np.int64))
        np.save(os.path.join(sub_dirs["noise_scale"], f"acoustic_noise_scale_{i:04d}.npy"),
                np.array([NOISE_SCALE], dtype=np.float32))
        np.save(os.path.join(sub_dirs["length_scale"], f"acoustic_length_scale_{i:04d}.npy"),
                np.array([LENGTH_SCALE], dtype=np.float32))

        actual_tokens = len([t for t in token_ids if t != 0])
        print(
            f"  [{i+1}/{num_samples}] \"{sentence}\" "
            f"→ {actual_tokens} tokens, mel={mel.shape}, {elapsed:.1f}s"
        )

    return mel.shape  # (1, 80, T)


def generate_vocoder_calibration(
    sess, mel_shape: tuple, num_samples: int, out_dir: str
):
    """生成声码器校准数据 — 使用声学模型输出的真实 mel"""
    os.makedirs(out_dir, exist_ok=True)
    print(f"\n{'='*60}")
    print(f"生成声码器校准数据 ({num_samples} 个样本, mel_shape={mel_shape})")
    print(f"{'='*60}")

    # 先运行一次声学模型拿到真实 mel 作为基准
    for i in range(num_samples):
        sentence = SAMPLE_SENTENCES[i % len(SAMPLE_SENTENCES)]
        # 重新生成 mel 用于声码器校准
        # (如果已有声学模型输出可以直接复用，这里为简单重新计算)
        t0 = time.time()
        # 用随机扰动产生多样性：在基准 mel 上加小噪声
        base_mel = np.random.randn(*mel_shape).astype(np.float32) * 0.1
        # 保存
        np.save(os.path.join(out_dir, f"vocoder_mels_{i:04d}.npy"), base_mel)
        elapsed = time.time() - t0
        print(f"  [{i+1}/{num_samples}] mel shape={base_mel.shape}, {elapsed:.2f}s")


def generate_vocoder_calibration_real(
    acoustic_sess, vocoder_sess, lexicon, token_map, max_len, num_samples, out_dir,
    mel_len=MAX_MEL_LEN
):
    """用声学模型的真实输出来生成声码器校准数据（更准确）
    mel 统一 pad/truncate 到 mel_len 帧，确保 BPU 输入 shape 固定
    """
    os.makedirs(out_dir, exist_ok=True)
    print(f"\n{'='*60}")
    print(f"生成声码器校准数据 — 使用声学模型真实 mel 输出")
    print(f"  固定 mel 帧数: {mel_len}")
    print(f"{'='*60}")

    fixed_shape = None
    for i in range(num_samples):
        sentence = SAMPLE_SENTENCES[i % len(SAMPLE_SENTENCES)]
        token_ids = text_to_token_ids(sentence, lexicon, token_map, max_len)

        t0 = time.time()
        mel = run_acoustic_model(acoustic_sess, token_ids)
        orig_shape = mel.shape

        # pad 或 truncate 到固定 mel 帧数
        if mel.shape[2] < mel_len:
            pad_w = mel_len - mel.shape[2]
            mel = np.pad(mel, ((0, 0), (0, 0), (0, pad_w)), mode="constant")
        elif mel.shape[2] > mel_len:
            mel = mel[:, :, :mel_len]

        fixed_shape = mel.shape

        # 添加微小扰动增加多样性
        noise = np.random.randn(*mel.shape).astype(np.float32) * 0.02
        mel_cal = (mel + noise).astype(np.float32)

        np.save(os.path.join(out_dir, f"vocoder_mels_{i:04d}.npy"), mel_cal)

        # 同时运行声码器验证（用固定长度 mel）
        outputs = vocoder_sess.run(None, {"mels": mel_cal})
        elapsed = time.time() - t0
        out_shapes = [o.shape for o in outputs]
        print(
            f"  [{i+1}/{num_samples}] \"{sentence}\" "
            f"→ mel orig={orig_shape} fixed={fixed_shape}, "
            f"vocoder_out={out_shapes}, {elapsed:.1f}s"
        )

    return fixed_shape


def verify_models(acoustic_sess, vocoder_sess, lexicon, token_map, max_len):
    """端到端验证：文本 → token → mel → audio"""
    print(f"\n{'='*60}")
    print("端到端推理验证")
    print(f"{'='*60}")

    sentence = SAMPLE_SENTENCES[0]
    token_ids = text_to_token_ids(sentence, lexicon, token_map, max_len)

    print(f"  文本: \"{sentence}\"")
    print(f"  Token IDs: {token_ids[:10]}... (共 {max_len} 个, 有效 {sum(1 for t in token_ids if t != 0)} 个)")

    # 声学模型
    t0 = time.time()
    mel = run_acoustic_model(acoustic_sess, token_ids)
    t1 = time.time()
    print(f"  声学模型: mel shape={mel.shape}, 耗时 {t1-t0:.2f}s")
    print(f"    mel 统计: min={mel.min():.4f}, max={mel.max():.4f}, mean={mel.mean():.4f}")

    # 声码器
    t0 = time.time()
    outputs = vocoder_sess.run(None, {"mels": mel})
    t1 = time.time()
    print(f"  声码器: 输出 {len(outputs)} 个张量, 耗时 {t1-t0:.2f}s")
    for j, o in enumerate(outputs):
        name = vocoder_sess.get_outputs()[j].name
        print(f"    [{name}] shape={o.shape}, min={o.min():.4f}, max={o.max():.4f}")

    print("\n  ✓ 模型验证通过")


def main():
    parser = argparse.ArgumentParser(description="为 Matcha-TTS 生成地平线 BPU 校准数据")
    parser.add_argument("--model-dir", default=DEFAULT_MODEL_DIR, help="模型目录")
    parser.add_argument("--out-dir", default=DEFAULT_OUT_DIR, help="校准数据输出目录")
    parser.add_argument("--num-samples", type=int, default=NUM_SAMPLES, help="校准样本数")
    parser.add_argument("--max-len", type=int, default=MAX_SEQ_LEN, help="固定 token 序列长度")
    args = parser.parse_args()

    import onnxruntime as ort

    model_dir = args.model_dir
    out_dir = args.out_dir
    acoustic_path = os.path.join(model_dir, "model-steps-3.onnx")
    vocoder_path = os.path.join(model_dir, "vocos-22khz-univ.onnx")
    tokens_path = os.path.join(model_dir, "tokens.txt")
    lexicon_path = os.path.join(model_dir, "lexicon.txt")

    # 检查文件
    for p in [acoustic_path, vocoder_path, tokens_path, lexicon_path]:
        if not os.path.exists(p):
            print(f"错误: 文件不存在 {p}")
            return 1

    # 加载词典和 token 映射
    token_map = load_tokens(tokens_path)
    lexicon = load_lexicon(lexicon_path)
    print(f"已加载: {len(token_map)} 个 token, {len(lexicon)} 个词条")
    print(f"声学模型: {acoustic_path}")
    print(f"声码器:   {vocoder_path}")
    print(f"序列长度: {args.max_len}, 样本数: {args.num_samples}")

    # 创建推理会话
    print("\n加载声学模型 (可能需要几秒)...")
    acoustic_sess = ort.InferenceSession(acoustic_path)
    print("加载声码器...")
    vocoder_sess = ort.InferenceSession(vocoder_path)

    # 1. 端到端验证
    verify_models(acoustic_sess, vocoder_sess, lexicon, token_map, args.max_len)

    # 2. 生成声学模型校准数据
    acoustic_cal_dir = os.path.join(out_dir, "acoustic_cal")
    mel_shape = generate_acoustic_calibration(
        acoustic_sess, lexicon, token_map, args.max_len, args.num_samples, acoustic_cal_dir
    )

    # 3. 生成声码器校准数据（使用声学模型的真实 mel 输出）
    vocoder_cal_dir = os.path.join(out_dir, "vocoder_cal")
    generate_vocoder_calibration_real(
        acoustic_sess, vocoder_sess, lexicon, token_map,
        args.max_len, args.num_samples, vocoder_cal_dir
    )

    # 4. 汇总
    print(f"\n{'='*60}")
    print("校准数据生成完成!")
    print(f"{'='*60}")
    print(f"  声学模型: {acoustic_cal_dir}/ ({args.num_samples} × 4 个 .npy)")
    print(f"    x:           (1, {args.max_len})  int64")
    print(f"    x_length:    (1,)         int64")
    print(f"    noise_scale: (1,)         float32")
    print(f"    length_scale:(1,)         float32")
    print(f"  声码器:   {vocoder_cal_dir}/ ({args.num_samples} 个 .npy)")
    print(f"    mels:        {mel_shape}  float32")
    print(f"\n固定参数: noise_scale={NOISE_SCALE}, length_scale={LENGTH_SCALE}")
    print(f"\n下一步: 将 horizon_convert/ 目录拷贝到 Docker workspace，运行 compile.sh")
    return 0


if __name__ == "__main__":
    exit(main())
