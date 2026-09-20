#!/usr/bin/env python3
"""
Matcha TTS 中文语音合成脚本
基于 sherpa-onnx，支持自定义文本、语速、输出文件

用法:
  python3 tts.py "你好，欢迎使用语音合成系统"
  python3 tts.py --output=hello.wav --speed=1.2 "你好世界"
  python3 tts.py --num-threads=4 "很长的文本..."
"""

import argparse
import time
import os
import sys

try:
    import sherpa_onnx
    import soundfile as sf
except ImportError:
    print("错误: 请先安装依赖")
    print("  pip install sherpa-onnx soundfile")
    sys.exit(1)


def get_args():
    parser = argparse.ArgumentParser(
        description="Matcha TTS 中文语音合成",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    parser.add_argument(
        "text",
        type=str,
        help="要合成的文本",
    )
    parser.add_argument(
        "--output", "-o",
        type=str,
        default="tts_output.wav",
        help="输出 WAV 文件路径",
    )
    parser.add_argument(
        "--speed",
        type=float,
        default=1.0,
        help="语速，越大越快，越小越慢",
    )
    parser.add_argument(
        "--num-threads",
        type=int,
        default=2,
        help="线程数",
    )
    parser.add_argument(
        "--model-dir",
        type=str,
        default="./matcha-icefall-zh-baker",
        help="模型目录路径",
    )
    parser.add_argument(
        "--vocoder",
        type=str,
        default="./matcha-icefall-zh-baker/vocos-22khz-univ.onnx",
        help="声码器模型路径",
    )
    parser.add_argument(
        "--debug",
        action="store_true",
        help="显示调试信息",
    )
    return parser.parse_args()


def main():
    args = get_args()

    model_dir = args.model_dir
    vocoder = args.vocoder

    # 检查模型文件是否存在
    required_files = [
        os.path.join(model_dir, "model-steps-3.onnx"),
        vocoder,
        os.path.join(model_dir, "lexicon.txt"),
        os.path.join(model_dir, "tokens.txt"),
        os.path.join(model_dir, "phone.fst"),
        os.path.join(model_dir, "date.fst"),
        os.path.join(model_dir, "number.fst"),
    ]
    for f in required_files:
        if not os.path.exists(f):
            print(f"错误: 模型文件不存在: {f}")
            sys.exit(1)

    dict_dir = os.path.join(model_dir, "dict")
    if not os.path.isdir(dict_dir):
        print(f"错误: 字典目录不存在: {dict_dir}")
        sys.exit(1)

    # 构建 TTS 配置
    tts_config = sherpa_onnx.OfflineTtsConfig(
        model=sherpa_onnx.OfflineTtsModelConfig(
            matcha=sherpa_onnx.OfflineTtsMatchaModelConfig(
                acoustic_model=os.path.join(model_dir, "model-steps-3.onnx"),
                vocoder=vocoder,
                lexicon=os.path.join(model_dir, "lexicon.txt"),
                tokens=os.path.join(model_dir, "tokens.txt"),
            ),
            num_threads=args.num_threads,
            debug=args.debug,
            provider="cpu",
        ),
        rule_fsts=",".join([
            os.path.join(model_dir, "phone.fst"),
            os.path.join(model_dir, "date.fst"),
            os.path.join(model_dir, "number.fst"),
        ]),
        max_num_sentences=1,
    )

    if not tts_config.validate():
        print("错误: TTS 配置无效，请检查模型路径")
        sys.exit(1)

    # 创建 TTS 实例
    tts = sherpa_onnx.OfflineTts(tts_config)

    # 生成语音
    print(f"合成文本: {args.text}")
    print(f"语速: {args.speed}")

    start = time.time()
    gen_config = sherpa_onnx.GenerationConfig()
    gen_config.sid = 0
    gen_config.speed = args.speed
    gen_config.silence_scale = 0.2

    audio = tts.generate(args.text, gen_config)
    elapsed = time.time() - start

    if len(audio.samples) == 0:
        print("错误: 语音生成失败")
        sys.exit(1)

    # 保存 WAV
    sf.write(
        args.output,
        audio.samples,
        samplerate=audio.sample_rate,
        subtype="PCM_16",
    )

    duration = len(audio.samples) / audio.sample_rate
    rtf = elapsed / duration if duration > 0 else 0

    print(f"已保存到: {args.output}")
    print(f"音频时长: {duration:.2f}s")
    print(f"耗时: {elapsed:.2f}s")
    print(f"RTF: {rtf:.3f}")


if __name__ == "__main__":
    main()
