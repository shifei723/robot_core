#!/usr/bin/env python3
"""
实时关键词检测（KWS）Demo
基于 sherpa-onnx 的中文 Zipformer 模型，使用麦克风实时采集音频。

用法：
    uv run python kws_demo.py

默认使用 model/ 目录下的非量化模型；如要加速，可改用 *.int8.onnx。
"""

import argparse

import numpy as np
import pyaudio
import sherpa_onnx


def get_args():
    parser = argparse.ArgumentParser(
        formatter_class=argparse.ArgumentDefaultsHelpFormatter
    )
    parser.add_argument(
        "--tokens",
        type=str,
        default="model/tokens.txt",
        help="词表文件路径",
    )
    parser.add_argument(
        "--encoder",
        type=str,
        default="model/encoder-epoch-12-avg-2-chunk-16-left-64.onnx",
        help="Encoder ONNX 模型路径",
    )
    parser.add_argument(
        "--decoder",
        type=str,
        default="model/decoder-epoch-12-avg-2-chunk-16-left-64.onnx",
        help="Decoder ONNX 模型路径",
    )
    parser.add_argument(
        "--joiner",
        type=str,
        default="model/joiner-epoch-12-avg-2-chunk-16-left-64.onnx",
        help="Joiner ONNX 模型路径",
    )
    parser.add_argument(
        "--keywords-file",
        type=str,
        default="model/keywords.txt",
        help="关键词文件路径（ppinyin token 格式）",
    )
    parser.add_argument(
        "--keywords-score",
        type=float,
        default=1.0,
        help="关键词增强分数，越大越容易被检出",
    )
    parser.add_argument(
        "--keywords-threshold",
        type=float,
        default=0.25,
        help="触发阈值，越大越难触发",
    )
    parser.add_argument(
        "--num-trailing-blanks",
        type=int,
        default=1,
        help="关键词后空白帧数；token 重叠时可适当增大",
    )
    parser.add_argument(
        "--num-threads",
        type=int,
        default=2,
        help="ONNX 推理线程数",
    )
    parser.add_argument(
        "--provider",
        type=str,
        default="cpu",
        choices=["cpu", "cuda", "coreml"],
        help="推理后端",
    )
    parser.add_argument(
        "--sample-rate",
        type=int,
        default=16000,
        help="音频采样率（模型训练时固定 16kHz）",
    )
    return parser.parse_args()


def create_spotter(args):
    return sherpa_onnx.KeywordSpotter(
        tokens=args.tokens,
        encoder=args.encoder,
        decoder=args.decoder,
        joiner=args.joiner,
        keywords_file=args.keywords_file,
        num_threads=args.num_threads,
        sample_rate=args.sample_rate,
        keywords_score=args.keywords_score,
        keywords_threshold=args.keywords_threshold,
        num_trailing_blanks=args.num_trailing_blanks,
        provider=args.provider,
    )


def main():
    args = get_args()
    print("Initializing keyword spotter...")
    kws = create_spotter(args)
    stream = kws.create_stream()

    chunk_size = int(0.1 * args.sample_rate)  # 100 ms
    pa = pyaudio.PyAudio()
    mic = pa.open(
        format=pyaudio.paInt16,
        channels=1,
        rate=args.sample_rate,
        input=True,
        frames_per_buffer=chunk_size,
    )

    print(f"Listening at {args.sample_rate} Hz, chunk={chunk_size} samples")
    print("Press Ctrl+C to stop.\n")

    try:
        while True:
            data = mic.read(chunk_size, exception_on_overflow=False)
            samples = np.frombuffer(data, dtype=np.int16).astype(np.float32) / 32768.0
            stream.accept_waveform(args.sample_rate, samples)

            while kws.is_ready(stream):
                kws.decode_stream(stream)

            result = kws.get_result(stream)
            if result:
                print(f"Detected keyword: {result}")
                kws.reset_stream(stream)
    except KeyboardInterrupt:
        print("\nStopped by user.")
    finally:
        mic.stop_stream()
        mic.close()
        pa.terminate()


if __name__ == "__main__":
    main()
