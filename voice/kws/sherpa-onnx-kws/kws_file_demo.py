#!/usr/bin/env python3
"""
离线关键词检测（KWS）Demo
读取本地 WAV 文件进行检测，适用于快速验证模型与关键词文件。

用法：
    uv run python kws_file_demo.py --wav model/test_wavs/3.wav
"""

import argparse
import wave

import numpy as np
import sherpa_onnx


def get_args():
    parser = argparse.ArgumentParser(
        formatter_class=argparse.ArgumentDefaultsHelpFormatter
    )
    parser.add_argument("--tokens", type=str, default="model/tokens.txt")
    parser.add_argument(
        "--encoder",
        type=str,
        default="model/encoder-epoch-12-avg-2-chunk-16-left-64.onnx",
    )
    parser.add_argument(
        "--decoder",
        type=str,
        default="model/decoder-epoch-12-avg-2-chunk-16-left-64.onnx",
    )
    parser.add_argument(
        "--joiner",
        type=str,
        default="model/joiner-epoch-12-avg-2-chunk-16-left-64.onnx",
    )
    parser.add_argument("--keywords-file", type=str, default="model/keywords.txt")
    parser.add_argument("--wav", type=str, required=True, help="待检测的 16kHz 单声道 WAV 文件")
    parser.add_argument("--keywords-score", type=float, default=1.0)
    parser.add_argument("--keywords-threshold", type=float, default=0.25)
    parser.add_argument("--num-trailing-blanks", type=int, default=1)
    parser.add_argument("--num-threads", type=int, default=2)
    parser.add_argument("--provider", type=str, default="cpu")
    return parser.parse_args()


def read_wave(path):
    with wave.open(path) as f:
        assert f.getnchannels() == 1, f.getnchannels()
        assert f.getsampwidth() == 2, f.getsampwidth()
        num_samples = f.getnframes()
        samples = f.readframes(num_samples)
        samples_int16 = np.frombuffer(samples, dtype=np.int16)
        samples_float32 = samples_int16.astype(np.float32) / 32768
    return samples_float32, f.getframerate()


def main():
    args = get_args()
    kws = sherpa_onnx.KeywordSpotter(
        tokens=args.tokens,
        encoder=args.encoder,
        decoder=args.decoder,
        joiner=args.joiner,
        keywords_file=args.keywords_file,
        num_threads=args.num_threads,
        sample_rate=16000,
        keywords_score=args.keywords_score,
        keywords_threshold=args.keywords_threshold,
        num_trailing_blanks=args.num_trailing_blanks,
        provider=args.provider,
    )

    samples, sample_rate = read_wave(args.wav)
    tail_paddings = np.zeros(int(0.66 * sample_rate), dtype=np.float32)

    s = kws.create_stream()
    s.accept_waveform(sample_rate, samples)
    s.accept_waveform(sample_rate, tail_paddings)
    s.input_finished()

    detected = []
    while kws.is_ready(s):
        kws.decode_stream(s)
        r = kws.get_result(s)
        if r:
            detected.append(r)
            kws.reset_stream(s)

    if detected:
        print(f"Detected: {', '.join(detected)}")
    else:
        print("No keyword detected")


if __name__ == "__main__":
    main()
