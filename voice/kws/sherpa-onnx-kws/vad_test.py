#!/usr/bin/env python3
"""
离线 VAD 测试：读取 WAV 文件，输出语音段落。

用法：
    uv run python vad_test.py --wav model/test_wavs/3.wav
"""

import argparse
import wave

import numpy as np
import sherpa_onnx


def get_args():
    parser = argparse.ArgumentParser(
        formatter_class=argparse.ArgumentDefaultsHelpFormatter
    )
    parser.add_argument("--wav", type=str, required=True)
    parser.add_argument("--vad-model", type=str, default="vad_model/silero_vad.onnx")
    parser.add_argument("--sample-rate", type=int, default=16000)
    parser.add_argument("--threshold", type=float, default=0.5)
    parser.add_argument("--min-silence-duration", type=float, default=0.5)
    parser.add_argument("--min-speech-duration", type=float, default=0.25)
    parser.add_argument("--window-size", type=int, default=512)
    return parser.parse_args()


def create_vad_config(args):
    silero_cfg = sherpa_onnx.SileroVadModelConfig(
        model=args.vad_model,
        threshold=args.threshold,
        min_silence_duration=args.min_silence_duration,
        min_speech_duration=args.min_speech_duration,
        window_size=args.window_size,
        max_speech_duration=20.0,
    )
    return sherpa_onnx.VadModelConfig(
        silero_vad=silero_cfg,
        sample_rate=args.sample_rate,
        num_threads=2,
        provider="cpu",
    )


def read_wave(path):
    with wave.open(path) as f:
        assert f.getnchannels() == 1 and f.getsampwidth() == 2
        samples = f.readframes(f.getnframes())
        return np.frombuffer(samples, dtype=np.int16).astype(np.float32) / 32768


def main():
    args = get_args()
    vad_cfg = create_vad_config(args)
    vad = sherpa_onnx.VoiceActivityDetector(vad_cfg, buffer_size_in_seconds=60)
    samples = read_wave(args.wav)

    vad.accept_waveform(samples)
    vad.flush()

    segments = []
    while not vad.empty():
        seg = vad.front
        start = seg.start
        end = seg.start + len(seg.samples)
        segments.append((start, end))
        vad.pop()

    print(f"Found {len(segments)} speech segment(s):")
    for start, end in segments:
        print(f"  {start / args.sample_rate:.2f}s - {end / args.sample_rate:.2f}s "
              f"({(end - start) / args.sample_rate:.2f}s)")


if __name__ == "__main__":
    main()
