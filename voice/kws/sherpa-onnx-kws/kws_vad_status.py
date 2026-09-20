#!/usr/bin/env python3
"""
语音唤醒 + VAD 命令检测 + 状态输出 Demo

流程：
1. 持续监听麦克风，使用 KWS 检测唤醒词"你好小飞"。
2. 检测到唤醒词后，切换为 VAD 模式，捕获后续语音命令。
3. 当检测到足够长时间的静音时，认为命令结束，保存为 WAV 文件。
4. 只保留最新一条命令，文件名固定为 latest_command.wav，新命令直接覆盖。
5. 在 commands/kws_status 文件中实时输出唤醒状态：
   - 未唤醒（IDLE）：写入 0
   - 已唤醒（LISTENING，VAD 进行中）：写入 1

状态文件路径：commands/kws_status（纯文本，内容为 "0" 或 "1"）

用法：
    python kws_vad_status_demo.py

保存的命令音频默认在 commands/latest_command.wav。
"""

import argparse
import os
import time
import wave

import numpy as np
import pyaudio
import sherpa_onnx


def get_args():
    parser = argparse.ArgumentParser(
        formatter_class=argparse.ArgumentDefaultsHelpFormatter
    )
    # KWS 参数
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
    parser.add_argument(
        "--keywords-file",
        type=str,
        default="keywords_xiaofei.txt",
        help="唤醒词文件",
    )
    parser.add_argument("--keywords-score", type=float, default=1.0)
    parser.add_argument("--keywords-threshold", type=float, default=0.25)
    parser.add_argument("--num-trailing-blanks", type=int, default=1)

    # VAD 参数
    parser.add_argument(
        "--vad-model",
        type=str,
        default="vad_model/silero_vad.onnx",
        help="Silero VAD ONNX 模型路径",
    )
    parser.add_argument("--vad-threshold", type=float, default=0.5)
    parser.add_argument("--vad-min-silence-duration", type=float, default=0.5)
    parser.add_argument("--vad-min-speech-duration", type=float, default=0.25)
    parser.add_argument("--vad-window-size", type=int, default=512)
    parser.add_argument("--vad-max-speech-duration", type=float, default=20.0)

    # 公共参数
    parser.add_argument("--num-threads", type=int, default=2)
    parser.add_argument("--provider", type=str, default="cpu")
    parser.add_argument("--sample-rate", type=int, default=16000)
    parser.add_argument(
        "--kws-chunk-size",
        type=int,
        default=int(0.1 * 16000),
        help="KWS 每次读取的采样点数（默认 100ms）",
    )
    parser.add_argument(
        "--max-command-duration",
        type=float,
        default=30.0,
        help="唤醒后最长等待命令的时间（秒）",
    )
    parser.add_argument(
        "--output-dir",
        type=str,
        default="commands",
        help="保存命令音频的目录",
    )
    parser.add_argument(
        "--status-file",
        type=str,
        default="kws_status",
        help="状态文件名（保存在 output-dir 下，内容为 0 或 1）",
    )
    parser.add_argument(
        "--wav",
        type=str,
        default=None,
        help="用于离线测试的 WAV 文件路径（不提供则使用麦克风）",
    )
    return parser.parse_args()


def create_kws(args):
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


def create_vad_config(args):
    silero_cfg = sherpa_onnx.SileroVadModelConfig(
        model=args.vad_model,
        threshold=args.vad_threshold,
        min_silence_duration=args.vad_min_silence_duration,
        min_speech_duration=args.vad_min_speech_duration,
        window_size=args.vad_window_size,
        max_speech_duration=args.vad_max_speech_duration,
    )
    vad_cfg = sherpa_onnx.VadModelConfig(
        silero_vad=silero_cfg,
        sample_rate=args.sample_rate,
        num_threads=args.num_threads,
        provider=args.provider,
    )
    if not vad_cfg.validate():
        raise RuntimeError("Invalid VAD config")
    return vad_cfg


def save_wav(path, samples, sample_rate):
    int16 = (samples * 32767).astype(np.int16)
    with wave.open(path, "wb") as f:
        f.setnchannels(1)
        f.setsampwidth(2)
        f.setframerate(sample_rate)
        f.writeframes(int16.tobytes())


def write_status(status_path, value):
    """将状态值（0 或 1）原子写入状态文件。

    先写临时文件再 rename，避免其他进程读到半写内容。
    """
    tmp_path = status_path + ".tmp"
    with open(tmp_path, "w") as f:
        f.write(str(value))
    os.replace(tmp_path, status_path)


def main():
    args = get_args()
    os.makedirs(args.output_dir, exist_ok=True)

    status_path = os.path.join(args.output_dir, args.status_file)
    # 初始化状态为 0（未唤醒）
    write_status(status_path, 0)

    print("Loading KWS model...")
    kws = create_kws(args)
    print("Loading VAD model...")
    vad = sherpa_onnx.VoiceActivityDetector(
        create_vad_config(args), buffer_size_in_seconds=60
    )

    kws_stream = kws.create_stream()

    if args.wav:
        wav_file = wave.open(args.wav)
        if (
            wav_file.getnchannels() != 1
            or wav_file.getsampwidth() != 2
            or wav_file.getframerate() != args.sample_rate
        ):
            raise ValueError(f"--wav must be mono 16-bit {args.sample_rate} Hz")
        pa = mic = None
        print(f"Using WAV file for offline test: {args.wav}")
    else:
        wav_file = None
        pa = pyaudio.PyAudio()
        mic = pa.open(
            format=pyaudio.paInt16,
            channels=1,
            rate=args.sample_rate,
            input=True,
            frames_per_buffer=args.kws_chunk_size,
        )

    def read_chunk():
        if wav_file:
            frames = wav_file.readframes(args.kws_chunk_size)
            if not frames:
                return None
            return frames
        return mic.read(args.kws_chunk_size, exception_on_overflow=False)

    state = "IDLE"  # IDLE -> LISTENING -> IDLE
    listening_start = 0
    command_count = 0

    # 固定输出文件名
    output_path = os.path.join(args.output_dir, "latest_command.wav")

    def finalize_command():
        nonlocal command_count
        vad.flush()
        if not vad.empty():
            seg = vad.front
            segment = np.array(seg.samples, dtype=np.float32)
            if len(segment) > 0:
                command_count += 1
                save_wav(output_path, segment, args.sample_rate)
                duration = len(segment) / args.sample_rate
                print(
                    f"[VAD] 命令音频已保存: {output_path} ({duration:.2f}s) [覆盖更新]\n"
                )
            else:
                print("[VAD] 未检测到有效语音命令\n")
        else:
            print("[VAD] 未检测到有效语音命令\n")
        vad.reset()
        # VAD 结束，状态回到未唤醒
        write_status(status_path, 0)
        print(f"[STATUS] 状态已更新: 0 (未唤醒) -> {status_path}\n")
        return kws.create_stream()

    print(f"Listening for wake word from keywords: {args.keywords_file}")
    print(f"Status file: {status_path} (0=未唤醒, 1=已唤醒)")
    if wav_file is None:
        print("Say '你好小飞', then speak your command. Press Ctrl+C to stop.\n")
    else:
        print("Processing file chunks...\n")

    try:
        while True:
            data = read_chunk()
            if data is None:
                if state == "LISTENING":
                    print("[VAD] 文件结束，尝试收尾\n")
                    kws_stream = finalize_command()
                    state = "IDLE"
                break

            samples = np.frombuffer(data, dtype=np.int16).astype(np.float32) / 32768.0

            if state == "IDLE":
                kws_stream.accept_waveform(args.sample_rate, samples)
                while kws.is_ready(kws_stream):
                    kws.decode_stream(kws_stream)

                result = kws.get_result(kws_stream)
                if result:
                    print(f"[KWS] 唤醒词检测到: {result}")
                    # 唤醒，状态置 1
                    write_status(status_path, 1)
                    print(f"[STATUS] 状态已更新: 1 (已唤醒) -> {status_path}")
                    kws.reset_stream(kws_stream)
                    vad.reset()
                    state = "LISTENING"
                    listening_start = time.time()
            else:
                vad.accept_waveform(samples)

                if not vad.empty():
                    kws_stream = finalize_command()
                    state = "IDLE"
                    continue

                elapsed = time.time() - listening_start
                if elapsed >= args.max_command_duration:
                    print("[VAD] 等待命令超时\n")
                    kws_stream = finalize_command()
                    state = "IDLE"

    except KeyboardInterrupt:
        print("\nStopped by user.")
    finally:
        # 退出时确保状态归 0
        write_status(status_path, 0)
        if mic:
            mic.stop_stream()
            mic.close()
        if pa:
            pa.terminate()
        if wav_file:
            wav_file.close()


if __name__ == "__main__":
    main()
