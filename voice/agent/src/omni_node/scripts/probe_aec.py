#!/usr/bin/env python3
"""probe_aec.py — 实测设备是否有硬件回音消除(AEC)

方法: 分三段测量麦克风输入能量
  A. 静默基线    : 什么都不放，录 3 秒
  B. 播放期间    : 用扬声器放测试音，同时录 3 秒
  C. 播放后基线  : 停止播放，再录 3 秒
判定: 若 B 的能量 ≈ A/C，说明 AEC 有效（听不到自己）；
      若 B 明显高于基线，说明没有 AEC 或未生效。
"""
import math
import os
import struct
import subprocess
import sys
import threading
import time
import wave

DUR = 3
RATE = 16000
TONE_WAV = "/tmp/aec_tone.wav"


def make_tone(path, seconds=8, freq=440, amp=22000, rate=48000):
    """生成一段单频测试音（比语音更容易量化对比）"""
    w = wave.open(path, "w")
    w.setnchannels(1)
    w.setsampwidth(2)
    w.setframerate(rate)
    frames = b"".join(
        struct.pack("<h", int(amp * math.sin(2 * math.pi * freq * i / rate)))
        for i in range(rate * seconds))
    w.writeframes(frames)
    w.close()


def record(path, seconds=DUR):
    subprocess.run(
        ["arecord", "-D", "pulse", "-f", "S16_LE", "-r", str(RATE),
         "-c", "1", "-d", str(seconds), path],
        capture_output=True, timeout=seconds + 10)


def rms(path):
    w = wave.open(path)
    n = w.getnframes()
    if n == 0:
        return 0.0
    data = struct.unpack("<%dh" % n, w.readframes(n))
    mean = sum(data) / n
    return math.sqrt(sum((x - mean) ** 2 for x in data) / n)


def main():
    make_tone(TONE_WAV)

    print("A. 静默基线 (3秒，请保持安静)...")
    record("/tmp/aec_a.wav")
    a = rms("/tmp/aec_a.wav")
    print(f"   基线 RMS = {a:.1f}")

    print("B. 播放测试音并同时录音 (3秒)...")
    player = subprocess.Popen(
        ["aplay", "-D", "pulse", TONE_WAV],
        stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    time.sleep(0.8)          # 等扬声器真正出声
    record("/tmp/aec_b.wav")
    b = rms("/tmp/aec_b.wav")
    player.terminate()
    try:
        player.wait(timeout=3)
    except subprocess.TimeoutExpired:
        player.kill()
    print(f"   播放期间 RMS = {b:.1f}")

    time.sleep(1.5)
    print("C. 播放后基线 (3秒)...")
    record("/tmp/aec_c.wav")
    c = rms("/tmp/aec_c.wav")
    print(f"   播放后基线 RMS = {c:.1f}")

    base = max(a, c, 1.0)
    ratio = b / base
    print("\n═══ 判定 ═══")
    print(f"播放期间 / 基线 = {ratio:.1f} 倍")
    if ratio < 2.0:
        print("结论: AEC 有效 —— 播放自己的声音基本没被录进去")
        print("      => 静默窗口可以关掉 (--ack-blank-ms 0)")
    elif ratio < 6.0:
        print("结论: 部分抑制 —— 有一定消除但仍能听到自己")
        print("      => 建议保留较短的静默窗口")
    else:
        print("结论: 无 AEC —— 自己的声音被完整录了进去")
        print("      => 必须保留静默窗口，否则会把应答当成命令")
    sys.stdout.flush()
    os._exit(0)


if __name__ == "__main__":
    main()
