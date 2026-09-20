#!/usr/bin/env python3
"""
TTS 模块 - 供唤醒词流水线调用

用法:
  from tts_module import MatchaTTS

  tts = MatchaTTS(model_dir="./matcha-icefall-zh-baker")
  tts.speak("你好，请问有什么可以帮您？", output="reply.wav")
"""

import os
import sys
import time
import logging

try:
    import sherpa_onnx
    import soundfile as sf
except ImportError:
    print("请安装: pip install sherpa-onnx soundfile")
    sys.exit(1)

logger = logging.getLogger(__name__)


class MatchaTTS:
    """Matcha TTS 中文语音合成器"""

    def __init__(
        self,
        model_dir: str = "./matcha-icefall-zh-baker",
        vocoder: str = None,
        num_threads: int = 2,
        speed: float = 1.0,
    ):
        """
        初始化 TTS

        Args:
            model_dir: 模型目录路径
            vocoder: 声码器路径，默认在 model_dir 中查找
            num_threads: 推理线程数
            speed: 语速
        """
        self.model_dir = model_dir
        self.speed = speed

        if vocoder is None:
            vocoder = os.path.join(model_dir, "vocos-22khz-univ.onnx")

        # 验证文件
        for name in ["model-steps-3.onnx", "lexicon.txt", "tokens.txt",
                      "phone.fst", "date.fst", "number.fst"]:
            path = os.path.join(model_dir, name)
            if not os.path.exists(path):
                raise FileNotFoundError(f"模型文件缺失: {path}")

        dict_dir = os.path.join(model_dir, "dict")
        if not os.path.isdir(dict_dir):
            raise FileNotFoundError(f"字典目录缺失: {dict_dir}")

        # 构建配置
        config = sherpa_onnx.OfflineTtsConfig(
            model=sherpa_onnx.OfflineTtsModelConfig(
                matcha=sherpa_onnx.OfflineTtsMatchaModelConfig(
                    acoustic_model=os.path.join(model_dir, "model-steps-3.onnx"),
                    vocoder=vocoder,
                    lexicon=os.path.join(model_dir, "lexicon.txt"),
                    tokens=os.path.join(model_dir, "tokens.txt"),
                ),
                num_threads=num_threads,
                debug=False,
                provider="cpu",
            ),
            rule_fsts=",".join([
                os.path.join(model_dir, "phone.fst"),
                os.path.join(model_dir, "date.fst"),
                os.path.join(model_dir, "number.fst"),
            ]),
            max_num_sentences=1,
        )

        if not config.validate():
            raise ValueError("TTS 配置无效")

        self._tts = sherpa_onnx.OfflineTts(config)
        logger.info("MatchaTTS 初始化完成")

    def generate(self, text: str, speed: float = None):
        """
        生成语音

        Args:
            text: 输入文本
            speed: 语速覆盖

        Returns:
            (samples, sample_rate) 元组
        """
        gen_config = sherpa_onnx.GenerationConfig()
        gen_config.sid = 0
        gen_config.speed = speed if speed is not None else self.speed
        gen_config.silence_scale = 0.2

        start = time.time()
        audio = self._tts.generate(text, gen_config)
        elapsed = time.time() - start

        if len(audio.samples) == 0:
            raise RuntimeError("语音生成失败")

        duration = len(audio.samples) / audio.sample_rate
        logger.debug(
            f"TTS 生成: {duration:.2f}s 音频, 耗时 {elapsed:.2f}s, "
            f"RTF={elapsed/duration:.3f}"
        )

        return audio.samples, audio.sample_rate

    def speak(self, text: str, output: str = "tts_output.wav", speed: float = None):
        """
        生成语音并保存到 WAV 文件

        Args:
            text: 输入文本
            output: 输出 WAV 路径
            speed: 语速覆盖

        Returns:
            输出文件路径
        """
        samples, sample_rate = self.generate(text, speed)
        sf.write(output, samples, samplerate=sample_rate, subtype="PCM_16")
        logger.info(f"已保存到 {output}")
        return output


if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser(description="Matcha TTS 模块测试")
    parser.add_argument("text", help="要合成的文本")
    parser.add_argument("--output", "-o", default="test_tts.wav")
    parser.add_argument("--model-dir", default="./matcha-icefall-zh-baker")
    parser.add_argument("--speed", type=float, default=1.0)
    args = parser.parse_args()

    logging.basicConfig(level=logging.DEBUG)
    tts = MatchaTTS(model_dir=args.model_dir, speed=args.speed)
    tts.speak(args.text, output=args.output)
