#!/usr/bin/env python3
"""
将普通中文关键词文本转换为 sherpa-onnx 需要的 ppinyin token 格式。

用法：
    uv run python generate_keywords.py \
        --tokens model/tokens.txt \
        --input keywords_raw.txt \
        --output keywords.txt

输入文件格式（每行一个）：
    关键词 @显示名称

也可以直接调用 sherpa-onnx-cli：
    sherpa-onnx-cli text2token \
        --tokens model/tokens.txt \
        --tokens-type ppinyin \
        keywords_raw.txt keywords.txt
"""

import argparse
import subprocess
import sys
from pathlib import Path


def get_args():
    parser = argparse.ArgumentParser(
        formatter_class=argparse.ArgumentDefaultsHelpFormatter
    )
    parser.add_argument("--tokens", type=str, default="model/tokens.txt")
    parser.add_argument("--input", type=str, default="keywords_raw.txt")
    parser.add_argument("--output", type=str, default="keywords.txt")
    parser.add_argument(
        "--tokens-type",
        type=str,
        default="ppinyin",
        help="中文模型通常用 ppinyin（带声调拼音）",
    )
    return parser.parse_args()


def main():
    args = get_args()
    cmd = [
        "sherpa-onnx-cli",
        "text2token",
        "--tokens",
        args.tokens,
        "--tokens-type",
        args.tokens_type,
        args.input,
        args.output,
    ]
    print("Running:", " ".join(cmd))
    subprocess.run(cmd, check=True)
    print(f"Generated {args.output}")


if __name__ == "__main__":
    main()
