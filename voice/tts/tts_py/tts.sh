#!/bin/bash
# Matcha TTS 快捷脚本
# 用法: ./tts.sh "要合成的文本" [输出文件]

set -e

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
VENV_DIR="$SCRIPT_DIR/venv"
TTS_SCRIPT="$SCRIPT_DIR/tts.py"

TEXT="${1:?用法: $0 \"要合成的文本\" [输出文件]}"
OUTPUT="${2:-tts_output.wav}"

# 激活虚拟环境
if [ -d "$VENV_DIR" ]; then
    source "$VENV_DIR/bin/activate"
else
    echo "错误: 虚拟环境不存在: $VENV_DIR"
    echo "请先运行: python3 -m venv $VENV_DIR && source $VENV_DIR/bin/activate && pip install sherpa-onnx soundfile"
    exit 1
fi

# 运行 TTS
python3 "$TTS_SCRIPT" --output "$OUTPUT" "$TEXT"
