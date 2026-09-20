#!/bin/bash
# TTS 项目环境初始化脚本
# 支持 macOS 和 Ubuntu (ARM64/x64)
#
# 用法:
#   首次部署: bash setup.sh
#   日常使用: source venv/bin/activate

set -e

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
VENV_DIR="$SCRIPT_DIR/venv"
MODEL_DIR="$SCRIPT_DIR/matcha-icefall-zh-baker"

echo "=== Matcha TTS 环境初始化 ==="
echo "项目路径: $SCRIPT_DIR"

# 检测 Python
PYTHON=""
if command -v python3 &>/dev/null; then
    PYTHON="python3"
elif command -v python &>/dev/null; then
    PYTHON="python"
else
    echo "错误: 未找到 python3 或 python"
    echo "请先安装: sudo apt install python3 python3-pip python3-venv"
    exit 1
fi

PY_VERSION=$($PYTHON --version 2>&1 | awk '{print $2}')
echo "Python 版本: $PY_VERSION"

# 检查模型文件
echo ""
echo "=== 检查模型文件 ==="
REQUIRED_FILES=(
    "model-steps-3.onnx"
    "vocos-22khz-univ.onnx"
    "lexicon.txt"
    "tokens.txt"
    "phone.fst"
    "date.fst"
    "number.fst"
    "dict/jieba.dict.utf8"
)

ALL_OK=true
for f in "${REQUIRED_FILES[@]}"; do
    if [ -f "$MODEL_DIR/$f" ]; then
        SIZE=$(ls -lh "$MODEL_DIR/$f" | awk '{print $5}')
        echo "  OK: $f ($SIZE)"
    else
        echo "  缺失: $MODEL_DIR/$f"
        ALL_OK=false
    fi
done

if [ "$ALL_OK" = false ]; then
    echo ""
    echo "警告: 部分模型文件缺失，请检查模型目录"
    echo "模型下载方式见 DEPLOY.md 第 2 节"
fi

# 创建虚拟环境
echo ""
echo "=== 创建 Python 虚拟环境 ==="
if [ -d "$VENV_DIR" ]; then
    echo "虚拟环境已存在: $VENV_DIR"
else
    $PYTHON -m venv "$VENV_DIR"
    echo "虚拟环境已创建"
fi

# 激活并安装依赖
echo ""
echo "=== 安装 Python 依赖 ==="
source "$VENV_DIR/bin/activate"
pip install --upgrade pip -q
pip install sherpa-onnx soundfile -q

echo ""
echo "=== 安装完成 ==="
echo "sherpa-onnx: $(pip show sherpa-onnx 2>/dev/null | grep Version || echo '未安装')"
echo "soundfile: $(pip show soundfile 2>/dev/null | grep Version || echo '未安装')"

# 快速验证
echo ""
echo "=== 验证 TTS ==="
cd "$SCRIPT_DIR"
python3 tts.py "测试语音合成" --output="$SCRIPT_DIR/test_setup.wav" 2>&1 | tail -5

if [ -f "$SCRIPT_DIR/test_setup.wav" ]; then
    SIZE=$(ls -lh "$SCRIPT_DIR/test_setup.wav" | awk '{print $5}')
    echo "测试成功! 生成文件: test_setup.wav ($SIZE)"
    rm -f "$SCRIPT_DIR/test_setup.wav"
else
    echo "测试失败，请检查上方错误信息"
fi

echo ""
echo "=== 初始化完成 ==="
echo "使用方式:"
echo "  cd $SCRIPT_DIR"
echo "  source venv/bin/activate"
echo "  python3 tts.py \"你好世界\""
echo ""
echo "或直接使用快捷脚本:"
echo "  ./tts.sh \"你好世界\" output.wav"
