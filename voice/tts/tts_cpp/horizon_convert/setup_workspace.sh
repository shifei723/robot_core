#!/bin/bash
# setup_workspace.sh - 初始化 Docker workspace 目录结构
#
# 用法: bash setup_workspace.sh [model_dir]
#   model_dir: 模型目录路径 (默认: ../matcha-icefall-zh-baker)
#
# 此脚本会:
#   1. 创建 output/onnx/ 目录
#   2. 将 ONNX 模型拷贝/链接到 output/onnx/
#   3. 验证所有文件完整性

set -e

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
MODEL_DIR="${1:-$(dirname "$SCRIPT_DIR")/matcha-icefall-zh-baker}"
ONNX_DIR="${SCRIPT_DIR}/output/onnx"

RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m'

log()  { echo -e "${GREEN}[INFO]${NC} $*"; }
warn() { echo -e "${YELLOW}[WARN]${NC} $*"; }
err()  { echo -e "${RED}[ERROR]${NC} $*"; }

# 检查模型目录
if [ ! -d "$MODEL_DIR" ]; then
    err "模型目录不存在: $MODEL_DIR"
    echo "用法: bash setup_workspace.sh [模型目录路径]"
    exit 1
fi

log "模型目录: $MODEL_DIR"

# 创建 output/onnx 目录
mkdir -p "$ONNX_DIR"

# 拷贝或链接 ONNX 模型
for f in model-steps-3.onnx vocos-22khz-univ.onnx; do
    src="${MODEL_DIR}/${f}"
    dst="${ONNX_DIR}/${f}"
    if [ ! -f "$src" ]; then
        err "ONNX 模型不存在: $src"
        exit 1
    fi
    if [ ! -f "$dst" ]; then
        # 优先用硬链接（同文件系统）；失败则用软链接
        if ln "$src" "$dst" 2>/dev/null; then
            log "硬链接: $f"
        elif ln -sf "$src" "$dst"; then
            log "软链接: $f"
        else
            cp "$src" "$dst"
            log "拷贝: $f"
        fi
    else
        log "已存在: $f"
    fi
done

# 验证
log "验证 workspace 完整性..."

check_file() {
    local path="$1"
    local desc="$2"
    if [ -f "$path" ] || [ -L "$path" ]; then
        local size
        size=$(ls -lh "$path" | awk '{print $5}')
        echo "  ✓ $desc ($size)"
    else
        echo "  ✗ $desc — 缺失!"
        return 1
    fi
}

echo ""
echo "ONNX 模型:"
check_file "${ONNX_DIR}/model-steps-3.onnx" "声学模型"
check_file "${ONNX_DIR}/vocos-22khz-univ.onnx" "声码器"

echo ""
echo "YAML 配置:"
check_file "${SCRIPT_DIR}/acoustic_config.yaml" "声学模型配置"
check_file "${SCRIPT_DIR}/vocoder_config.yaml" "声码器配置"

echo ""
echo "校准数据:"
for sub in x x_length noise_scale length_scale; do
    count=$(ls "${SCRIPT_DIR}/calibration_data/acoustic_cal/${sub}/"*.npy 2>/dev/null | wc -l | tr -d ' ')
    echo "  acoustic_cal/${sub}/: ${count} 个样本"
done
count=$(ls "${SCRIPT_DIR}/calibration_data/vocoder_cal/"*.npy 2>/dev/null | wc -l | tr -d ' ')
echo "  vocoder_cal/: ${count} 个样本"

echo ""
log "Workspace 准备完成!"
echo ""
echo "下一步: 将此目录挂载到 Docker 容器:"
echo "  sudo docker run -d --network host --name xgs_OE_v3.7.0 \\"
echo "    -v /path/to/oe-package:/open_explorer \\"
echo "    -v ${SCRIPT_DIR}:/workspace -w /workspace \\"
echo "    ai_toolchain_ubuntu_22_s100_s600_cpu:v3.7.0 sleep infinity"
echo ""
echo "然后编译: bash compile.sh"
