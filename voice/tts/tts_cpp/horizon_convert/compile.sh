#!/bin/bash
# compile.sh - 在 Docker 容器内编译 Matcha-TTS ONNX 模型为地平线 BPU .hbm 格式
#
# 前提:
#   1. Docker 容器 xgs_OE_v3.7.0 已启动
#   2. 本目录 (horizon_convert/) 已挂载到 /workspace
#   3. 校准数据已通过 gen_calibration_data.py 生成
#
# 用法:
#   bash compile.sh              # 编译两个模型
#   bash compile.sh acoustic     # 只编译声学模型
#   bash compile.sh vocoder      # 只编译声码器
#   bash compile.sh check        # 只检查模型兼容性

set -e

CONTAINER="xgs_OE_v3.7.0"
WORKSPACE="/workspace"
ONNX_DIR="${WORKSPACE}/output/onnx"
OUTPUT_DIR="${WORKSPACE}/output"

# 颜色
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m'

log()  { echo -e "${GREEN}[INFO]${NC} $*"; }
warn() { echo -e "${YELLOW}[WARN]${NC} $*"; }
err()  { echo -e "${RED}[ERROR]${NC} $*"; }

# ── 检查 Docker 容器 ──
check_container() {
    if ! docker ps --format '{{.Names}}' | grep -q "^${CONTAINER}$"; then
        err "Docker 容器 ${CONTAINER} 未运行"
        echo "请先启动容器:"
        echo "  sudo docker start ${CONTAINER}"
        echo "或创建新容器:"
        echo "  sudo docker run -d --network host --name ${CONTAINER} \\"
        echo "    -v /path/to/oe-package:/open_explorer \\"
        echo "    -v \$(pwd):${WORKSPACE} -w ${WORKSPACE} \\"
        echo "    ai_toolchain_ubuntu_22_s100_s600_cpu:v3.7.0 sleep infinity"
        exit 1
    fi
    log "容器 ${CONTAINER} 运行中"
}

# ── 检查文件 ──
check_files() {
    log "检查 ONNX 模型和校准数据..."

    # ONNX 模型
    for f in model-steps-3.onnx vocos-22khz-univ.onnx; do
        if [ ! -f "${ONNX_DIR}/${f}" ]; then
            err "ONNX 模型不存在: ${ONNX_DIR}/${f}"
            echo "请确保模型文件已拷贝到 output/onnx/ 目录"
            exit 1
        fi
    done

    # 校准数据
    local cal_dir="${WORKSPACE}/calibration_data"
    for sub in acoustic_cal/x acoustic_cal/x_length acoustic_cal/noise_scale acoustic_cal/length_scale vocoder_cal; do
        local count=$(ls "${cal_dir}/${sub}"/*.npy 2>/dev/null | wc -l)
        if [ "$count" -eq 0 ]; then
            err "校准数据缺失: ${cal_dir}/${sub}/"
            echo "请先运行: python3 gen_calibration_data.py"
            exit 1
        fi
        log "  ${sub}: ${count} 个样本"
    done

    # YAML 配置
    for f in acoustic_config.yaml vocoder_config.yaml; do
        if [ ! -f "${WORKSPACE}/${f}" ]; then
            err "配置文件不存在: ${WORKSPACE}/${f}"
            exit 1
        fi
    done

    log "所有文件检查通过"
}

# ── 检查模型兼容性（不编译） ──
check_model() {
    log "检查声学模型 ONNX 兼容性..."
    docker exec ${CONTAINER} bash -c "
        python3 -c \"
import onnxruntime as ort
import numpy as np
sess = ort.InferenceSession('${ONNX_DIR}/model-steps-3.onnx')
print('声学模型输入:')
for inp in sess.get_inputs():
    print(f'  {inp.name}: {inp.shape} ({inp.type})')
print('声学模型输出:')
for out in sess.get_outputs():
    print(f'  {out.name}: {out.shape} ({out.type})')
print()

sess2 = ort.InferenceSession('${ONNX_DIR}/vocos-22khz-univ.onnx')
print('声码器输入:')
for inp in sess2.get_inputs():
    print(f'  {inp.name}: {inp.shape} ({inp.type})')
print('声码器输出:')
for out in sess2.get_outputs():
    print(f'  {out.name}: {out.shape} ({out.type})')
\"
    "
    log "模型检查完成"
}

# ── 编译声学模型 ──
compile_acoustic() {
    log "========================================="
    log "编译声学模型 (Matcha-TTS Acoustic Model)"
    log "  ONNX: model-steps-3.onnx (72MB)"
    log "  预计耗时: 30-90 分钟"
    log "========================================="

    local start=$(date +%s)
    docker exec ${CONTAINER} hb_compile --config ${WORKSPACE}/acoustic_config.yaml 2>&1
    local end=$(date +%s)
    local elapsed=$(( end - start ))

    local hbm="${OUTPUT_DIR}/acoustic_quantized_nash_m/matcha_acoustic.hbm"
    if [ -f "${hbm}" ]; then
        local size=$(du -h "${hbm}" | cut -f1)
        log "声学模型编译成功! 耗时: $((elapsed/60))分${elapsed%60}秒, HBM 大小: ${size}"
    else
        err "声学模型编译失败 — HBM 文件未生成"
        err "请检查上方日志中的错误信息"
        echo ""
        echo "常见问题:"
        echo "  1. int64 输入不支持: 需要修改 ONNX 将 x 转为 float32"
        echo "  2. ODE solver 循环不支持: 需要展开循环或拆分模型"
        echo "  3. 校准数据格式错误: 检查 .npy 文件的 shape 和 dtype"
        return 1
    fi
}

# ── 编译声码器 ──
compile_vocoder() {
    log "========================================="
    log "编译声码器 (Vocos Vocoder)"
    log "  ONNX: vocos-22khz-univ.onnx (51MB)"
    log "  预计耗时: 1-5 分钟"
    log "========================================="

    local start=$(date +%s)
    docker exec ${CONTAINER} hb_compile --config ${WORKSPACE}/vocoder_config.yaml 2>&1
    local end=$(date +%s)
    local elapsed=$(( end - start ))

    local hbm="${OUTPUT_DIR}/vocoder_quantized_nash_m/vocos_vocoder.hbm"
    if [ -f "${hbm}" ]; then
        local size=$(du -h "${hbm}" | cut -f1)
        log "声码器编译成功! 耗时: $((elapsed/60))分${elapsed%60}秒, HBM 大小: ${size}"
    else
        err "声码器编译失败 — HBM 文件未生成"
        err "请检查上方日志中的错误信息"
        return 1
    fi
}

# ── 查看编译结果 ──
show_results() {
    log "========================================="
    log "编译结果汇总"
    log "========================================="

    echo ""
    echo "声学模型:"
    local a_hbm="${OUTPUT_DIR}/acoustic_quantized_nash_m/matcha_acoustic.hbm"
    if [ -f "${a_hbm}" ]; then
        echo "  HBM: ${a_hbm} ($(du -h "${a_hbm}" | cut -f1))"
        docker exec ${CONTAINER} hrt_model_exec model_info --model_file="${a_hbm}" 2>/dev/null || true
    else
        echo "  未生成"
    fi

    echo ""
    echo "声码器:"
    local v_hbm="${OUTPUT_DIR}/vocoder_quantized_nash_m/vocos_vocoder.hbm"
    if [ -f "${v_hbm}" ]; then
        echo "  HBM: ${v_hbm} ($(du -h "${v_hbm}" | cut -f1))"
        docker exec ${CONTAINER} hrt_model_exec model_info --model_file="${v_hbm}" 2>/dev/null || true
    else
        echo "  未生成"
    fi
    echo ""
}

# ── 主流程 ──
main() {
    local target="${1:-all}"

    check_container
    check_files

    case "${target}" in
        acoustic)
            compile_acoustic
            ;;
        vocoder)
            compile_vocoder
            ;;
        check)
            check_model
            ;;
        all)
            check_model
            compile_vocoder     # 声码器快，先编译
            compile_acoustic    # 声学模型慢，后编译
            ;;
        *)
            echo "用法: bash compile.sh [all|acoustic|vocoder|check]"
            exit 1
            ;;
    esac

    show_results
}

main "$@"
