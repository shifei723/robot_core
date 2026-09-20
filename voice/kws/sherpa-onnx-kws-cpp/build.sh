#!/bin/bash
set -e

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
BUILD_DIR="$SCRIPT_DIR/build"

echo "=== Building sherpa-onnx-kws-cpp ==="
echo "Platform: $(uname -s) $(uname -m)"

# 检查项目内 SDK 是否存在
SDK_DIR="$SCRIPT_DIR/sherpa-onnx-v1.13.3-linux-aarch64-shared-cpu"
if [ -d "$SDK_DIR/lib" ] && [ -f "$SDK_DIR/include/sherpa-onnx/c-api/c-api.h" ]; then
    echo "SDK found: $SDK_DIR"
    SHERPA_ONNX_SDK_PATH="$SDK_DIR"
elif [ -n "$SHERPA_ONNX_SDK_PATH" ]; then
    echo "Using user-specified SDK: $SHERPA_ONNX_SDK_PATH"
else
    echo "[ERROR] SDK not found!" >&2
    echo "Expected: $SDK_DIR" >&2
    echo "Or set SHERPA_ONNX_SDK_PATH=/path/to/sdk" >&2
    exit 1
fi

# 检查 PortAudio
if ! dpkg -s portaudio19-dev >/dev/null 2>&1; then
    echo "[WARN] portaudio19-dev may not be installed."
    echo "       Install with: sudo apt install -y portaudio19-dev"
fi

mkdir -p "$BUILD_DIR"
cd "$BUILD_DIR"

cmake "$SCRIPT_DIR" \
    -DCMAKE_BUILD_TYPE=Release \
    -DSHERPA_ONNX_SDK_PATH="$SHERPA_ONNX_SDK_PATH"

make -j$(nproc 2>/dev/null || echo 2)

echo ""
echo "=== Build successful ==="
echo "Executable: $BUILD_DIR/kws_vad_demo"
echo ""
echo "Run with:"
echo "  cd $SCRIPT_DIR"
echo "  ./build/kws_vad_demo --keywords-file keywords_xiaofei.txt --model-dir model"
