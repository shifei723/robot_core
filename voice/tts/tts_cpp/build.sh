#!/bin/bash
# build.sh - 自动下载 SDK 并编译
set -e

DIR="$(cd "$(dirname "$0")" && pwd)"
SDK="$DIR/sherpa-onnx-sdk"
VER="1.13.3"

detect() {
    case "$(uname -s)-$(uname -m)" in
        Darwin-arm64)  echo "osx-arm64" ;;
        Linux-aarch64) echo "linux-aarch64" ;;
        Linux-x86_64)  echo "linux-x64" ;;
        *) echo "unknown" ;;
    esac
}

sdk_url() {
    case "$1" in
        osx-arm64)     echo "https://github.com/k2-fsa/sherpa-onnx/releases/download/v${VER}/sherpa-onnx-v${VER}-osx-arm64-shared.tar.bz2" ;;
        linux-aarch64) echo "https://github.com/k2-fsa/sherpa-onnx/releases/download/v${VER}/sherpa-onnx-v${VER}-linux-aarch64-shared-cpu.tar.bz2" ;;
        linux-x64)     echo "https://github.com/k2-fsa/sherpa-onnx/releases/download/v${VER}/sherpa-onnx-v${VER}-linux-x64-shared.tar.bz2" ;;
    esac
}

PLAT=$(detect)
echo "=== TTS C++ 编译 ==="
echo "平台: $PLAT"

if [ "$1" = "--clean" ]; then
    rm -rf "$DIR/build" "$SDK"
    echo "已清理"
fi

if [ ! -d "$SDK" ]; then
    URL=$(sdk_url "$PLAT")
    echo "下载 SDK: $URL"
    TB="$DIR/sdk.tar.bz2"
    curl -L --progress-bar -o "$TB" "$URL" || {
        echo "下载失败，尝试镜像..."
        curl -L --progress-bar -o "$TB" "https://ghfast.top/$URL"
    }
    tar xf "$TB" -C "$DIR"
    EXTRACTED=$(find "$DIR" -maxdepth 1 -type d -name "sherpa-onnx-v*" | head -1)
    mv "$EXTRACTED" "$SDK"
    rm -f "$TB"
    echo "SDK 已安装"
fi

# ─── 检查 ZeroMQ ───
if [ -f /opt/homebrew/lib/libzmq.dylib ] || [ -f /usr/local/lib/libzmq.dylib ] || [ -f /usr/lib/libzmq.so ]; then
    echo "ZeroMQ: 已安装"
else
    echo "ZeroMQ: 未安装"
    if [ "$(uname -s)" = "Darwin" ]; then
        echo "  安装: brew install zeromq"
    else
        echo "  安装: sudo apt install libzmq3-dev"
    fi
    echo "  (tts_stream 需要 ZeroMQ，tts 不需要)"
fi

# ─── 编译 ───
mkdir -p "$DIR/build" && cd "$DIR/build"
cmake -DCMAKE_BUILD_TYPE=Release -DSHERPA_ONNX_SDK="$SDK" "$DIR"
make -j$(nproc 2>/dev/null || echo 4)

echo ""
echo "=== 编译完成 ==="
echo "  tts        (简单版): $DIR/build/tts"
echo "  tts_stream (流水线): $DIR/build/tts_stream"

if [ "$(uname -s)" = "Linux" ]; then
    cat > "$DIR/build/run.sh" << RUNEOF
#!/bin/bash
export LD_LIBRARY_PATH="$SDK/lib:\$LD_LIBRARY_PATH"
exec "$DIR/build/tts" "\$@"
RUNEOF
    cat > "$DIR/build/run_stream.sh" << RUNEOF
#!/bin/bash
export LD_LIBRARY_PATH="$SDK/lib:\$LD_LIBRARY_PATH"
exec "$DIR/build/tts_stream" "\$@"
RUNEOF
    chmod +x "$DIR/build/run.sh" "$DIR/build/run_stream.sh"
    echo ""
    echo "Linux 运行:"
    echo "  ./build/run.sh \"文本\""
    echo "  ./build/run_stream.sh  # 然后另一个终端: python3 publish_text.py"
else
    echo ""
    echo "运行:"
    echo "  终端1: ./build/tts_stream"
    echo "  终端2: python3 publish_text.py"
fi
