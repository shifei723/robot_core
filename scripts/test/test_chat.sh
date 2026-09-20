#!/bin/bash
# --- robot_core 可移植自定位 ---
ROBOT_CORE="$(cd "$(dirname "$(readlink -f "${BASH_SOURCE[0]:-$0}")")/../.." && pwd)"; export ROBOT_CORE
# test_chat.sh — 用预录音频注入一次"纯聊天"提问，观察模型原始输出
# 目的: 判断新的工具调用 prompt 是否导致聊天也被输出成 JSON
set +u
source "$ROBOT_CORE/setup.sh"
true
true 2>/dev/null
export PYTHONUNBUFFERED=1

WAV=${1:-$ROBOT_CORE/voice/agent/src/omni_node/scripts/hangzhou.wav}
CMD_DIR=$ROBOT_CORE/voice/kws/sherpa-onnx-kws-cpp/commands

echo "=== 启动 watch_omni 观察原始输出 ==="
python3 $ROBOT_CORE/install/agent_ws/omni_node/lib/omni_node/watch_omni.py \
    > /tmp/chat_watch.log 2>&1 &
PID_W=$!
sleep 6

echo "=== 注入聊天音频: $(basename "$WAV") ==="
cp "$WAV" "$CMD_DIR/latest_command.wav"
echo 1 > "$CMD_DIR/kws_status"
sleep 0.5
echo 0 > "$CMD_DIR/kws_status"

echo "=== 等待推理 (45秒) ==="
sleep 45

kill -INT $PID_W 2>/dev/null
sleep 2

echo ""
echo "=== 模型原始输出 (/omni/output_text) ==="
cat /tmp/chat_watch.log

echo ""
echo "=== tts_bridge 是否放行 ==="
tail -6 /tmp/pipeline_logs/tts_bridge.log

kill $PID_W 2>/dev/null
