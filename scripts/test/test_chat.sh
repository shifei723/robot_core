#!/bin/bash
# test_chat.sh — 用预录音频注入一次"纯聊天"提问，观察模型原始输出
# 目的: 判断新的工具调用 prompt 是否导致聊天也被输出成 JSON
set +u
source /opt/ros/humble/setup.bash
source /data/sf_code/agent_ws/install/setup.bash
source /data/sf_code/ros_ws/install/setup.bash 2>/dev/null
export PYTHONUNBUFFERED=1

WAV=${1:-/data/sf_code/agent_ws/src/omni_node/scripts/hangzhou.wav}
CMD_DIR=/data/sf_code/kws/sherpa-onnx-kws-cpp/commands

echo "=== 启动 watch_omni 观察原始输出 ==="
python3 /data/sf_code/agent_ws/install/omni_node/lib/omni_node/watch_omni.py \
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
