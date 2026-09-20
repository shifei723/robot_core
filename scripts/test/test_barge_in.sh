#!/bin/bash
# test_barge_in.sh — 端到端唤醒打断验证（在完整链路上跑）
# 模拟: 让机器人开始长篇回答 → 中途发唤醒打断 → 验证 TTS 队列被真正清空
set +u
source /opt/ros/humble/setup.bash
source /data/sf_code/agent_ws/install/setup.bash
export PYTHONUNBUFFERED=1

echo "=== 前置检查 ==="
grep -E "Interrupt topic" /tmp/pipeline_logs/hobot_tts.log 2>/dev/null \
  && echo "  hobot_tts 打断接口: 就绪" \
  || { echo "  [失败] hobot_tts 无打断接口，请确认用的是改造版"; exit 1; }

echo ""
echo "=== 注入杭州提问（会产生 8 句长回答）==="
CMD_DIR=/data/sf_code/kws/sherpa-onnx-kws-cpp/commands
cp /data/sf_code/agent_ws/src/omni_node/scripts/hangzhou.wav "$CMD_DIR/latest_command.wav"
echo 1 > "$CMD_DIR/kws_status"; sleep 0.5; echo 0 > "$CMD_DIR/kws_status"

echo "=== 等 12 秒让它开始播报 ==="
sleep 12
echo "--- 已播报的内容 ---"
grep -E "TTS#" /tmp/pipeline_logs/tts_bridge.log | tail -4

echo ""
echo "=== 模拟唤醒打断（kws_status 上升沿）==="
echo 1 > "$CMD_DIR/kws_status"
sleep 3

echo "--- transformVL 是否广播打断 ---"
grep -E "广播打断" /tmp/pipeline_logs/transformVL.log | tail -2
echo "--- tts_bridge 响应 ---"
grep -E "\[打断\]" /tmp/pipeline_logs/tts_bridge.log | tail -3
echo "--- hobot_tts 真正取消（关键）---"
grep -E "\[Interrupt\]" /tmp/pipeline_logs/hobot_tts.log | tail -3

# 复位状态，避免影响后续
echo 0 > "$CMD_DIR/kws_status"
echo ""
echo "=== 测试结束 ==="
