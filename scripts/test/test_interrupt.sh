#!/bin/bash
# --- robot_core 可移植自定位 ---
ROBOT_CORE="$(cd "$(dirname "$(readlink -f "${BASH_SOURCE[0]:-$0}")")/../.." && pwd)"; export ROBOT_CORE
# test_interrupt.sh — 唤醒打断联调测试
# 用法: bash test_interrupt.sh
set +u
SCRIPTS=$ROBOT_CORE/voice/agent/src/omni_node/scripts

source "$ROBOT_CORE/setup.sh"
true
export PYTHONUNBUFFERED=1

echo "=== 清理旧实例 ==="
pkill -f tts_bridge.py 2>/dev/null
pkill -f tool_agent.py 2>/dev/null
pkill -f fake_nav_server.py 2>/dev/null
pkill -f mock_wl_base.py 2>/dev/null
sleep 2

cd "$SCRIPTS" || exit 1
FAKE_NAV_SEC=5 python3 fake_nav_server.py > /tmp/fakenav.log 2>&1 &
PID_NAV=$!
python3 mock_wl_base.py > /tmp/mockbase.log 2>&1 &
PID_BASE=$!
python3 tool_agent.py > /tmp/toolagent.log 2>&1 &
PID_AGENT=$!
python3 tts_bridge.py --ros-args -p backend:=hobot -p unmute_timeout:=5.0 \
    > /tmp/ttsbridge.log 2>&1 &
PID_TTS=$!
sleep 10

echo "=== 执行打断测试 ==="
timeout 60 python3 probe_interrupt.py 2>&1

echo ""
echo "=== tool_agent 日志 ==="
grep -E "解析出|执行:|播报|打断|完成" /tmp/toolagent.log | tail -14

echo ""
echo "=== tts_bridge 打断与恢复 ==="
grep -E "打断|已打断丢弃|TTS#" /tmp/ttsbridge.log | tail -10

echo ""
echo "=== 验证静音已解除 (应为 Mute: no) ==="
pactl list sink-inputs 2>/dev/null | grep -E "Sink Input #|Mute:|process.binary" | head -6

kill $PID_NAV $PID_AGENT $PID_TTS $PID_BASE 2>/dev/null
echo ""
echo "=== 测试结束 ==="
