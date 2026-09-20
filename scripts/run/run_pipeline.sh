#!/bin/bash
# ============================================================
# 语音交互全链路一键启动脚本
#
# 链路: KWS唤醒(你好无恙)+录音 -> transformVL桥接 -> omni多模态推理
#       -> tts_bridge -> TTS播报 (matcha/hobot 双后端可选)
#
# 用法:
#   ./run_pipeline.sh                     # 启动全链路 (默认 matcha TTS)
#   TTS_BACKEND=hobot ./run_pipeline.sh   # 使用 hobot_tts 后端
#   ./run_pipeline.sh stop                # 停止全部组件
#   ./run_pipeline.sh status              # 查看运行状态
#
# 日志: /tmp/pipeline_logs/*.log
# ============================================================
# 注意: 不能用 set -u，ROS setup.bash 内部有未定义变量引用

KWS_DIR=/data/sf_code/kws/sherpa-onnx-kws-cpp
AGENT_WS=/data/sf_code/agent_ws
ROS_WS=/data/sf_code/ros_ws
TTS_DIR=/data/sf_code/tts/tts_cpp
LOG_DIR=/tmp/pipeline_logs
PIDFILE=$LOG_DIR/pipeline.pids
# TTS 后端: matcha(tts_stream) 或 hobot(hobot_tts)，可用环境变量覆盖
TTS_BACKEND=${TTS_BACKEND:-hobot}

COMPONENTS=(kws camera omni transformVL tts_bridge tts_stream)

stop_all() {
    echo "=== 停止全链路 ==="
    if [ -f "$PIDFILE" ]; then
        while read -r name pid; do
            if kill -0 "$pid" 2>/dev/null; then
                kill "$pid" 2>/dev/null
                echo "  [停止] $name (pid=$pid)"
            fi
        done < "$PIDFILE"
        rm -f "$PIDFILE"
    fi
    # 兜底清理
    pkill -f kws_vad_demo 2>/dev/null
    pkill -f "omni_node/omni_node" 2>/dev/null
    pkill -f transformVL.py 2>/dev/null
    pkill -f camera_pub.py 2>/dev/null
    pkill -f tts_bridge.py 2>/dev/null
    pkill -f tool_agent.py 2>/dev/null
    pkill -f tts_stream 2>/dev/null
    pkill -f "hobot_tts" 2>/dev/null
    echo "=== 已全部停止 ==="
}

show_status() {
    echo "=== 组件状态 ==="
    for pat in kws_vad_demo camera_pub.py "omni_node/omni_node" transformVL.py tts_bridge.py tool_agent.py tts_stream hobot_tts; do
        pid=$(pgrep -f "$pat" | head -1)
        name=$(basename "$pat")
        if [ -n "$pid" ]; then
            echo "  [运行中] $name (pid=$pid)"
        else
            echo "  [未运行] $name"
        fi
    done
}

case "${1:-start}" in
    stop)   stop_all;   exit 0 ;;
    status) show_status; exit 0 ;;
    start)  ;;
    *) echo "用法: $0 [start|stop|status]"; exit 1 ;;
esac

# ── 启动前清理旧进程 ──
stop_all >/dev/null 2>&1
mkdir -p "$LOG_DIR"
: > "$PIDFILE"

source /opt/ros/humble/setup.bash
source "$AGENT_WS/install/setup.bash"
# tool_agent 需要 ros_ws 里的 zone_interfaces（导航服务接口）
[ -f "$ROS_WS/install/setup.bash" ] && source "$ROS_WS/install/setup.bash"

echo "============================================"
echo " 启动语音交互全链路  $(date '+%F %T')"
echo " 日志目录: $LOG_DIR"
echo "============================================"

# ── 1. omni 推理节点（模型加载最慢，最先启动）──
ros2 launch omni_node omni.launch.py > "$LOG_DIR/omni.log" 2>&1 &
echo "omni $!" >> "$PIDFILE"
echo "[1/6] omni_node 启动中 (加载 Qwen2.5-Omni-3B，需约1-2分钟)..."

# ── 2. TTS 后端（matcha=tts_stream / hobot=hobot_tts）──
if [ "$TTS_BACKEND" = "hobot" ]; then
    source /opt/tros/humble/setup.bash
    # 优先用改造版 hobot_tts（支持 /tts_stop 真正取消队列，唤醒打断必需）；
    # 它必须在 tros 之后 source，才能覆盖 /opt/tros 里的预编译版本
    HOBOT_WS=/data/sf_code/tts/hobot_ws/install/setup.bash
    if [ -f "$HOBOT_WS" ]; then
        source "$HOBOT_WS"
        echo "      (使用改造版 hobot_tts，支持唤醒打断)"
    else
        echo "      [警告] 未找到改造版 hobot_tts，唤醒打断将无法清空 TTS 队列"
    fi
    export GLOG_minloglevel=1
    # hobot_tts 直接订阅 /tts_text 话题，用 default 设备走 PulseAudio
    ros2 run hobot_tts hobot_tts --ros-args -p playback_device:="default" \
        > "$LOG_DIR/hobot_tts.log" 2>&1 &
    echo "hobot_tts $!" >> "$PIDFILE"
    echo "[2/6] hobot_tts 已启动 (WeTTS vits, 16kHz)"
else
    # 注意: 必须用 tts_py 下的原版动态形状模型；
    # tts_cpp 自带的 model-steps-3.onnx 已被 horizon_convert 改成固定形状(max_len=50)，
    # 合成任意长度文本会抛 ONNX 广播异常导致进程崩溃
    cd "$TTS_DIR"
    LD_LIBRARY_PATH="$TTS_DIR/sherpa-onnx-sdk/lib:${LD_LIBRARY_PATH:-}" \
        ./build/tts_stream -m /data/sf_code/tts/tts_py/matcha-icefall-zh-baker \
        > "$LOG_DIR/tts_stream.log" 2>&1 &
    echo "tts_stream $!" >> "$PIDFILE"
    echo "[2/6] tts_stream 已启动 (Matcha, 22kHz)"
fi

# ── 3. TTS 桥接（ROS话题 -> 对应后端）──
python3 "$AGENT_WS/install/omni_node/lib/omni_node/tts_bridge.py" \
    --ros-args -p backend:="$TTS_BACKEND" \
    > "$LOG_DIR/tts_bridge.log" 2>&1 &
echo "tts_bridge $!" >> "$PIDFILE"
echo "[3/6] tts_bridge 已启动 (backend=$TTS_BACKEND)"

# ── 4. 相机发布节点 ──
python3 "$AGENT_WS/install/omni_node/lib/omni_node/camera_pub.py" \
    > "$LOG_DIR/camera.log" 2>&1 &
echo "camera $!" >> "$PIDFILE"
echo "[4/6] camera_pub 已启动"

# ── 5. KWS 桥接（监听状态文件下降沿，注意传入正确的 commands 路径）──
python3 "$AGENT_WS/install/omni_node/lib/omni_node/transformVL.py" \
    --ros-args -p commands_dir:="$KWS_DIR/commands" \
    > "$LOG_DIR/transformVL.log" 2>&1 &
echo "transformVL $!" >> "$PIDFILE"
echo "[5/6] transformVL 已启动 (commands_dir=$KWS_DIR/commands)"

# ── 5.5 Tool-Call 执行器（解析模型输出的 JSON → 导航/点头/摇头）──
python3 "$AGENT_WS/install/omni_node/lib/omni_node/tool_agent.py" \
    > "$LOG_DIR/tool_agent.log" 2>&1 &
echo "tool_agent $!" >> "$PIDFILE"
echo "[5.5] tool_agent 已启动 (工具调用: navigate/nod/shake)"

# ── 6. KWS 唤醒+录音（最后启动，避免模型未就绪时误触发）──
cd "$KWS_DIR"
./build/kws_vad_demo --keywords-file keywords_xiaofei.txt --model-dir model \
    > "$LOG_DIR/kws.log" 2>&1 &
echo "kws $!" >> "$PIDFILE"
echo "[6/6] kws_vad_demo 已启动"

echo ""
echo "============================================"
echo " 全部组件已启动!"
echo ""
echo " 使用方法:"
echo "   1. 等待 omni 模型加载完成:"
echo "      tail -f $LOG_DIR/omni.log   # 看到 'XLM init success' 即就绪"
echo "   2. 对麦克风说唤醒词: 你好无恙"
echo "   3. 听到后继续说命令(如: 你看到了什么?)"
echo "   4. 等待推理完成，扬声器自动播报回答"
echo ""
echo " 当前 TTS 后端: $TTS_BACKEND"
echo ""
echo " 监控命令:"
echo "   $0 status                          # 组件状态"
echo "   tail -f $LOG_DIR/kws.log           # 唤醒日志"
echo "   tail -f $LOG_DIR/omni.log          # 推理日志"
echo "   tail -f $LOG_DIR/tts_bridge.log    # 播报文本"
echo "   python3 $AGENT_WS/install/omni_node/lib/omni_node/watch_omni.py   # 推理结果实时查看"
echo "   $0 stop                            # 停止全部"
echo "============================================"
