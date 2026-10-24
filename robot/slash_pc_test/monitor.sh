#!/bin/bash
# =====================================================================
#  轮式腿机器人上位机 (Web) —— 后台服务控制脚本
#
#  用法:
#    ./monitor.sh start              启动服务并自动打开浏览器
#    ./monitor.sh start --no-browser 启动但不打开浏览器 (纯后台采集)
#    ./monitor.sh stop               停止服务 (释放串口)
#    ./monitor.sh restart            重启
#    ./monitor.sh status             查看运行状态 / 端口 / 串口连接
#    ./monitor.sh log                实时查看服务日志 (Ctrl+C 退出)
#    ./monitor.sh open               在浏览器打开页面
#
#  说明: 服务运行时会独占串口, 若要让其他程序(如 Arduino 串口监视器、
#        ROS 节点) 访问机器人, 请先 ./monitor.sh stop
# =====================================================================

set -u
DIR="$(cd "$(dirname "$0")" && pwd)"
cd "$DIR"

PORT="${PORT:-8765}"
PID_FILE="$DIR/.monitor.pid"
LOG_FILE="$DIR/logs/service.log"
URL="http://127.0.0.1:$PORT/"

is_running() {
    [ -f "$PID_FILE" ] || return 1
    local pid
    pid="$(cat "$PID_FILE" 2>/dev/null)"
    [ -n "${pid:-}" ] || return 1
    kill -0 "$pid" 2>/dev/null
}

pid_of() { cat "$PID_FILE" 2>/dev/null; }

case "${1:-status}" in

  start)
    if is_running; then
        echo "服务已在运行 (PID $(pid_of))  $URL"
        exit 0
    fi
    mkdir -p "$(dirname "$LOG_FILE")"
    EXTRA=""
    if [ "${2:-}" = "--no-browser" ] || [ "${2:-}" = "-n" ]; then
        EXTRA="--no-browser"
    fi
    nohup python3 -u web_monitor.py --port "$PORT" $EXTRA >> "$LOG_FILE" 2>&1 &
    echo $! > "$PID_FILE"

    # 等待端口就绪
    for _ in $(seq 1 40); do
        if curl -s -o /dev/null -m 1 "$URL" 2>/dev/null; then break; fi
        sleep 0.25
    done

    if is_running && curl -s -o /dev/null -m 2 "$URL" 2>/dev/null; then
        echo "已启动 (PID $(pid_of))  $URL"
        echo "日志: $LOG_FILE"
        if [ -z "$EXTRA" ]; then
            sleep 0.4
            open "$URL"
            echo "已在浏览器打开"
        fi
    else
        echo "启动失败, 请查看日志: tail -f $LOG_FILE"
        exit 1
    fi
    ;;

  stop)
    if ! is_running; then
        echo "服务未在运行"
        rm -f "$PID_FILE"
        exit 0
    fi
    pid="$(pid_of)"
    echo "正在停止 (PID $pid)…"
    kill -TERM "$pid" 2>/dev/null
    for _ in $(seq 1 25); do
        kill -0 "$pid" 2>/dev/null || break
        sleep 0.2
    done
    if kill -0 "$pid" 2>/dev/null; then
        echo "优雅退出超时, 强制结束"
        kill -9 "$pid" 2>/dev/null
        sleep 0.3
    fi
    rm -f "$PID_FILE"
    echo "已停止 (串口已释放)"
    ;;

  restart)
    "$0" stop
    sleep 0.5
    "$0" start "${2:-}"
    ;;

  status)
    if is_running; then
        pid="$(pid_of)"
        echo "状态: 运行中 (PID $pid)"
        echo "地址: $URL"
        echo "日志: $LOG_FILE"
        lsof -nP -iTCP:"$PORT" -sTCP:LISTEN 2>/dev/null | tail -n +2 | awk '{print "监听: "$1" "$9}'
        if command -v curl >/dev/null 2>&1; then
            info="$(curl -s -m 2 "$URL/api/state?since=999999999" 2>/dev/null)"
            if [ -n "$info" ]; then
                echo "$info" | python3 -c "
import json,sys
try:
    s=json.load(sys.stdin)
except Exception:
    raise SystemExit
st=s.get('stats',{})
print('串口: ' + (('已连接 '+s.get('port','')+' @'+str(s.get('baudrate',''))) if s.get('connected') else '未连接'))
print('帧率: %.1f Hz | 帧数: %d | CRC错: %d'%(st.get('hz',0),st.get('frames',0),st.get('crc_err',0)))
print('录制: ' + ('进行中 [%s]'%s['record']['name'] if s.get('recording') else '未录制'))
"
            fi
        fi
    else
        echo "状态: 未运行"
        rm -f "$PID_FILE"
    fi
    ;;

  log)
    mkdir -p "$(dirname "$LOG_FILE")"
    echo "实时日志 (Ctrl+C 退出): $LOG_FILE"
    tail -f "$LOG_FILE"
    ;;

  open)
    open "$URL"
    echo "已打开 $URL"
    ;;

  *)
    echo "用法: $0 {start|stop|restart|status|log|open}"
    echo "  start [--no-browser]   启动服务"
    echo "  stop                   停止服务并释放串口"
    echo "  restart                重启"
    echo "  status                 查看状态"
    echo "  log                    实时日志"
    echo "  open                   打开页面"
    exit 1
    ;;
esac
