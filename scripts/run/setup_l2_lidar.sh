#!/bin/bash
# ============================================================
# Unitree L2 雷达串口模式自动配置脚本
#
# 解决问题: L2 上电后可能处于 UDP 模式，而 ROS2 驱动配置为串口模式
#          (initialize_type=1)，需先把雷达切到串口模式才能出点云。
#
# 模式切换的正确认知（对照 example_lidar_serial.cpp 实测得出）:
#   串口链路本身就能下发工作模式指令，无需先接网线走 UDP。
#   完整序列: startLidarRotation -> setLidarWorkMode(8) -> resetLidar
#   UDP 途径仅用于"只接了网线、没接 USB"的场景。
#
# 用法:
#   ./setup_l2_lidar.sh                 # 自动确保处于串口模式（幂等）
#   ./setup_l2_lidar.sh detect          # 只检测当前模式，不做改动
#   ./setup_l2_lidar.sh quiet           # 停转降噪（平时保持安静）
#   ./setup_l2_lidar.sh spin            # 启转（需要点云时）
#   ./setup_l2_lidar.sh to-udp          # 反向切回 UDP 模式
#   ./setup_l2_lidar.sh gen-udev        # 为已识别的雷达生成固定符号链接
#
# 环境变量（仅 UDP 兜底路径需要）:
#   LIDAR_IP   雷达 IP      (默认 192.168.1.62，与 ROS2 launch 一致)
#   LOCAL_IP   本机 IP      (默认 192.168.1.50)
#   LIDAR_NIC  连雷达的网卡 (默认 eth0)
#   PROBE_TIMEOUT 探测超时秒数 (默认 8)
# ============================================================

SDK_DIR=/data/sf_code/unilidar_sdk2/unitree_lidar_sdk
TOOL=$SDK_DIR/bin/set_lidar_mode

LIDAR_IP=${LIDAR_IP:-192.168.1.62}
LOCAL_IP=${LOCAL_IP:-192.168.1.50}
LIDAR_NIC=${LIDAR_NIC:-eth0}
# 4Mbaud 下雷达常需数秒才吐出首个版本包，超时给足余量
PROBE_TIMEOUT=${PROBE_TIMEOUT:-8}
UDEV_RULE=/etc/udev/rules.d/99-unilidar.rules

if [ ! -x "$TOOL" ]; then
    echo "[错误] 未找到工具 $TOOL"
    echo "       请先编译: cd $SDK_DIR/build && cmake .. && make set_lidar_mode"
    exit 1
fi

# ── 枚举候选串口，逐个探测是否为 L2 ──
# 不硬编码 VID/PID，直接用 SDK 握手确认，避免不同批次硬件差异
detect_serial() {
    for dev in /dev/unilidar /dev/ttyACM* /dev/ttyUSB*; do
        [ -e "$dev" ] || continue
        if "$TOOL" --detect-serial --port "$dev" --timeout "$PROBE_TIMEOUT" \
                >/dev/null 2>&1; then
            echo "$dev"
            return 0
        fi
    done
    return 1
}

detect_udp() {
    "$TOOL" --detect-udp --lidar-ip "$LIDAR_IP" --local-ip "$LOCAL_IP" \
            --timeout "$PROBE_TIMEOUT" >/dev/null 2>&1
}

# 经串口链路直接下发切串口模式指令（首选路径，无需网线）
# 注意: 进度信息全部输到 stderr，stdout 只留设备路径供调用方捕获
switch_via_serial() {
    local dev rc
    for dev in /dev/unilidar /dev/ttyACM* /dev/ttyUSB*; do
        [ -e "$dev" ] || continue
        echo "  [尝试] 经 $dev 下发串口模式指令" >&2
        "$TOOL" --to-serial --port "$dev" --timeout "$PROBE_TIMEOUT" \
            > /tmp/l2_switch.out 2>&1
        rc=$?
        grep -v WARNING /tmp/l2_switch.out | sed 's/^/    /' >&2
        if [ "$rc" -eq 0 ]; then
            echo "$dev"
            return 0
        fi
    done
    return 1
}

# 定位雷达串口设备：优先用固定链接，否则逐个探测
# 注: 停转状态下雷达不吐数据，探测会失败，故转子控制不依赖探测
resolve_port() {
    if [ -e /dev/unilidar ]; then
        echo /dev/unilidar
        return 0
    fi
    local dev
    dev=$(detect_serial) && { echo "$dev"; return 0; }
    # 停转时探不到，退而取第一个存在的 CDC 设备
    for dev in /dev/ttyACM* /dev/ttyUSB*; do
        [ -e "$dev" ] && { echo "$dev"; return 0; }
    done
    return 1
}

# ── 确保连雷达的网卡 UP 且 IP 正确（切串口模式的前提）──
ensure_nic() {
    if ! ip link show "$LIDAR_NIC" >/dev/null 2>&1; then
        echo "  [跳过] 网卡 $LIDAR_NIC 不存在"
        return 1
    fi
    local state
    state=$(ip -brief link show "$LIDAR_NIC" | awk '{print $2}')
    if [ "$state" != "UP" ]; then
        echo "  [配置] $LIDAR_NIC 当前 $state，正在拉起..."
        sudo ip link set "$LIDAR_NIC" up
        sleep 2
    fi
    if ! ip -brief addr show "$LIDAR_NIC" | grep -q "$LOCAL_IP"; then
        echo "  [配置] 为 $LIDAR_NIC 添加 IP $LOCAL_IP/24"
        sudo ip addr add "$LOCAL_IP/24" dev "$LIDAR_NIC" 2>/dev/null
        sleep 1
    fi
    echo "  [就绪] $(ip -brief addr show "$LIDAR_NIC")"
    return 0
}

# ── 生成 udev 规则，给雷达一个稳定的 /dev/unilidar 符号链接 ──
# 避免多个 USB 串口设备争抢 ttyACM0 编号导致驱动连错设备
gen_udev() {
    local dev
    dev=$(detect_serial) || { echo "[错误] 未探测到串口模式的 L2，无法生成规则"; exit 1; }
    echo "[发现] L2 位于 $dev"

    local vid pid serial
    vid=$(udevadm info -q property -n "$dev" | grep -oP '(?<=^ID_VENDOR_ID=).*')
    pid=$(udevadm info -q property -n "$dev" | grep -oP '(?<=^ID_MODEL_ID=).*')
    serial=$(udevadm info -q property -n "$dev" | grep -oP '(?<=^ID_SERIAL_SHORT=).*')
    echo "[识别] VID=$vid PID=$pid SERIAL=$serial"

    # 有序列号则用序列号精确匹配，避免误匹配同芯片的其他设备
    local rule
    if [ -n "$serial" ]; then
        rule="SUBSYSTEM==\"tty\", ATTRS{idVendor}==\"$vid\", ATTRS{idProduct}==\"$pid\", ATTRS{serial}==\"$serial\", MODE=\"0666\", GROUP=\"dialout\", SYMLINK+=\"unilidar\""
    else
        rule="SUBSYSTEM==\"tty\", ATTRS{idVendor}==\"$vid\", ATTRS{idProduct}==\"$pid\", MODE=\"0666\", GROUP=\"dialout\", SYMLINK+=\"unilidar\""
    fi

    echo "$rule" | sudo tee "$UDEV_RULE" >/dev/null
    sudo udevadm control --reload-rules && sudo udevadm trigger
    sleep 2
    if [ -e /dev/unilidar ]; then
        echo "[成功] 已创建固定链接 /dev/unilidar -> $(readlink -f /dev/unilidar)"
        echo "       建议把 ROS2 launch 的 serial_port 改为 /dev/unilidar"
    else
        echo "[警告] 规则已写入 $UDEV_RULE，但 /dev/unilidar 未生成，请重新插拔雷达"
    fi
}

# ══════════════════════════════════════════════
case "${1:-auto}" in
    detect)
        echo "=== 检测 L2 当前工作模式 ==="
        if dev=$(detect_serial); then
            echo "[结果] 串口模式，设备 = $dev"
            exit 0
        fi
        echo "  串口无响应，尝试 UDP..."
        ensure_nic
        if detect_udp; then
            echo "[结果] UDP 模式 (雷达 $LIDAR_IP)"
            exit 0
        fi
        echo "[结果] 两种模式均无响应 —— 雷达可能未连接/未上电，或 IP 配置不符"
        exit 3
        ;;

    to-udp)
        echo "=== 切换到 UDP 模式 ==="
        dev=$(detect_serial) || { echo "[错误] 未找到串口模式的雷达，无法切换"; exit 1; }
        echo "[发现] 串口设备 $dev"
        "$TOOL" --to-udp --port "$dev"
        echo "[提示] 雷达已切到 UDP 模式，ROS2 launch 需改为 initialize_type=2"
        exit 0
        ;;

    gen-udev)
        gen_udev
        exit 0
        ;;

    quiet|stop-rotation)
        # 平时保持安静：停转后雷达不再旋转，噪声消失，也不再输出点云
        port=$(resolve_port) || { echo "[错误] 未找到雷达串口设备"; exit 1; }
        echo "=== 停转降噪 ($port) ==="
        "$TOOL" --stop-rotation --port "$port" 2>&1 | grep -v WARNING
        exit "${PIPESTATUS[0]}"
        ;;

    spin|start-rotation)
        port=$(resolve_port) || { echo "[错误] 未找到雷达串口设备"; exit 1; }
        echo "=== 启动转子 ($port) ==="
        "$TOOL" --start-rotation --port "$port" --timeout "$PROBE_TIMEOUT" \
            2>&1 | grep -v WARNING
        exit "${PIPESTATUS[0]}"
        ;;

    auto)
        echo "============================================"
        echo " L2 雷达串口模式自动配置  $(date '+%F %T')"
        echo "============================================"

        # 1) 首选: 先尝试启转。若雷达本就处于串口模式（只是被停转降噪），
        #    启转后立即有数据，无需走带 resetLidar 的重量级模式切换
        echo "[1/3] 尝试启动转子..."
        port=$(resolve_port)
        if [ -n "$port" ] && "$TOOL" --start-rotation --port "$port" \
                --timeout "$PROBE_TIMEOUT" >/dev/null 2>&1; then
            echo ""
            echo "[成功] 雷达已旋转并输出点云，设备 = $port"
            echo ""
            echo "可直接启动建图: /data/sf_code/run_mapping.sh l2"
            exit 0
        fi
        echo "      启转后无数据，需先切换工作模式"

        # 2) 经串口链路切模式。工具内部已做幂等处理：
        #    已在串口模式则直接返回，不会多余地 resetLidar 打断正常工作的雷达
        echo "[2/3] 经 USB 串口切换到串口模式..."
        if dev=$(switch_via_serial); then
            echo ""
            echo "[成功] 串口模式就绪，设备 = $dev"
            if [ "$dev" != "/dev/ttyACM0" ] && [ "$dev" != "/dev/unilidar" ]; then
                echo "[注意] 设备为 $dev，与 launch 默认的 /dev/ttyACM0 不一致"
                echo "       需调整 serial_port 参数，或执行: $0 gen-udev 固定为 /dev/unilidar"
            fi
            echo ""
            echo "可直接启动建图: /data/sf_code/run_mapping.sh l2"
            exit 0
        fi
        echo "      串口路径未成功（USB 可能未接）"

        # 3) 兜底: 只接了网线时，经 UDP 下发切换指令
        echo "[3/3] 尝试 UDP 路径 (网卡 $LIDAR_NIC)..."
        if ! ensure_nic; then
            echo "[失败] USB 与网口均不可用"
            echo "       请确认雷达已上电，并接好 USB 线或网线"
            exit 2
        fi

        if ! detect_udp; then
            echo "[失败] UDP 也探测不到雷达 ($LIDAR_IP)"
            echo "       请检查: 雷达是否上电 / 网线是否插在 $LIDAR_NIC / 雷达 IP 是否为 $LIDAR_IP"
            echo "       (可用环境变量覆盖: LIDAR_IP=x.x.x.x LOCAL_IP=y.y.y.y LIDAR_NIC=ethN $0)"
            exit 3
        fi
        echo "      [就绪] UDP 已连通雷达，下发切换指令"
        "$TOOL" --to-serial --via udp --lidar-ip "$LIDAR_IP" --local-ip "$LOCAL_IP" \
            2>&1 | grep -v WARNING | sed 's/^/      /'

        echo "      等待串口设备枚举 (最多 20 秒)..."
        for i in $(seq 1 10); do
            sleep 2
            if dev=$(detect_serial); then
                echo ""
                echo "[成功] 已切换到串口模式，设备 = $dev"
                echo ""
                echo "下一步:"
                echo "  1. 建议固定设备名(避免编号漂移): $0 gen-udev"
                echo "  2. 启动建图: /data/sf_code/run_mapping.sh l2"
                exit 0
            fi
            echo "      ...第 $i 次探测未成功"
        done

        echo ""
        echo "[失败] 指令已下发但串口设备未出现"
        echo "       L2 的串口链路走 USB，请确认 USB 线已连接到设备"
        echo "       也可尝试重新插拔 USB 后执行: $0 detect"
        exit 3
        ;;

    *)
        sed -n '15,26p' "$0" | sed 's/^# \?//'
        exit 1
        ;;
esac
