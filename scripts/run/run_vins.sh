#!/bin/bash
# run_vins.sh — RealSense D435i + VINS-Fusion 视觉惯性里程计（双目红外+IMU）
#
# 链路: realsense2_camera(infra1+infra2+IMU) → vins_node → /vins_estimator/odometry
#       （后续可把该话题喂给 rtabmap 做建图，替代易跟丢的 rgbd_odometry）
#
# 为什么用双目红外而不是单目彩色（单目实测的三个失败点）:
#  1. 尺度: 单目靠平移激励恢复尺度，日志刷 26 次 "IMU excitation not enouth"、
#     182 次 "Not enough features or parallax"，耗时 27s 才初始化。
#     双目由 50mm 硬件基线直接给出尺度，初始化几秒即可。
#  2. 快门: color 是卷帘快门，快速运动时行错位导致 "unstable features tracking"；
#     红外(infra1/2)是全局快门，无此问题。
#  3. 外参: 单目用 estimate_extrinsic:1 把外参优化跑偏到 3.78°/55mm，反污染位姿。
#     双目版改用 estimate_extrinsic:0 锁死出厂外参。
#
# 关键点:
#  * VINS 吃原始 IMU（/camera/camera/imu），不是 madgwick 滤波后的 /imu/data
#  * VINS 是纯特征法 VIO，不使用深度图 → 本脚本关闭 depth/color 流省带宽
#  * 必须关闭红外投影器(emitter): 散斑图案随相机移动，会污染光流跟踪
#  * 初始化成功标志: 日志出现 "solver costs: xx [ms]"
#
# 用法:
#   run_vins.sh            启动(双目红外+IMU，带 rviz2，显示在 VNC 桌面)
#   run_vins.sh --mono     退回单目彩色+IMU 配置（对比用）
#   run_vins.sh --no-rviz  纯后台
#   run_vins.sh status     查看状态
#   run_vins.sh stop       停止
set +u

LOG_DIR=/tmp/vins_logs
VINS_WS=/data/sf_code/rtabmap
VINS_SRC=$VINS_WS/src/VINS-Fusion-ROS2-main
CFG_DIR=$VINS_SRC/config/realsense_d435i
RVIZ_CFG=${RVIZ_CFG:-/data/sf_code/vins_output/vins_nav.rviz}

# 模式: stereo(默认) | mono，决定配置文件与相机流
VINS_MODE=stereo
for a in "$@"; do [ "$a" = "--mono" ] && VINS_MODE=mono; done

CAM_NS=/camera/camera
IMU_RAW_TOPIC=$CAM_NS/imu
# 注意: vins_node 的话题用相对名发布，ROS2 下相对名不含节点名，
# 所以实际话题是 /odometry 而不是 /vins_estimator/odometry
ODOM_TOPIC=/odometry
PATH_TOPIC=/path
if [ "$VINS_MODE" = "mono" ]; then
    VINS_CONFIG=${VINS_CONFIG:-$CFG_DIR/d435i_mono_imu_config.yaml}
    IMG_TOPIC=$CAM_NS/color/image_raw
else
    VINS_CONFIG=${VINS_CONFIG:-$CFG_DIR/d435i_stereo_imu_config.yaml}
    IMG_TOPIC=$CAM_NS/infra1/image_rect_raw
fi

need_source() {
    source /opt/ros/humble/setup.bash
    if [ -f "$VINS_WS/install/setup.bash" ]; then
        source "$VINS_WS/install/setup.bash"
    else
        echo "[错误] $VINS_WS/install/setup.bash 不存在，请先编译工作空间!"
        exit 1
    fi
}

# 等待日志文件出现指定关键字（事件驱动，代替固定 sleep）
# 用法: wait_log <logfile> <pattern> <timeout秒>
wait_log() {
    local f=$1 pat=$2 tmo=$3 i=0
    while [ "$i" -lt "$tmo" ]; do
        grep -q "$pat" "$f" 2>/dev/null && return 0
        sleep 0.5; i=$((i+1))
    done
    return 1
}

# 等待 ROS2 节点注册完成
# 用法: wait_node <node名> <timeout秒>
wait_node() {
    local node=$1 tmo=$2 i=0
    while [ "$i" -lt "$tmo" ]; do
        ros2 node list 2>/dev/null | grep -qx "$node" && return 0
        sleep 0.5; i=$((i+1))
    done
    return 1
}

# 等待进程退出
# 用法: wait_gone <pgrep模式> <timeout秒>
wait_gone() {
    local pat=$1 tmo=$2 i=0
    while [ "$i" -lt "$((tmo*2))" ]; do
        pgrep -f "$pat" >/dev/null 2>&1 || return 0
        sleep 0.5; i=$((i+1))
    done
    return 1
}

case "$1" in
status)
    need_source
    # 模式以实际运行中的配置为准，避免 status 不带 --mono 时看错话题
    if grep -q "mono_imu_config" "$LOG_DIR/vins.log" 2>/dev/null; then
        VINS_MODE=mono; IMG_TOPIC=$CAM_NS/color/image_raw
    fi
    echo "=== 组件状态 (模式: $VINS_MODE) ==="
    for p in realsense2_camera_node vins_node "rviz2.*vins_nav"; do
        pid=$(pgrep -f "[${p:0:1}]${p:1}" | head -1)
        [ -n "$pid" ] && echo "  [运行中] $p (pid=$pid)" || echo "  [未运行] $p"
    done
    echo ""
    echo "=== 话题频率 ==="
    for t in "$IMG_TOPIC" "$IMU_RAW_TOPIC" "$ODOM_TOPIC"; do
        printf "  %-50s " "$t"
        rate=$(timeout 6 ros2 topic hz "$t" 2>/dev/null | grep -m1 "average rate" | awk '{print $3}')
        if [ -z "$rate" ]; then
            rate=$(timeout 10 ros2 topic hz "$t" 2>/dev/null | grep -m1 "average rate" | awk '{print $3}')
        fi
        [ -n "$rate" ] && echo "$rate Hz" || echo "无数据"
    done
    echo ""
    echo "=== 初始化状态 ==="
    if grep -q "solver costs" "$LOG_DIR/vins.log" 2>/dev/null; then
        echo "  已初始化（出现后端优化输出）"
        grep "solver costs" "$LOG_DIR/vins.log" | tail -1 | sed 's/^/  /'
    else
        echo "  未初始化: 请对纹理丰富场景做缓慢 8 字晃动"
        grep -E "excitation|features" "$LOG_DIR/vins.log" 2>/dev/null | tail -1 | sed 's/^/  /'
    fi
    echo ""
    echo "轨迹文件: /data/sf_code/vins_output/vio.csv"
    exit 0
    ;;
stop)
    echo "停止 VINS..."
    # 只杀 VINS 专用的 rviz2（按配置文件名精确匹配，避免误杀其他 rviz2）
    pkill -f "rviz2.*vins_nav.rviz" 2>/dev/null
    pkill -f vins_node 2>/dev/null
    pkill -f realsense2_camera_node 2>/dev/null
    # 等优雅退出，超时未退则强杀（GUI 的 rviz2 对 SIGTERM 常不响应）
    for pat in "rviz2.*vins_nav.rviz" vins_node realsense2_camera_node; do
        if ! wait_gone "[${pat:0:1}]${pat:1}" 10; then
            echo "  强制结束: $pat"
            pkill -9 -f "[${pat:0:1}]${pat:1}" 2>/dev/null
        fi
    done
    echo "已停止（轨迹保留在 /data/sf_code/vins_output/vio.csv）"
    exit 0
    ;;
esac

USE_RVIZ=1
for a in "$@"; do [ "$a" = "--no-rviz" ] && USE_RVIZ=0; done

mkdir -p "$LOG_DIR" /data/sf_code/vins_output
rm -f "$LOG_DIR"/*.log   # 清空旧日志，避免残留关键字干扰就绪检测
need_source

echo "============================================"
echo " D435i + VINS-Fusion 视觉惯性里程计"
echo " 模式:     $VINS_MODE  (图像: $IMG_TOPIC)"
echo " 配置文件: $VINS_CONFIG"
echo " 日志目录: $LOG_DIR"
echo "============================================"

if [ ! -f "$VINS_CONFIG" ]; then
    echo "[错误] 配置文件不存在: $VINS_CONFIG"
    exit 1
fi

# ── 1. 相机 ──
# enable_sync=true : 驱动层帧同步，多路图像时间戳对齐，VIO 必需
# 640x360x30       : 与内参文件分辨率一致
# 双目模式只开 infra1+infra2+IMU：VINS 不用深度图和彩色图，关掉省 USB 带宽和 CPU
pkill -9 -f realsense2_camera_node 2>/dev/null
wait_gone "[r]ealsense2_camera_node" 5
if [ "$VINS_MODE" = "mono" ]; then
    echo "[1/3] 启动 D435i (单目彩色 640x360@30 + IMU gyro200/accel100)..."
    nohup ros2 launch realsense2_camera rs_launch.py \
        enable_depth:=true enable_color:=true align_depth.enable:=true \
        enable_sync:=true \
        enable_gyro:=true enable_accel:=true unite_imu_method:=2 \
        rgb_camera.color_profile:=640x360x30 \
        depth_module.depth_profile:=640x360x30 \
        > "$LOG_DIR/camera.log" 2>&1 &
else
    echo "[1/3] 启动 D435i (双目红外 640x360@30 + IMU gyro200/accel100)..."
    nohup ros2 launch realsense2_camera rs_launch.py \
        enable_depth:=false enable_color:=false align_depth.enable:=false \
        enable_infra1:=true enable_infra2:=true \
        depth_module.infra_profile:=640x360x30 \
        enable_sync:=true \
        enable_gyro:=true enable_accel:=true unite_imu_method:=2 \
        > "$LOG_DIR/camera.log" 2>&1 &
fi

if ! wait_log "$LOG_DIR/camera.log" "Node Is Up" 60; then
    echo "      [失败] 相机未就绪，查看 $LOG_DIR/camera.log"
    exit 1
fi
echo "      相机就绪"

# ── 1b. 关闭红外投影器 ──
# 背景: emitter 打出的散斑图案是投影上去的，相机一动图案就在物体表面滑动，
#       KLT 光流会跟着这些"假特征"跑 → 位姿被污染。VINS 不用深度，直接关掉。
# emitter_enabled 是驱动动态注册的 integer 参数(0=关/1=开/2=自动)，
# launch 参数列表里没有声明，只能运行时 set（传 bool 会被驱动报类型错误）
if [ "$VINS_MODE" != "mono" ]; then
    if timeout 15 ros2 param set /camera/camera depth_module.emitter_enabled 0 2>/dev/null | grep -q "successful"; then
        echo "      红外投影器已关闭（避免散斑污染光流）"
    else
        echo "      [警告] 关闭投影器失败，散斑可能影响跟踪精度"
        echo "             手动执行: ros2 param set /camera/camera depth_module.emitter_enabled 0"
    fi
fi

# ── 2. IMU 检查（VINS 必需，无回退）──
if ! timeout 8 ros2 topic list 2>/dev/null | grep -qx "$IMU_RAW_TOPIC"; then
    echo "      [失败] 无 IMU 话题 $IMU_RAW_TOPIC，VINS 无法启动"
    echo "             排查: librealsense 需开 FORCE_RSUSB_BACKEND"
    exit 1
fi
echo "      检测到原始 IMU: $IMU_RAW_TOPIC"

# ── 3. VINS 节点 ──
if [ "$VINS_MODE" = "mono" ]; then
    echo "[2/3] 启动 vins_node (单目彩色 + IMU)..."
else
    echo "[2/3] 启动 vins_node (双目红外 + IMU)..."
fi
nohup ros2 run vins vins_node "$VINS_CONFIG" \
    > "$LOG_DIR/vins.log" 2>&1 &
if ! wait_node /vins_estimator 30; then
    echo "      [失败] vins_estimator 未注册，查看 $LOG_DIR/vins.log"
    exit 1
fi
echo "      VINS 已启动"

# ── 等待初始化完成 ──
# 背景: 双目尺度由基线给出，不需要强平移激励，但仍需轻微运动让 IMU 偏置可观测。
# 策略: 日志出现 "solver costs"（后端优化启动）即认为初始化成功。
#       超时不阻断（rviz 里可继续观察）
echo "[3/3] 等待 VINS 初始化..."
if [ "$VINS_MODE" = "mono" ]; then
    echo "      ★ 单目需要充分平移激励：对纹理丰富场景做缓慢 8 字晃动（幅度30~50cm）★"
    INIT_TMO=180
else
    echo "      ★ 双目只需轻微平稳平移（十几厘米即可），对准纹理丰富的区域★"
    INIT_TMO=90
fi
if wait_log "$LOG_DIR/vins.log" "solver costs" "$INIT_TMO"; then
    echo "      ★ VINS 初始化成功，里程计开始输出（$ODOM_TOPIC）"
else
    echo "      [警告] $INIT_TMO 秒内未检测到初始化成功。"
    echo "             请继续晃动相机；确认状态: $0 status"
fi

# ── 可视化（rviz2）──
# 显示内容(见 vins_nav.rviz)，注意这些话题都在根命名空间:
#   /path        轨迹 (nav_msgs/Path)
#   /odometry    当前位姿 (nav_msgs/Odometry)
#   /point_cloud 特征点云 (sensor_msgs/PointCloud，老类型非 PointCloud2)
#   /image_track 特征跟踪图像（双目下为左右拼接图）
# VINS 不发布自己的 TF 树，Fixed Frame 直接用 world（path/odometry 的 frame_id）
if [ "$USE_RVIZ" = "1" ]; then
    # 走 VNC: 让窗口显示在设备物理桌面上，Mac 通过 VNC 观看
    export DISPLAY=:0
    export XAUTHORITY=/home/sunrise/.Xauthority
    echo "[+] 启动 rviz2 (显示在设备桌面，请用 VNC 查看)..."
    nohup rviz2 -d "$RVIZ_CFG" > "$LOG_DIR/rviz.log" 2>&1 &
    if wait_node /rviz2 30; then
        echo "      rviz2 已就绪"
    else
        echo "      [警告] rviz2 未就绪，查看 $LOG_DIR/rviz.log（不影响里程计）"
    fi
fi

echo ""
echo "VINS 已启动! (模式: $VINS_MODE)"
echo "  里程计话题: $ODOM_TOPIC"
echo "  查看状态:   $0 status"
echo "  停止:       $0 stop"
echo "  提示: 初始化后先静止 2 秒观察，静止时位姿应几乎不动；若静止仍漂移需查 IMU 噪声/外参"
