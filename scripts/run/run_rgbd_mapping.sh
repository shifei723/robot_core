#!/bin/bash
# --- robot_core 可移植自定位 ---
ROBOT_CORE="$(cd "$(dirname "$(readlink -f "${BASH_SOURCE[0]:-$0}")")/../.." && pwd)"; export ROBOT_CORE
source "$ROBOT_CORE/setup.sh"
# run_rgbd_mapping.sh — RealSense D435i + rtabmap RGB-D 建图
#
# 链路: realsense2_camera → rgbd_odometry(视觉里程计) → rtabmap(SLAM)
#
# 关键点:
#  1. LD_PRELOAD 必须设置，否则 PCL 报 libusb_set_option undefined
#     （和 fastlio2/point_lio 同一个坑）
#  2. 用 aligned_depth_to_color，深度与彩色像素对齐后 rtabmap 才能正确建彩色点云
#  3. IMU 已可用: librealsense 源码构建时开了 FORCE_RSUSB_BACKEND，走 libusb 用户态
#     后端绕过内核 hid-sensor-hub 抢占 HID 接口的问题（apt 版一直报
#     "No HID info provided, IMU is disabled" 就是这个原因）。
#     链路对齐官方 rtabmap_examples/realsense_d435i_color.launch.py:
#     /camera/camera/imu → imu_filter_madgwick → /imu/data → 里程计+SLAM
#  4. rtabmap 与 realsense2_camera 均为自编译，同在工作空间 $ROBOT_CORE
#     （realsense-ros 4.58.3 + librealsense 2.58.3，apt 版已卸载）
#
# 用法:
#   run_rgbd_mapping.sh            启动建图(带 rviz2，显示在 VNC 桌面)
#   run_rgbd_mapping.sh --no-rviz  纯后台建图
#   run_rgbd_mapping.sh status     查看状态
#   run_rgbd_mapping.sh stop       停止
#
# 可视化说明:
#   - 默认用 rviz2 (比 rtabmap_viz 的 PCL 渲染流畅得多，适合 ARM)
#   - rviz2 显示 rtabmap 发布的 /cloud_map(3D点云) 与 /map(2D栅格)
#   - 2D 栅格 /map 实时生成，建图完成后可直接用 map_saver_cli 保存成导航地图
set +u

LOG_DIR=/tmp/rgbd_mapping_logs
# 地图保存目录，可用环境变量覆盖: MAP_DIR=/xxx ./run_rgbd_mapping.sh
MAP_DIR=${MAP_DIR:-$ROBOT_CORE/data/rtabmap_maps}
DB=${DB:-$MAP_DIR/rtabmap_d435i.db}
RTABMAP_WS="$ROBOT_CORE/install/rtabmap"
export LD_PRELOAD=/usr/lib/aarch64-linux-gnu/libusb-1.0.so.0

CAM_NS=/camera/camera
RGB_TOPIC=$CAM_NS/color/image_raw
DEPTH_TOPIC=$CAM_NS/aligned_depth_to_color/image_raw
INFO_TOPIC=$CAM_NS/color/camera_info
# 相机原始 IMU（注意在默认命名空间下是 /camera/camera/imu，不是 /camera/imu）
IMU_RAW_TOPIC=$CAM_NS/imu
# madgwick 滤波后带姿态四元数的 IMU，喂给里程计/SLAM
IMU_TOPIC=/imu/data

need_source() {
    source /opt/ros/humble/setup.bash
    # overlay 自编译的 rtabmap 工作空间（后 source 覆盖 apt 同名包）
    if [ -f "$RTABMAP_WS/install/setup.bash" ]; then
        true
    else
        echo "[警告] $RTABMAP_WS/install/setup.bash 不存在，将回退到 apt 版 rtabmap!"
    fi
}

# 确认关键包来自自编译工作空间（realsense2_camera 与 rtabmap 现在都在同一 ws）
# 用法: show_pkg_origin <包名> <显示名>
show_pkg_origin() {
    local pkg=$1 label=$2 prefix
    prefix=$(ros2 pkg prefix "$pkg" 2>/dev/null)
    if [ -z "$prefix" ]; then
        echo "      [错误] 找不到 $pkg，请先构建: cd $RTABMAP_WS && colcon build --symlink-install"
        return 1
    fi
    case "$prefix" in
        "$RTABMAP_WS"*) echo "      使用自编译 $label: $prefix" ;;
        *) echo "      [注意] 当前使用 apt 版 $label: $prefix" ;;
    esac
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
    echo "=== 组件状态 ==="
    for p in realsense2_camera_node rgbd_odometry "rtabmap_slam/rtabmap" "rviz2.*rgbd_nav"; do
        pid=$(pgrep -f "[${p:0:1}]${p:1}" | head -1)
        [ -n "$pid" ] && echo "  [运行中] $p (pid=$pid)" || echo "  [未运行] $p"
    done
    echo ""
    echo "=== 话题频率 ==="
    # 注意: rtabmap 发布的是 /mapData（根命名空间），不是 /rtabmap/mapData
    # 且 /mapData 仅在地图新增节点时发布，相机静止时无数据属正常
    # （RGBD/LinearUpdate=0.1m、RGBD/AngularUpdate=0.1rad 才触发建图更新）
    for t in "$RGB_TOPIC" "$DEPTH_TOPIC" "$IMU_RAW_TOPIC" "$IMU_TOPIC" /odom /mapData; do
        printf "  %-50s " "$t"
        rate=$(timeout 6 ros2 topic hz "$t" 2>/dev/null | grep -m1 "average rate" | awk '{print $3}')
        if [ -z "$rate" ]; then
            # 首次采样可能因 DDS 发现未完成而失败，放宽窗口重试
            rate=$(timeout 10 ros2 topic hz "$t" 2>/dev/null | grep -m1 "average rate" | awk '{print $3}')
        fi
        [ -n "$rate" ] && echo "$rate Hz" || echo "无数据"
    done
    echo ""
    echo "地图数据库: $DB"
    [ -f "$DB" ] && echo "  大小: $(du -h "$DB" | cut -f1)"
    exit 0
    ;;
stop)
    echo "停止建图..."
    # 只杀建图专用的 rviz2（按配置文件名精确匹配，避免误杀 nav2 等其他 rviz2）
    pkill -f "rviz2.*rgbd_nav.rviz" 2>/dev/null
    pkill -f "rtabmap_slam/rtabmap" 2>/dev/null
    pkill -f rgbd_odometry 2>/dev/null
    pkill -f imu_filter_madgwick 2>/dev/null
    pkill -f realsense2_camera_node 2>/dev/null
    # 等优雅退出，超时未退则强杀（GUI 的 rviz2 对 SIGTERM 常不响应）
    for pat in "rviz2.*rgbd_nav.rviz" "rtabmap_slam/rtabmap" rgbd_odometry imu_filter_madgwick realsense2_camera_node; do
        if ! wait_gone "[${pat:0:1}]${pat:1}" 10; then
            echo "  强制结束: $pat"
            pkill -9 -f "[${pat:0:1}]${pat:1}" 2>/dev/null
        fi
    done
    echo "已停止（地图保留在 $DB）"
    exit 0
    ;;
esac

USE_RVIZ=1
[ "$1" = "--no-rviz" ] && USE_RVIZ=0

mkdir -p "$LOG_DIR" "$MAP_DIR"
rm -f "$LOG_DIR"/*.log   # 清空旧日志，避免残留关键字干扰就绪检测
need_source

echo "============================================"
echo " RealSense D435i + rtabmap RGB-D 建图"
echo " 地图数据库: $DB"
echo " 日志目录:   $LOG_DIR"
echo "============================================"
show_pkg_origin rtabmap_slam rtabmap
show_pkg_origin realsense2_camera realsense2_camera

# ── 1. 相机 ──
# 关键参数（对照官方 rtabmap_examples/realsense_d435i_color.launch.py）:
#  enable_sync=true : 驱动层帧同步。不开它的话彩色与深度不是同一时刻的，
#                     特征点关联的 3D 深度就不自洽，实测表现为 RANSAC 0 个内点
#  640x360x30       : 降分辨率，ARM 上 CPU 开销小很多，建图精度足够
#  注意: 4.58.x 驱动参数名是 color_profile/depth_profile（旧名 profile 已废弃），
#        emitter_enabled 在新驱动中已移除（激光投射器默认开启）
echo "[1/3] 启动 D435i (帧同步+IR发射器, 640x360@30)..."
pkill -9 -f realsense2_camera_node 2>/dev/null
wait_gone "[r]ealsense2_camera_node" 5
nohup ros2 launch realsense2_camera rs_launch.py \
    enable_depth:=true enable_color:=true align_depth.enable:=true \
    enable_sync:=true \
    enable_gyro:=true enable_accel:=true unite_imu_method:=2 \
    rgb_camera.color_profile:=640x360x30 \
    depth_module.depth_profile:=640x360x30 \
    > "$LOG_DIR/camera.log" 2>&1 &

if ! wait_log "$LOG_DIR/camera.log" "Node Is Up" 60; then
    echo "      [失败] 相机未就绪，查看 $LOG_DIR/camera.log"
    exit 1
fi
echo "      相机就绪"

# ── 1.5 IMU（官方示例用 IMU 做帧间预测，快速运动不重影的关键）──
# RSUSB 后端下 IMU 应当直接可用；仍保留检测与回退，避免驱动异常时整条链路起不来
USE_IMU=0
if timeout 8 ros2 topic list 2>/dev/null | grep -qx "$IMU_RAW_TOPIC"; then
    USE_IMU=1
fi
if [ "$USE_IMU" = "1" ]; then
    echo "      检测到 IMU（$IMU_RAW_TOPIC），启用 madgwick 姿态滤波（官方链路）"
    nohup ros2 run imu_filter_madgwick imu_filter_madgwick_node --ros-args \
        -p use_mag:=false -p world_frame:=enu -p publish_tf:=false \
        -r imu/data_raw:="$IMU_RAW_TOPIC" \
        > "$LOG_DIR/imu.log" 2>&1 &
    wait_node /imu_filter_madgwick 15 || { echo "      [警告] madgwick 未就绪，回退纯视觉"; USE_IMU=0; }
else
    echo "      [注意] 无 IMU 话题，回退纯视觉里程计"
    echo "             排查: rs-enumerate-devices | grep -A2 'Motion Module'（应列出 Accel/Gyro）"
    echo "             若 Motion Module 缺失，检查 librealsense 是否开了 FORCE_RSUSB_BACKEND"
fi

# ── 2. 视觉里程计 ──
# approx_sync=false: 配合上面的 enable_sync，用精确时间同步
# wait_imu_to_init=true: 官方链路，用 IMU 做帧间预测，快速运动不易跟丢；
#   无 IMU 时自动置 false（否则会一直等 IMU 而不出位姿）
echo "[2/3] 启动 rgbd_odometry (视觉里程计)..."
# Odom/ResetCountdown=8 : 连续失败 8 次才重置。重置会把轨迹砍断导致地图错位，
#   宁可多等几帧也不轻易重置（上次建图 5 分钟内 6 次重置是帧间不重合的主因）
# Vis/MinInliers=12 : 降低内点门槛（默认20），快速运动/弱纹理时不易跟丢
#   注意: 参数名是 Vis/MinInliers，不是 Odom/MinInliers（后者在 RTAB-Map 中不存在，
#   传了也不报错、默默失效，日志会仍旧显示 "Not enough inliers x/20"）
# Vis/Iterations=1000 : RANSAC 迭代次数（默认300）。曾出现 matches=46 但 inliers=0，
#   说明匹配够但迭代不够找不到解，加大迭代数可救回快速运动帧
# publish_null_when_lost=false : 丢失时不发布“空位姿”，防止 rtabmap 收到无效里程计
nohup ros2 run rtabmap_odom rgbd_odometry --ros-args \
    -p frame_id:=camera_link \
    -p approx_sync:=false \
    -p wait_imu_to_init:=$([ "$USE_IMU" = "1" ] && echo true || echo false) \
    -p publish_null_when_lost:=false \
    -p "Odom/ResetCountdown:='8'" \
    -p "Vis/MinInliers:='12'" \
    -p "Vis/Iterations:='1000'" \
    $([ "$USE_IMU" = "1" ] && echo "-r imu:=$IMU_TOPIC") \
    -r rgb/image:=$RGB_TOPIC \
    -r depth/image:=$DEPTH_TOPIC \
    -r rgb/camera_info:=$INFO_TOPIC \
    > "$LOG_DIR/odom.log" 2>&1 &
if ! wait_node /rgbd_odometry 30; then
    echo "      [失败] 里程计节点未注册，查看 $LOG_DIR/odom.log"
    exit 1
fi
echo "      里程计已启动"

# ── 等待里程计锁定画面，再启动 SLAM ──
# 背景: 开机瞬间若里程计丢失，位姿会发散且无法自恢复，导致 rtabmap 一帧都建不出来。
# 策略: 监测 odom.log 的失败计数，失败停止增长即认为已锁定（ResetCountdown 会自动重置直到锁定）
echo "      等待里程计锁定画面..."
lock_i=0
prev_fails=-1
while [ "$lock_i" -lt 120 ]; do   # 最多等 60s
    fails=$(grep -c "Registration failed" "$LOG_DIR/odom.log" 2>/dev/null)
    if [ "$fails" -gt 0 ] && [ "$fails" = "$prev_fails" ]; then
        echo "      里程计已锁定（可开始缓慢移动相机建图）"
        break
    fi
    if [ "$lock_i" -ge 20 ] && [ "$fails" = "0" ]; then
        echo "      里程计已锁定（全程无失败）"
        break
    fi
    prev_fails=$fails
    sleep 1
    lock_i=$((lock_i+2))
done
[ "$lock_i" -ge 120 ] && echo "      [警告] 60s 内未确认稳定锁定，继续启动（请对准纹理丰富的场景）"

# ── 3. rtabmap SLAM ──
# 注意: rtabmap 的核心参数（Rtabmap/*、Vis/*、Grid/* 这类）必须用字符串传，
# 写成 double/int 会抛 InvalidParameterTypeException 直接崩溃。
# delete_db_on_start: 每次重新建图。要增量建图请去掉该参数
# IMU 接入 SLAM 端（对齐官方示例）: 提供重力方向约束，图优化时抑制地图倾斜
#
# 2D 导航地图参数（/map 话题，OccupancyGrid，通过 config_path ini 加载，
# 不能用命令行 -p 传 Grid/3D——rcl 参数名段不允许数字开头）:
#   Grid/3D=false            : 只生成 2D 栅格(不做 OctoMap)，省内存省时间
#   Grid/RayTracing=true     : 障碍物与相机之间的区域标记为 free，导航必需
#   Grid/CellSize=0.05       : 5cm 栅格，导航标准分辨率
#   Grid/RangeMax=8.0        : 单帧栅格最大探测距离(默认5m)
#   Grid/MapFrameProjection=true: 在地图系投影(有 IMU 重力约束，姿态稳定)
#   cloud_output_voxelized=true : /cloud_map 点云体素化后发布，rviz2 显示流畅
# 注: rtabmap 退出时会写回全量参数到 ini，故每次启动前重新生成干净版本
cat > "$MAP_DIR/grid_params.ini" <<'EOF'
# RTAB-Map 核心参数（Grid 2D 导航地图配置）
# 由 rtabmap 节点 config_path 参数加载（RTAB-Map 的 ini 格式: / 用 \ 分隔）
[Core]
Grid\3D=false
Grid\RayTracing=true
Grid\CellSize=0.05
Grid\RangeMax=8.0
Grid\MapFrameProjection=true
# 回环/邻近注册也用更大 RANSAC 迭代，快速运动帧间验证更可靠（默认300）
Vis\Iterations=1000
EOF
echo "[3/3] 启动 rtabmap SLAM..."
nohup ros2 run rtabmap_slam rtabmap --ros-args \
    -p frame_id:=camera_link \
    -p subscribe_depth:=true \
    -p approx_sync:=false \
    -p database_path:="$DB" \
    -p config_path:="$MAP_DIR/grid_params.ini" \
    -p "Rtabmap/DetectionRate:='0.5'" \
    -p cloud_output_voxelized:=true \
    $([ "$USE_IMU" = "1" ] && echo "-r imu:=$IMU_TOPIC") \
    -r rgb/image:=$RGB_TOPIC \
    -r depth/image:=$DEPTH_TOPIC \
    -r rgb/camera_info:=$INFO_TOPIC \
    -p delete_db_on_start:=true \
    > "$LOG_DIR/rtabmap.log" 2>&1 &
if ! wait_node /rtabmap 30; then
    echo "      [失败] rtabmap 节点未注册，查看 $LOG_DIR/rtabmap.log"
    exit 1
fi
echo "      SLAM 已启动"

# ── 可视化（rviz2，比 rtabmap_viz 流畅）──
# 显示内容(见 rgbd_nav.rviz): /cloud_map 3D彩色点云 + /map 2D栅格 + TF + Grid
# rtabmap 节点已发布这两个话题，rviz2 无需订阅原始图像，负载低
if [ "$USE_RVIZ" = "1" ]; then
    # 走 VNC: 让窗口显示在设备物理桌面上，Mac 通过 VNC 观看
    export DISPLAY=:0
    export XAUTHORITY=$HOME/.Xauthority
    RVIZ_CFG=${RVIZ_CFG:-$ROBOT_CORE/data/rtabmap_maps/rgbd_nav.rviz}
    echo "[+] 启动 rviz2 (显示在设备桌面，请用 VNC 查看)..."
    nohup rviz2 -d "$RVIZ_CFG" > "$LOG_DIR/rviz.log" 2>&1 &
    if wait_node /rviz2 30; then
        echo "      rviz2 已就绪"
    else
        echo "      [警告] rviz2 未就绪，查看 $LOG_DIR/rviz.log（不影响建图）"
    fi
fi

echo ""
echo "建图已启动! 拿着相机缓慢移动即可建图。"
echo "  查看状态: $0 status"
echo "  停止建图: $0 stop"
