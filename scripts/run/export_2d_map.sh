#!/bin/bash
# export_2d_map.sh — 把 rtabmap 数据库离线导出为 2D 导航地图 (pgm+yaml，nav2 格式)
#
# 原理:
#   1. 启动 rtabmap 节点加载已有 .db（不需要相机/雷达在线）
#   2. 调用 /publish_map 服务发布整张地图（含 2D 栅格 /map）
#   3. 用 nav2 的 map_saver_cli 把 /map 保存为 map.pgm + map.yaml
#   4. 停止 rtabmap 节点
#
# 用法:
#   ./export_2d_map.sh
#   DB=/path/to/xxx.db OUT=/path/to/map ./export_2d_map.sh   # 自定义路径
#
# 输出:
#   $OUT.pgm + $OUT.yaml（默认 /data/sf_code/rtabmap_maps/nav2_map.{pgm,yaml}）
#   可直接被 nav2 map_server 加载:
#     ros2 run nav2_map_server map_server --ros-args -p yaml_filename:=<OUT>.yaml
set +u

MAP_DIR=${MAP_DIR:-/data/sf_code/rtabmap_maps}
DB=${DB:-$MAP_DIR/rtabmap_d435i.db}
OUT=${OUT:-$MAP_DIR/nav2_map}      # 输出前缀（不含扩展名）
RTABMAP_WS=/data/sf_code/rtabmap
LOG_FILE=/tmp/export_2d_map.log
export LD_PRELOAD=/usr/lib/aarch64-linux-gnu/libusb-1.0.so.0

# 防止与正在运行的建图冲突
if pgrep -f "[r]tabmap_slam/rtabmap" >/dev/null 2>&1; then
    echo "[错误] rtabmap 节点正在运行（可能正在建图），请先停止:"
    echo "       /data/sf_code/run_rgbd_mapping.sh stop"
    exit 1
fi

if [ ! -f "$DB" ]; then
    echo "[错误] 数据库不存在: $DB"
    echo "       可用环境变量指定: DB=/path/to/xxx.db $0"
    exit 1
fi

source /opt/ros/humble/setup.bash
if [ -f "$RTABMAP_WS/install/setup.bash" ]; then
    source "$RTABMAP_WS/install/setup.bash"
fi
export LD_LIBRARY_PATH=$RTABMAP_WS/install/rtabmap/lib:/opt/ros/humble/lib/aarch64-linux-gnu:/opt/ros/humble/lib:$LD_LIBRARY_PATH

echo "============================================"
echo " rtabmap 数据库 → 2D 导航地图"
echo " 数据库: $DB ($(du -h "$DB" | cut -f1))"
echo " 输出:   $OUT.pgm / $OUT.yaml"
echo "============================================"

# ── 1. 启动 rtabmap 节点加载数据库 ──
# 关键参数:
#   latch=true            : 地图话题 Transient Local，晚订阅也能收到整张地图
#   publish_tf=false      : 无传感器数据，不发布 TF（避免空 tf 干扰）
#   config_path=grid_params.ini : 核心 Grid 参数（rcl 命令行不能传 Grid/3D）
#                                Grid/3D=false 只生成 2D 栅格;
#                                Grid/RayTracing=true 障碍物与相机间标记 free;
#                                Grid/MapFrameProjection=true 地图系投影
# 注: rtabmap 退出时会写回全量参数到 ini，故启动前重新生成干净版本
cat > "$MAP_DIR/grid_params.ini" <<'EOF'
# RTAB-Map 核心参数（Grid 2D 导航地图配置）
# 由 rtabmap 节点 config_path 参数加载（RTAB-Map 的 ini 格式: / 用 \ 分隔）
[Core]
Grid\3D=false
Grid\RayTracing=true
Grid\CellSize=0.05
Grid\RangeMax=8.0
Grid\MapFrameProjection=true
EOF
echo "[1/4] 启动 rtabmap 节点加载数据库..."
nohup ros2 run rtabmap_slam rtabmap --ros-args \
    -p database_path:="$DB" \
    -p publish_tf:=false \
    -p latch:=true \
    -p config_path:="$MAP_DIR/grid_params.ini" \
    > "$LOG_FILE" 2>&1 &

# ── 2. 等待数据库加载完成（事件驱动：轮询日志）──
i=0
while [ "$i" -lt 90 ]; do
    grep -q "Setup callbacks" "$LOG_FILE" 2>/dev/null && break
    sleep 1; i=$((i+1))
done
if [ "$i" -ge 90 ]; then
    echo "[失败] rtabmap 节点未就绪，查看 $LOG_FILE"
    pkill -f "[r]tabmap_slam/rtabmap" 2>/dev/null
    exit 1
fi
echo "      数据库已加载完成"

# ── 3. 发布整张地图（2D 栅格 /map + 3D /cloud_map），带重试（服务发现可能慢）──
echo "[2/4] 发布整张地图..."
ok=0
i=0
while [ "$i" -lt 8 ]; do
    # timeout 保护: 服务发现失败时 ros2 service call 会无限等待
    if timeout 8 ros2 service call /publish_map rtabmap_msgs/srv/PublishMap \
            "{global_map: true, optimized: true, graph_only: false}" 2>&1 | grep -q "success"; then
        ok=1; break
    fi
    sleep 1; i=$((i+1))
done
if [ "$ok" = "0" ]; then
    echo "[失败] publish_map 服务调用失败，查看 $LOG_FILE"
    pkill -f "[r]tabmap_slam/rtabmap" 2>/dev/null
    exit 1
fi

# 等 /map 真的发布出消息（大图生成 + 优化需要几秒）
i=0
while [ "$i" -lt 10 ]; do
    if timeout 5 ros2 topic echo /map --once 2>/dev/null >/dev/null; then
        echo "      /map 已发布"
        break
    fi
    sleep 1; i=$((i+1))
done
[ "$i" -ge 10 ] && echo "[警告] 50s 内未等到 /map 消息，仍尝试保存..."

# ── 4. 保存为 nav2 格式 ──
echo "[3/4] 保存 2D 栅格地图 (map_saver_cli)..."
rm -f "$OUT.pgm" "$OUT.yaml"
if ! timeout 60 ros2 run nav2_map_server map_saver_cli -f "$OUT" \
        --ros-args -p mode:=trinary -p map_topic:=/map 2>&1 | tail -3; then
    echo "[失败] map_saver_cli 保存失败"
    pkill -f "[r]tabmap_slam/rtabmap" 2>/dev/null
    exit 1
fi
if [ ! -f "$OUT.pgm" ] || [ ! -f "$OUT.yaml" ]; then
    echo "[失败] 输出文件不存在: $OUT.pgm / $OUT.yaml"
    pkill -f "[r]tabmap_slam/rtabmap" 2>/dev/null
    exit 1
fi

# ── 5. 清理 ──
echo "[4/4] 停止 rtabmap 节点..."
pkill -f "[r]tabmap_slam/rtabmap" 2>/dev/null
sleep 3
# 恢复干净的 grid_params.ini（rtabmap 退出时会把全量参数写回）
cat > "$MAP_DIR/grid_params.ini" <<'EOF'
# RTAB-Map 核心参数（Grid 2D 导航地图配置）
# 由 rtabmap 节点 config_path 参数加载（RTAB-Map 的 ini 格式: / 用 \ 分隔）
[Core]
Grid\3D=false
Grid\RayTracing=true
Grid\CellSize=0.05
Grid\RangeMax=8.0
Grid\MapFrameProjection=true
EOF

echo ""
echo "导出完成!"
echo "  地图: $OUT.pgm / $OUT.yaml"
echo ""
echo "导航加载方式（nav2 map_server）:"
echo "  ros2 run nav2_map_server map_server --ros-args -p yaml_filename:=$OUT.yaml"
echo "  ros2 run nav2_map_server lifecycle_manager_nav2 --ros-args -p autostart:=true -p node_names:=[map_server]"
