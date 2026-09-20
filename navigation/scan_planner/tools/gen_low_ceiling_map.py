#!/usr/bin/env python3
"""生成低天花板 / 桌子测试地图 (HAVEN 高度自适应验证)

机器人几何 (由 URDF 实测):
    base_link 世界高度 = 腿高 + 0.0322   (轮半径 0.0922 - 髋电机高 0.06)
    机器人顶部          = base_link + 0.328  (雷达顶)
    base_z 范围 [0.17, 0.40], 标称 0.28  ->  顶部 [0.498, 0.728], 标称 0.608

因此净空高度 z_ceil 与通行性的关系 (安全余量 0.03):
    z_ceil >= 0.64  ->  无需下蹲
    0.53 <= z_ceil < 0.64  ->  必须下蹲
    z_ceil <  0.53  ->  完全无法通过

场景 (20x20 封闭房间):
  1. 四周边界墙, 外部不可达
  2. 中央隔墙 + 3m 宽门洞, 门洞上方横梁底面 0.58 m
     -> 两个半区之间唯一通路, 强制下蹲 (base_z <= 0.222)
  3. 若干张桌子 (桌面底面 0.56 m + 四条桌腿)
     -> 可从桌下钻过, 也可绕行, 用于观察高度自适应决策

用法:
    python3 gen_low_ceiling_map.py [输出路径]
默认输出: <repo>/map_low_ceiling.pcd

启动 (起点 -8,-8 在左半区, 目标点设在右半区触发穿越):
    ros2 launch scan_planner run.launch.py robot_model:=slash \\
        use_pcd_map:=true \\
        pcd_map_file:=/home/dz/ros_ws/nav/SCAN-Planner/map_low_ceiling.pcd
"""

import os
import sys

import numpy as np

# ---------------- 场景参数 ----------------
RES = 0.05             # 点间距, 与栅格分辨率一致

ROOM = 20.0            # 房间边长 (与 map_size_x/y 一致)
WALL_THICK = 0.2       # 墙厚
WALL_HEIGHT = 1.5      # 墙高

DOOR_HALF_W = 1.5      # 门洞半宽
BEAM_Z_BOTTOM = 0.58   # 门洞横梁底面 -> 需下蹲至 base_z<=0.222

TABLE_Z_BOTTOM = 0.56  # 桌面底面 -> 需下蹲至 base_z<=0.202
TABLE_THICK = 0.06     # 桌面厚度
TABLE_LEG = 0.08       # 桌腿截面边长

# (中心x, 中心y, 长x, 宽y)
TABLES = [
    (-5.0, 3.0, 3.0, 2.0),
    (-4.0, -4.0, 2.4, 2.0),
    (5.0, 4.0, 3.0, 2.0),
    (4.5, -3.5, 2.4, 2.0),
]


def box_points(x_range, y_range, z_range, res=RES):
    xs = np.arange(x_range[0], x_range[1] + 1e-9, res)
    ys = np.arange(y_range[0], y_range[1] + 1e-9, res)
    zs = np.arange(z_range[0], z_range[1] + 1e-9, res)
    gx, gy, gz = np.meshgrid(xs, ys, zs, indexing="ij")
    return np.stack([gx.ravel(), gy.ravel(), gz.ravel()], axis=1)


def table_points(cx, cy, lx, ly):
    """一张桌子: 桌面板 + 四条桌腿"""
    hx, hy = lx / 2.0, ly / 2.0
    parts = [
        # 桌面
        box_points((cx - hx, cx + hx), (cy - hy, cy + hy),
                   (TABLE_Z_BOTTOM, TABLE_Z_BOTTOM + TABLE_THICK)),
    ]
    # 四条桌腿 (从地面到桌面底部)
    for sx in (-1.0, 1.0):
        for sy in (-1.0, 1.0):
            lx0 = cx + sx * (hx - TABLE_LEG)
            ly0 = cy + sy * (hy - TABLE_LEG)
            parts.append(box_points(
                (min(lx0, lx0 + TABLE_LEG), max(lx0, lx0 + TABLE_LEG)),
                (min(ly0, ly0 + TABLE_LEG), max(ly0, ly0 + TABLE_LEG)),
                (0.0, TABLE_Z_BOTTOM)))
    return parts


def main():
    out_path = sys.argv[1] if len(sys.argv) > 1 else os.path.join(
        os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
        "map_low_ceiling.pcd")

    h = ROOM / 2.0
    t = WALL_THICK
    d = DOOR_HALF_W
    zw = (0.0, WALL_HEIGHT)

    parts = [
        # ---- 房间四周边界墙 (外部不可达) ----
        box_points((-h, h), (h - t, h), zw),        # +y
        box_points((-h, h), (-h, -h + t), zw),      # -y
        box_points((h - t, h), (-h, h), zw),        # +x
        box_points((-h, -h + t), (-h, h), zw),      # -x

        # ---- 中央隔墙 (x=0), 门洞两侧为实墙 ----
        box_points((-t / 2, t / 2), (d, h), zw),
        box_points((-t / 2, t / 2), (-h, -d), zw),

        # ---- 门洞上方的低矮横梁: 仅存在于 BEAM_Z_BOTTOM 以上 ----
        box_points((-t / 2, t / 2), (-d, d), (BEAM_Z_BOTTOM, WALL_HEIGHT)),
    ]

    for tb in TABLES:
        parts.extend(table_points(*tb))

    pts = np.concatenate(parts, axis=0).astype(np.float32)

    with open(out_path, "w") as f:
        f.write("# .PCD v0.7 - Point Cloud Data file format\n")
        f.write("VERSION 0.7\n")
        f.write("FIELDS x y z\n")
        f.write("SIZE 4 4 4\n")
        f.write("TYPE F F F\n")
        f.write("COUNT 1 1 1\n")
        f.write(f"WIDTH {len(pts)}\n")
        f.write("HEIGHT 1\n")
        f.write("VIEWPOINT 0 0 0 1 0 0 0\n")
        f.write(f"POINTS {len(pts)}\n")
        f.write("DATA ascii\n")
        for p in pts:
            f.write(f"{p[0]:.3f} {p[1]:.3f} {p[2]:.3f}\n")

    print(f"写入 {len(pts)} 点到 {out_path}")
    print(f"房间: {ROOM}x{ROOM} m 封闭, 墙高 {WALL_HEIGHT} m")
    print(f"门洞: x=0, y∈[{-d},{d}], 横梁底面 {BEAM_Z_BOTTOM} m "
          f"-> 需下蹲至 base_z<={BEAM_Z_BOTTOM - 0.358:.3f}")
    print(f"桌子 x{len(TABLES)}: 桌面底面 {TABLE_Z_BOTTOM} m "
          f"-> 钻桌下需 base_z<={TABLE_Z_BOTTOM - 0.358:.3f}")
    print("标称 base_z=0.28 (顶部 0.608) 无法直接通过, 必须降高")


if __name__ == "__main__":
    main()
