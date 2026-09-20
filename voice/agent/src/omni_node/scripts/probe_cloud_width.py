#!/usr/bin/env python3
"""统计 /unilidar/cloud 每帧点数与点的距离分布

point_lio 的 unilidar_handler 会丢弃距离 ≤ blind(0.5m) 的点，
若绝大多数点都在盲区内，预处理后就为空，表现为 lose lidar。
"""
import math
import os
import sys
import threading
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2

BLIND = 0.5


class Counter(Node):
    def __init__(self):
        super().__init__("cloud_width_probe")
        self.widths = []
        self.field_names = None
        self.dist_samples = []      # 抽样帧的点距离
        self.beyond_blind = []      # 每帧超出盲区的点数
        self.create_subscription(PointCloud2, "/unilidar/cloud", self._cb, 50)

    def _cb(self, m):
        n = m.width * max(1, m.height)
        self.widths.append(n)
        if self.field_names is None:
            self.field_names = [f.name for f in m.fields]
        # 只对前几帧做逐点解析，避免拖慢回调
        if len(self.beyond_blind) < 30:
            cnt = 0
            dists = []
            for p in point_cloud2.read_points(
                    m, field_names=("x", "y", "z"), skip_nans=True):
                d = math.sqrt(float(p[0]) ** 2 + float(p[1]) ** 2 + float(p[2]) ** 2)
                dists.append(d)
                if d > BLIND:
                    cnt += 1
            self.beyond_blind.append(cnt)
            if len(self.dist_samples) < 3:
                self.dist_samples.append(dists)


def main():
    rclpy.init()
    node = Counter()
    threading.Thread(target=rclpy.spin, args=(node,), daemon=True).start()
    time.sleep(12)

    w = node.widths
    if not w:
        print("未收到任何点云")
        os._exit(1)
    zero = sum(1 for x in w if x == 0)
    small = sum(1 for x in w if 0 < x < 500)
    ok = sum(1 for x in w if x >= 500)
    print(f"采样帧数: {len(w)}  ({len(w)/12:.1f} Hz)")
    print(f"  空帧(0点)     : {zero}  ({zero/len(w)*100:.0f}%)")
    print(f"  残缺帧(<500点): {small}  ({small/len(w)*100:.0f}%)")
    print(f"  正常帧(>=500) : {ok}  ({ok/len(w)*100:.0f}%)")
    print(f"点数范围: {min(w)} ~ {max(w)}, 平均 {sum(w)/len(w):.0f}")
    print(f"PointCloud2 字段: {node.field_names}")

    bb = node.beyond_blind
    if bb:
        print(f"\n═══ blind={BLIND}m 过滤后剩余点数 (point_lio 实际用的) ═══")
        print(f"采样 {len(bb)} 帧: 最少 {min(bb)}, 最多 {max(bb)}, 平均 {sum(bb)/len(bb):.0f}")
        empty_after = sum(1 for x in bb if x == 0)
        print(f"过滤后为空的帧: {empty_after}/{len(bb)} "
              f"({empty_after/len(bb)*100:.0f}%)  ← 这些帧会触发 lose lidar")
    if node.dist_samples:
        d = node.dist_samples[0]
        d_sorted = sorted(d)
        print(f"\n首帧点距离分布: 最近 {d_sorted[0]:.3f}m, "
              f"中位数 {d_sorted[len(d_sorted)//2]:.3f}m, 最远 {d_sorted[-1]:.3f}m")
        within = sum(1 for x in d if x <= BLIND)
        print(f"首帧在盲区内(≤{BLIND}m)的点: {within}/{len(d)} "
              f"({within/len(d)*100:.0f}%)")
    sys.stdout.flush()
    os._exit(0)


if __name__ == "__main__":
    main()
