#!/usr/bin/env python3
"""probe_rs_frame.py — 检查 D435i 实际画面内容

诊断视觉里程计 quality=0 的原因：
  - 彩色图是否全黑/过曝（镜头被挡）
  - 深度图有效像素比例（正对无纹理平面或距离过近时深度会大面积无效）
"""
import os
import sys
import threading

import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image


class Probe(Node):
    def __init__(self):
        super().__init__("rs_frame_probe")
        self.bridge = CvBridge()
        self.color = None
        self.depth = None
        self.create_subscription(
            Image, "/camera/camera/color/image_raw", self._on_color, 10)
        self.create_subscription(
            Image, "/camera/camera/aligned_depth_to_color/image_raw",
            self._on_depth, 10)

    def _on_color(self, msg):
        if self.color is None:
            self.color = self.bridge.imgmsg_to_cv2(msg, "bgr8")

    def _on_depth(self, msg):
        if self.depth is None:
            self.depth = self.bridge.imgmsg_to_cv2(msg, "passthrough")


def main():
    rclpy.init()
    node = Probe()
    threading.Thread(target=rclpy.spin, args=(node,), daemon=True).start()

    import time
    end = time.time() + 20
    while time.time() < end and (node.color is None or node.depth is None):
        time.sleep(0.3)

    if node.color is None or node.depth is None:
        print(f"未收到图像 (color={node.color is not None} "
              f"depth={node.depth is not None})")
        os._exit(1)

    c = node.color
    gray = c.mean(axis=2)
    print("=== 彩色图 ===")
    print(f"  尺寸: {c.shape}")
    print(f"  亮度: 均值={gray.mean():.1f} 最小={gray.min()} 最大={gray.max()}")
    print(f"  纹理(标准差): {gray.std():.1f}   <10 基本是无纹理平面/遮挡")

    d = node.depth.astype(np.float32)
    valid = d > 0
    print("=== 深度图 ===")
    print(f"  尺寸: {d.shape}")
    print(f"  有效像素: {100.0 * valid.sum() / d.size:.1f}%")
    if valid.sum() > 0:
        v = d[valid] / 1000.0
        print(f"  距离: 最近={v.min():.3f}m 中位={np.median(v):.3f}m "
              f"最远={v.max():.3f}m")

    print("\n=== 判定 ===")
    if gray.mean() < 15:
        print("彩色图几乎全黑 → 镜头被遮挡或环境无光，这就是 quality=0 的原因")
    elif gray.std() < 10:
        print("画面几乎无纹理 → 正对白墙/桌面等平面，提不到特征点")
    elif valid.sum() / d.size < 0.1:
        print("深度几乎全无效 → 距离过近(<0.2m)或过远，请后退到 0.5~3m")
    else:
        print("画面看起来正常，quality=0 可能是相机完全静止导致；请移动相机")
    sys.stdout.flush()
    os._exit(0)


if __name__ == "__main__":
    main()
