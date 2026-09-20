#!/usr/bin/env python3
"""
Slash 腿高控制工具 — GUI 滑块控制腿高
发布 Float32 到 /leg_height_cmd 话题
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32

import tkinter as tk
from tkinter import ttk


class LegHeightGUI(Node):
    def __init__(self, master):
        super().__init__("leg_height_gui")

        # 发布腿高指令
        self.height_pub = self.create_publisher(Float32, "leg_height_cmd", 10)

        # 当前腿高
        self.current_height = 0.25

        # 创建 GUI
        self.master = master
        master.title("Slash 腿高控制")
        master.geometry("400x250")
        master.resizable(False, False)

        # 标题
        ttk.Label(master, text="Slash 腿高控制", font=("Arial", 16, "bold")).pack(pady=10)

        # 当前腿高显示
        self.height_label = ttk.Label(master, text="腿高: 0.250 m", font=("Arial", 14))
        self.height_label.pack(pady=5)

        # 滑块
        self.slider_frame = ttk.Frame(master)
        self.slider_frame.pack(pady=10, padx=20, fill="x")

        ttk.Label(self.slider_frame, text="0.15").grid(row=0, column=0)

        self.slider = ttk.Scale(
            self.slider_frame, from_=0.15, to=0.40,
            orient="horizontal", length=280,
            command=self._on_slider_change
        )
        self.slider.grid(row=0, column=1, padx=10)
        self.slider.set(0.25)  # 初始值

        ttk.Label(self.slider_frame, text="0.40").grid(row=0, column=2)

        # 快捷按钮
        btn_frame = ttk.Frame(master)
        btn_frame.pack(pady=10)

        ttk.Button(btn_frame, text="最低 (0.15)", command=lambda: self._set_height(0.15)).pack(side="left", padx=5)
        ttk.Button(btn_frame, text="站立 (0.25)", command=lambda: self._set_height(0.25)).pack(side="left", padx=5)
        ttk.Button(btn_frame, text="最高 (0.40)", command=lambda: self._set_height(0.40)).pack(side="left", padx=5)

        # 状态栏
        self.status_var = tk.StringVar(value="就绪")
        ttk.Label(master, textvariable=self.status_var).pack(side="bottom", pady=5)

    def _on_slider_change(self, value):
        """滑块变化时更新腿高"""
        h = float(value)
        self.current_height = h
        self.height_label.config(text=f"腿高: {h:.3f} m")

    def _set_height(self, h):
        """设置指定腿高"""
        self.slider.set(h)
        self.current_height = h
        self.height_label.config(text=f"腿高: {h:.3f} m")

    def publish(self):
        """发布腿高指令"""
        msg = Float32()
        msg.data = self.current_height
        self.height_pub.publish(msg)


def main():
    rclpy.init()
    node = None

    # 创建 GUI
    root = tk.Tk()

    # 创建节点
    node = LegHeightGUI(root)

    # 定时器发布
    def periodic_publish():
        node.publish()
        root.after(100, periodic_publish)  # 10Hz

    root.after(100, periodic_publish)

    # 退出回调
    def on_close():
        root.destroy()
        if node:
            node.destroy_node()
        rclpy.shutdown()

    root.protocol("WM_DELETE_WINDOW", on_close)

    # 主循环
    try:
        root.mainloop()
    except KeyboardInterrupt:
        on_close()


if __name__ == "__main__":
    main()
