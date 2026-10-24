#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
轮式腿机器人 Mac 上位机
======================

功能:
    1. 串口实时显示机器人状态 (里程计 / IMU / 6 电机反馈 / SBUS / 机器状态 / 曲线)
    2. 开始记录 / 结束记录 按钮, 支持自定义名称的全量录制
       (解码 CSV + 串口原始二进制 + 元数据 JSON)

协议: 与下位机固件 LeggedRobot.ino 一致 (帧头 0x7B 0x2D, CRC16-Modbus,
      ROS_body_t 182B 全量帧 / ROS_body_t1 124B 精简帧 / 181B 变体帧)

运行: python3 pc_monitor.py
说明: 仅依赖 Python 标准库; GUI 使用纯 tk 控件, 兼容系统自带的老版本 Tk 8.5。
"""

import os
# 屏蔽 macOS 系统 Tk 的弃用警告 (需在 import tkinter 前设置)
os.environ.setdefault("TK_SILENCE_DEPRECATION", "1")

import queue
import threading
import time
import tkinter as tk
from tkinter import filedialog, messagebox

from mac_serial import SerialPort, SerialPortError, list_ports
from recorder import Recorder, RecorderError, sanitize_name
from robot_protocol import (FRAME_FULL_LEN, FRAME_LITE_LEN, MOTOR_NAMES,
                            FrameParser, decode_frame)

APP_TITLE = "轮式腿机器人上位机"
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
DEFAULT_LOG_DIR = os.path.join(SCRIPT_DIR, "logs")

BAUDRATES = ["9600", "115200", "230400", "460800", "921600",
             "1000000", "2000000", "3000000"]
FRAME_FILTERS = [
    ("auto", "自动 (182B + 124B)"),
    ("full", "仅全量 ROS_body_t (182B)"),
    ("lite", "仅精简 ROS_body_t1 (124B)"),
]

# ---- 配色 (浅色主题, 纯 tk 控件在 Tk 8.5 上也可正常渲染) ----
BG = "#e8ebf0"          # 窗口/面板背景
BG_FRAME = "#dfe4ec"    # LabelFrame 背景
BG_VALUE = "#ffffff"    # 数值标签背景
FG_TEXT = "#2e3440"     # 普通文字
FG_DIM = "#7a839e"      # 次要文字
FG_BLUE = "#1c64c8"     # 统计数值
FG_GREEN = "#1a7f37"    # 里程计数值
FG_AMBER = "#9a6700"    # IMU 数值
FG_PURPLE = "#6c3fc0"   # 状态数值
FG_RED = "#c0392b"      # 录制/告警

NUM_FONT = ("Menlo", 13)
SMALL_FONT = ("Menlo", 11)
TITLE_FONT = ("Helvetica", 11, "bold")
SCOPE_POINTS = 300
REFRESH_MS = 100


# ================================================================== 曲线组件
class Scope(tk.Canvas):
    """轻量实时曲线 (Canvas 绘制, 无第三方依赖)"""

    def __init__(self, master, title, unit, series, width=380, height=140):
        super().__init__(master, width=width, height=height,
                         bg="#1e1e2e", highlightthickness=0)
        self.title, self.unit = title, unit
        self.series = list(series)  # [(显示名, 字段key, 颜色)]
        self.data = {key: [] for _, key, _ in self.series}
        self.w, self.h = width, height
        self.pad_l, self.pad_r, self.pad_t, self.pad_b = 46, 8, 18, 16

    def push(self, values):
        for _, key, _ in self.series:
            self.data[key].append(values.get(key, 0.0))

    def clear(self):
        for k in self.data:
            self.data[k].clear()
        self.redraw()

    def redraw(self):
        self.delete("all")
        w = self.winfo_width() or self.w
        h = self.winfo_height() or self.h
        x0, x1 = self.pad_l, w - self.pad_r
        y0, y1 = self.pad_t, h - self.pad_b
        self.create_text(x0, 10, text=self.title, anchor="w",
                         fill="#cdd6f4", font=("Helvetica", 10, "bold"))
        for i in range(1, 4):
            y = y0 + (y1 - y0) * i / 4
            self.create_line(x0, y, x1, y, fill="#313244")
        self.create_line(x0, y0, x0, y1, fill="#45475a")
        self.create_line(x0, y1, x1, y1, fill="#45475a")

        vals = [v for k in self.data for v in self.data[k]]
        if not vals:
            self.create_text((x0 + x1) / 2, (y0 + y1) / 2, text="等待数据…",
                             fill="#6c7086")
            return
        lo, hi = min(vals), max(vals)
        if hi - lo < 1e-9:
            lo, hi = lo - 1, hi + 1
        else:
            m = (hi - lo) * 0.12
            lo, hi = lo - m, hi + m
        self.create_text(x0 - 4, y0, text=f"{hi:.2f}", anchor="e",
                         fill="#9399b2", font=("Menlo", 8))
        self.create_text(x0 - 4, y1, text=f"{lo:.2f}", anchor="e",
                         fill="#9399b2", font=("Menlo", 8))
        self.create_text(x0 - 4, (y0 + y1) / 2, text=self.unit, anchor="e",
                         fill="#6c7086", font=("Menlo", 8))

        n = SCOPE_POINTS
        for _, key, color in self.series:
            pts = self.data[key][-n:]
            if len(pts) < 2:
                continue
            step = (x1 - x0) / (n - 1)
            coords = []
            for i, v in enumerate(pts):
                x = x0 + i * step
                y = y1 - (v - lo) / (hi - lo) * (y1 - y0)
                coords.extend((x, y))
            self.create_line(*coords, fill=color, width=1.4)


# ================================================================== SBUS 组件
class SbusBars(tk.Canvas):
    """10 通道遥控器数值条 (量程自动适配)"""

    def __init__(self, master, width=380, height=170):
        super().__init__(master, width=width, height=height,
                         bg="#1e1e2e", highlightthickness=0)
        self.w, self.h = width, height
        self.range = [[None, None] for _ in range(10)]  # 每通道 [lo, hi]
        self.values = [None] * 10

    def set(self, values):
        self.values = list(values)
        for i, v in enumerate(values):
            r = self.range[i]
            if r[0] is None:
                r[0] = r[1] = v
            else:
                r[0] = min(r[0], v)
                r[1] = max(r[1], v)

    def redraw(self):
        self.delete("all")
        w = self.winfo_width() or self.w
        h = self.winfo_height() or self.h
        for i in range(10):
            y = 6 + i * (h - 12) / 10
            bh = (h - 12) / 10 * 0.62
            self.create_text(4, y + bh / 2, text=f"CH{i+1}", anchor="w",
                             fill="#9399b2", font=("Menlo", 8))
            bx0, bx1 = 36, w - 58
            self.create_rectangle(bx0, y, bx1, y + bh, fill="#313244", outline="")
            v = self.values[i]
            if v is None:
                continue
            lo, hi = self.range[i]
            span = (hi - lo) or 1
            frac = (v - lo) / span
            mid = bx0 + (bx1 - bx0) * 0.5
            x = bx0 + (bx1 - bx0) * frac
            color = "#a6e3a1" if abs(frac - 0.5) < 0.08 else "#89b4fa"
            self.create_rectangle(mid, y + bh * 0.35, x, y + bh * 0.65,
                                  fill=color, outline="")
            self.create_text(bx1 + 4, y + bh / 2, text=f"{v:5d}", anchor="w",
                             fill="#cdd6f4", font=("Menlo", 8))

    def reset(self):
        self.range = [[None, None] for _ in range(10)]
        self.values = [None] * 10
        self.redraw()


# ================================================================== 主应用
class App:
    def __init__(self, root: tk.Tk):
        self.root = root
        self.root.title(APP_TITLE)
        self.root.geometry("1320x920")
        self.root.minsize(1160, 820)
        self.root.configure(bg=BG)

        # ---------------- 状态 ----------------
        self.ser = None
        self.reader_thread = None
        self.stop_event = threading.Event()
        self.parser = FrameParser()
        self.latest = {}                 # type -> 最新解码帧
        self.error_q = queue.Queue()
        self.recorder = Recorder()
        self.connected = False
        self.recording = False
        self.t_connect = None
        self._last_stat = (time.perf_counter(), 0, 0)  # (t, frames_ok, crc_err)
        self.rx_hz = 0.0

        self._build_ui()
        self.refresh_ports()
        self.root.after(REFRESH_MS, self._tick)
        self.root.protocol("WM_DELETE_WINDOW", self.on_close)
        # 启动自动连接 (默认开启)
        if self.var_autoconnect.get() and self.port_var.get():
            self.root.after(500, self.connect)

    # ------------------------------------------------------------ 控件助手
    def _frame(self, parent, text):
        f = tk.LabelFrame(parent, text=text, bg=BG_FRAME, fg=FG_TEXT,
                          font=TITLE_FONT, bd=1, relief=tk.GROOVE, padx=6, pady=6)
        return f

    def _label(self, parent, text, fg=FG_TEXT, font=None, bg=None, **kw):
        return tk.Label(parent, text=text, fg=fg, font=font,
                        bg=bg if bg is not None else BG_FRAME, **kw)

    def _value_label(self, parent, width=9, fg=FG_GREEN):
        return tk.Label(parent, text="--", width=width, font=NUM_FONT,
                        fg=fg, bg=BG_VALUE, relief=tk.SUNKEN, bd=1,
                        anchor="w", padx=4)

    def _button(self, parent, text, command, width=8, bg="#ffffff"):
        return tk.Button(parent, text=text, command=command, width=width,
                         bg=bg, fg=FG_TEXT, activebackground="#d5dbe5",
                         relief=tk.RAISED, bd=1, font=("Helvetica", 11))

    # ---------------------------------------------------------------- UI 构建
    def _build_ui(self):
        pad = dict(padx=8, pady=5)

        # ============ 连接区 ============
        f_conn = self._frame(self.root, "串口连接")
        f_conn.pack(fill="x", **pad)
        self._label(f_conn, "串口:").pack(side="left")
        self.port_var = tk.StringVar()
        self.port_menu = tk.OptionMenu(f_conn, self.port_var, "")
        self.port_menu.config(width=30, bg=BG_VALUE, fg=FG_TEXT,
                              activebackground="#d5dbe5", highlightthickness=0)
        self.port_menu.pack(side="left", padx=(0, 6))
        self._button(f_conn, "刷新", self.refresh_ports, width=5).pack(side="left")

        self._label(f_conn, "波特率:").pack(side="left", padx=(14, 0))
        self.baud_var = tk.StringVar(value="2000000")
        om_baud = tk.OptionMenu(f_conn, self.baud_var, *BAUDRATES)
        om_baud.config(width=9, bg=BG_VALUE, fg=FG_TEXT, highlightthickness=0)
        om_baud.pack(side="left")

        self._label(f_conn, "帧类型:").pack(side="left", padx=(14, 0))
        self.filter_var = tk.StringVar(value=FRAME_FILTERS[0][1])
        om_filter = tk.OptionMenu(f_conn, self.filter_var,
                                  *[d for _, d in FRAME_FILTERS])
        om_filter.config(width=19, bg=BG_VALUE, fg=FG_TEXT, highlightthickness=0)
        om_filter.pack(side="left")

        self.var_autoconnect = tk.BooleanVar(value=True)
        tk.Checkbutton(f_conn, text="启动自动连接", variable=self.var_autoconnect,
                       bg=BG_FRAME, fg=FG_TEXT, activebackground=BG_FRAME,
                       selectcolor=BG_VALUE).pack(side="left", padx=(14, 0))

        self.btn_connect = self._button(f_conn, "连接", self.connect, width=7,
                                        bg="#a6e3a1")
        self.btn_connect.pack(side="left", padx=(16, 4))
        self.btn_disconnect = self._button(f_conn, "断开", self.disconnect,
                                           width=7, bg="#f38ba8")
        self.btn_disconnect.pack(side="left")
        self.btn_disconnect.config(state="disabled")

        # ============ 状态栏 ============
        f_stat = tk.Frame(self.root, bg=BG)
        f_stat.pack(fill="x", **pad)
        self.led = tk.Label(f_stat, text="●", fg=FG_RED, bg=BG,
                            font=("Helvetica", 16))
        self.led.pack(side="left")
        self.lbl_conn = self._label(f_stat, "未连接", bg=BG, width=8)
        self.lbl_conn.pack(side="left")
        for text, var, w in (("RX 频率", "hz", 14), ("总帧数", "frames", 10),
                             ("变体帧", "variant", 8), ("CRC错误", "crc", 8),
                             ("帧长错误", "badlen", 8), ("丢弃字节", "drop", 9),
                             ("帧类型", "ftype", 14), ("运行时长", "elapsed", 9)):
            self._label(f_stat, text + ":", bg=BG).pack(side="left", padx=(12, 2))
            lbl = self._value_label(f_stat, width=w, fg=FG_BLUE)
            lbl.pack(side="left")
            setattr(self, f"lbl_{var}", lbl)

        # ============ 数据显示区 ============
        f_data = tk.Frame(self.root, bg=BG)
        f_data.pack(fill="both", expand=True, **pad)
        f_data.columnconfigure(0, weight=1)
        f_data.columnconfigure(1, weight=1)

        # ---- 左列: 里程计 + IMU ----
        f_left = tk.Frame(f_data, bg=BG)
        f_left.grid(row=0, column=0, sticky="nsew", padx=(0, 5))

        f_odo = self._frame(f_left, "里程计")
        f_odo.pack(fill="x", pady=(0, 6))
        self.odo_vars = {}
        for i, (name, key) in enumerate((("左轮速度", "lw"), ("右轮速度", "rw"),
                                         ("前进速度", "speed"), ("角速度", "av"))):
            self._label(f_odo, name + ":").grid(
                row=i // 2, column=(i % 2) * 2, sticky="e", padx=(6, 2), pady=4)
            lbl = self._value_label(f_odo, fg=FG_GREEN)
            lbl.grid(row=i // 2, column=(i % 2) * 2 + 1, sticky="w", padx=(0, 8))
            self.odo_vars[key] = lbl

        f_imu = self._frame(f_left, "IMU 姿态")
        f_imu.pack(fill="x")
        self.imu_vars = {}
        rows = (("加速度 ax/ay/az", "acc", "m/s²"), ("陀螺仪 gx/gy/gz", "gyro", "rad/s"),
                ("四元数 q0/q1/q2/q3", "quat", ""), ("欧拉角 Roll/Pitch/Yaw", "euler", "度"))
        for r, (name, key, unit) in enumerate(rows):
            self._label(f_imu, name + ":").grid(
                row=r, column=0, sticky="e", padx=(6, 2), pady=4)
            lbl = self._value_label(f_imu, width=30, fg=FG_AMBER)
            lbl.grid(row=r, column=1, sticky="w", padx=(0, 6))
            self.imu_vars[key] = lbl
            if unit:
                self._label(f_imu, unit, fg=FG_DIM).grid(
                    row=r, column=2, sticky="w")

        # ---- 右列: 电机表 + SBUS + 状态 ----
        f_right = tk.Frame(f_data, bg=BG)
        f_right.grid(row=0, column=1, sticky="nsew", padx=(5, 0))

        f_motor = self._frame(f_right, "电机反馈 (6 轴 MIT)")
        f_motor.pack(fill="x", pady=(0, 6))
        heads = ("电机", "位置(°)", "速度(°/s)", "力矩(N·m)", "温度(°C)")
        widths = (8, 11, 11, 11, 10)
        for c, (h, w) in enumerate(zip(heads, widths)):
            self._label(f_motor, h, font=("Helvetica", 10, "bold")).grid(
                row=0, column=c, padx=4, pady=(2, 4))
        self.motor_cells = []  # [(pos,vel,torque,temp), ...] label 列表
        for i, name in enumerate(MOTOR_NAMES):
            self._label(f_motor, name).grid(
                row=i + 1, column=0, padx=4, pady=2)
            cells = []
            for c in range(1, 5):
                lbl = self._value_label(f_motor, width=widths[c], fg=FG_GREEN)
                lbl.grid(row=i + 1, column=c, padx=3, pady=2)
                cells.append(lbl)
            self.motor_cells.append(cells)

        f_sb = self._frame(f_right, "遥控器 SBUS 通道")
        f_sb.pack(fill="x", pady=(0, 6))
        self.sbus_canvas = SbusBars(f_sb)
        self.sbus_canvas.pack(fill="x", padx=4, pady=4)

        f_rst = self._frame(f_right, "机器人状态")
        f_rst.pack(fill="x")
        self.stat_vars = {}
        for i, (name, key) in enumerate((("开关机", "power"), ("工作模式", "model"),
                                         ("帧计数 count", "count"))):
            self._label(f_rst, name + ":").grid(
                row=0, column=i * 2, sticky="e", padx=(6, 2), pady=4)
            lbl = self._value_label(f_rst, width=11, fg=FG_PURPLE)
            lbl.grid(row=0, column=i * 2 + 1, sticky="w", padx=(0, 10))
            self.stat_vars[key] = lbl

        # ============ 曲线区 ============
        f_scope = self._frame(self.root, "实时曲线")
        f_scope.pack(fill="x", **pad)
        f_scope.columnconfigure(0, weight=1)
        f_scope.columnconfigure(1, weight=1)
        f_scope.columnconfigure(2, weight=1)
        self.scopes = [
            Scope(f_scope, "轮式速度", "m/s",
                  (("左轮", "lw_speed", "#f38ba8"), ("右轮", "rw_speed", "#fab387"),
                   ("前进", "speed", "#a6e3a1"))),
            Scope(f_scope, "姿态角", "deg",
                  (("Roll", "roll", "#89b4fa"), ("Pitch", "pitch", "#f9e2af"))),
            Scope(f_scope, "关节位置", "deg",
                  (("左前", "m0_pos", "#cba6f7"), ("右前", "m2_pos", "#f5c2e7"),
                   ("左轮", "m4_pos", "#94e2d5"))),
        ]
        for i, sc in enumerate(self.scopes):
            sc.grid(row=0, column=i, sticky="nsew", padx=4, pady=4)

        # ============ 录制区 ============
        f_rec = self._frame(self.root, "全量录制")
        f_rec.pack(fill="x", **pad)
        self._label(f_rec, "名称:").pack(side="left")
        self.rec_name = tk.StringVar()
        tk.Entry(f_rec, textvariable=self.rec_name, width=24, bg=BG_VALUE,
                 fg=FG_TEXT, relief=tk.SUNKEN, bd=1).pack(side="left", padx=(0, 4))
        self._label(f_rec, "(留空则按时间戳自动命名)", fg=FG_DIM).pack(side="left")
        self._label(f_rec, "保存目录:").pack(side="left", padx=(16, 0))
        self.rec_dir = tk.StringVar(value=DEFAULT_LOG_DIR)
        tk.Entry(f_rec, textvariable=self.rec_dir, width=40, bg=BG_VALUE,
                 fg=FG_TEXT, relief=tk.SUNKEN, bd=1).pack(side="left")
        self._button(f_rec, "浏览…", self.choose_dir, width=6).pack(
            side="left", padx=4)
        self.var_csv = tk.BooleanVar(value=True)
        self.var_bin = tk.BooleanVar(value=True)
        for text, var in (("解码CSV", self.var_csv), ("原始bin", self.var_bin)):
            tk.Checkbutton(f_rec, text=text, variable=var, bg=BG_FRAME,
                           fg=FG_TEXT, activebackground=BG_FRAME,
                           selectcolor=BG_VALUE).pack(side="left", padx=(12, 0))

        # ============ 底部按钮 ============
        f_btn = tk.Frame(self.root, bg=BG)
        f_btn.pack(fill="x", **pad)
        self.btn_rec_start = self._button(f_btn, "● 开始记录", self.start_recording,
                                          width=15, bg="#a6e3a1")
        self.btn_rec_start.config(font=("Helvetica", 12, "bold"))
        self.btn_rec_start.pack(side="left", padx=(0, 12))
        self.btn_rec_stop = self._button(f_btn, "■ 结束记录", self.stop_recording,
                                         width=15, bg="#f38ba8")
        self.btn_rec_stop.config(font=("Helvetica", 12, "bold"), state="disabled")
        self.btn_rec_stop.pack(side="left")
        self.lbl_rec = self._label(f_btn, "未录制", fg=FG_DIM, bg=BG,
                                   font=SMALL_FONT)
        self.lbl_rec.pack(side="left", padx=20)

    # ---------------------------------------------------------------- 端口操作
    def refresh_ports(self):
        ports = list_ports()
        menu = self.port_menu["menu"]
        menu.delete(0, "end")
        for p in ports:
            menu.add_command(label=p,
                             command=lambda v=p: self.port_var.set(v))
        if ports:
            cur = self.port_var.get()
            if cur not in ports:
                # 优先选中 usbmodem (ESP32 原生 USB 串口)
                for p in ports:
                    if "usbmodem" in p:
                        self.port_var.set(p)
                        return
                self.port_var.set(ports[0])
        else:
            self.port_var.set("")

    def choose_dir(self):
        d = filedialog.askdirectory(initialdir=self.rec_dir.get() or SCRIPT_DIR)
        if d:
            self.rec_dir.set(d)

    # ---------------------------------------------------------------- 连接管理
    def _frame_lengths(self):
        idx = [d for _, d in FRAME_FILTERS].index(self.filter_var.get())
        key = FRAME_FILTERS[idx][0]
        if key == "full":
            return (FRAME_FULL_LEN,)
        if key == "lite":
            return (FRAME_LITE_LEN,)
        return (FRAME_FULL_LEN, FRAME_LITE_LEN)

    def connect(self):
        if self.connected:
            return
        port = self.port_var.get()
        if not port:
            messagebox.showwarning("提示", "未选择串口, 请点击「刷新」后选择")
            return
        try:
            baud = int(self.baud_var.get())
        except ValueError:
            messagebox.showwarning("提示", "波特率格式错误")
            return
        try:
            self.ser = SerialPort(port, baud).open()
        except SerialPortError as e:
            messagebox.showerror("打开失败", str(e))
            self.ser = None
            return

        self.parser = FrameParser(self._frame_lengths())
        self.latest.clear()
        for sc in self.scopes:
            sc.clear()
        self.sbus_canvas.reset()
        self._reset_display()
        self._last_stat = (time.perf_counter(), 0, 0)
        self.t_connect = time.perf_counter()
        self.stop_event.clear()
        self.reader_thread = threading.Thread(target=self._reader_loop, daemon=True)
        self.reader_thread.start()
        self.connected = True
        self.btn_connect.config(state="disabled")
        self.btn_disconnect.config(state="normal")
        self.led.config(fg="#1a7f37")
        self.lbl_conn.config(text=f"已连接 {port}")

    def disconnect(self):
        if not self.connected:
            return
        # 录制中先自动结束录制 (保留数据)
        if self.recorder.recording:
            self.stop_recording(notify=False)
        self.stop_event.set()
        if self.reader_thread:
            self.reader_thread.join(timeout=1.5)
        if self.ser:
            self.ser.close()
            self.ser = None
        self.connected = False
        self.btn_connect.config(state="normal")
        self.btn_disconnect.config(state="disabled")
        self.led.config(fg=FG_RED)
        self.lbl_conn.config(text="未连接")
        self.rx_hz = 0.0

    def _reset_display(self):
        for lbl in self.odo_vars.values():
            lbl.config(text="--")
        for lbl in self.imu_vars.values():
            lbl.config(text="--")
        for cells in self.motor_cells:
            for lbl in cells:
                lbl.config(text="--")
        for lbl in self.stat_vars.values():
            lbl.config(text="--")
        self.lbl_frames.config(text="0")
        self.lbl_variant.config(text="0")
        self.lbl_crc.config(text="0")
        self.lbl_badlen.config(text="0")
        self.lbl_drop.config(text="0")
        self.lbl_ftype.config(text="--")
        self.lbl_hz.config(text="0.0 Hz")

    # ---------------------------------------------------------------- 读取线程
    def _reader_loop(self):
        ser = self.ser
        while not self.stop_event.is_set():
            try:
                n = ser.in_waiting()
                if n <= 0:
                    time.sleep(0.0005)
                    continue
                chunk = ser.read(n)
                if not chunk:
                    continue
                self.recorder.write_raw(chunk)
                for _length, frame, ok in self.parser.feed(chunk):
                    if not ok:
                        self.recorder.note_crc_error()
                        continue
                    d = decode_frame(frame)
                    if d is None:
                        continue
                    self.latest[d["type"]] = d
                    for sc in self.scopes:
                        sc.push(self._scope_values(d))
                    self.recorder.write_frame(d)
            except SerialPortError as e:
                self.error_q.put(str(e))
                return
            except Exception as e:  # 防御: 读取线程不能崩溃
                self.error_q.put(f"读取线程异常: {e}")
                return

    @staticmethod
    def _scope_values(d):
        motors = d.get("motors") or [{}] * 6
        euler = d.get("euler_deg") or (None, None, None)
        return {
            "lw_speed": d.get("lw_speed") or 0.0,
            "rw_speed": d.get("rw_speed") or 0.0,
            "speed": d.get("speed") or 0.0,
            "roll": euler[0] or 0.0,
            "pitch": euler[1] or 0.0,
            "m0_pos": (motors[0].get("position") or 0.0),
            "m2_pos": (motors[2].get("position") or 0.0),
            "m4_pos": (motors[4].get("position") or 0.0),
        }

    # ---------------------------------------------------------------- 录制
    def start_recording(self):
        if self.recorder.recording:
            return
        name = self.rec_name.get()
        if name and name != sanitize_name(name):
            if not messagebox.askyesno(
                    "名称确认",
                    f"名称含非法字符, 将保存为:\n{sanitize_name(name)}\n\n继续?"):
                return
        save_dir = self.rec_dir.get() or DEFAULT_LOG_DIR
        baud = int(self.baud_var.get()) if self.baud_var.get().isdigit() else 0
        try:
            info = self.recorder.start(
                name=name, save_dir=save_dir,
                port=self.port_var.get() if self.connected else "",
                baudrate=baud,
                save_csv=self.var_csv.get(), save_bin=self.var_bin.get())
        except RecorderError as e:
            messagebox.showerror("无法开始录制", str(e))
            return
        self.recording = True
        self.btn_rec_start.config(state="disabled")
        self.btn_rec_stop.config(state="normal")
        self.lbl_rec.config(text=f"● 录制中: {info['dir']}", fg=FG_RED)

    def stop_recording(self, notify=True):
        if not self.recorder.recording:
            return
        summary = self.recorder.stop()
        self.recording = False
        self.btn_rec_start.config(state="normal")
        self.btn_rec_stop.config(state="disabled")
        self.lbl_rec.config(text="未录制", fg=FG_DIM)
        if notify and summary:
            messagebox.showinfo(
                "录制完成",
                f"名称: {summary['name']}\n"
                f"目录: {summary['dir']}\n"
                f"时长: {summary['duration_s']:.1f} s\n"
                f"全量帧: {summary['frames_full']}  精简帧: {summary['frames_lite']}\n"
                f"原始字节: {summary['raw_bytes']}")

    # ---------------------------------------------------------------- 界面刷新
    @staticmethod
    def _fmt(v, spec=".3f", dash="--"):
        return dash if v is None else format(v, spec)

    def _tick(self):
        try:
            # 读取线程错误
            try:
                while True:
                    msg = self.error_q.get_nowait()
                    messagebox.showerror("串口错误", msg)
                    self.disconnect()
            except queue.Empty:
                pass

            # ---- 统计 ----
            now = time.perf_counter()
            t0, f0, c0 = self._last_stat
            dt = now - t0
            if dt >= 0.5:
                f1, c1 = self.parser.frames_ok, self.parser.frames_crc_err
                self.rx_hz = (f1 - f0) / dt
                self._last_stat = (now, f1, c1)
            self.lbl_hz.config(text=f"{self.rx_hz:.1f} Hz")
            self.lbl_frames.config(text=str(self.parser.frames_ok))
            self.lbl_variant.config(text=str(self.parser.frames_variant))
            self.lbl_crc.config(text=str(self.parser.frames_crc_err))
            self.lbl_badlen.config(text=str(self.parser.frames_bad_len))
            self.lbl_drop.config(text=str(self.parser.bytes_dropped))
            types = {t for t, v in self.latest.items() if v is not None}
            if not types:
                self.lbl_ftype.config(text="--")
            elif types == {"full"}:
                self.lbl_ftype.config(text="全量 182B")
            elif types == {"lite"}:
                self.lbl_ftype.config(text="精简 124B")
            else:
                self.lbl_ftype.config(text="混合")
            if self.connected and self.t_connect:
                el = int(now - self.t_connect)
                self.lbl_elapsed.config(text=f"{el // 60:02d}:{el % 60:02d}")

            # ---- 数据面板 ----
            d = self.latest.get("full") or self.latest.get("lite")
            if d:
                for key, v in (("lw", d.get("lw_speed")), ("rw", d.get("rw_speed")),
                               ("speed", d.get("speed")), ("av", d.get("angular_vel"))):
                    self.odo_vars[key].config(text=self._fmt(v))
                acc, gyro = d.get("acc"), d.get("gyro")
                quat, euler = d.get("quat"), d.get("euler_deg")
                self.imu_vars["acc"].config(
                    text="  ".join(self._fmt(v, ".2f") for v in acc) if acc else "--")
                self.imu_vars["gyro"].config(
                    text="  ".join(self._fmt(v, ".3f") for v in gyro) if gyro else "--")
                self.imu_vars["quat"].config(
                    text="  ".join(self._fmt(v, ".4f") for v in quat) if quat else "--")
                self.imu_vars["euler"].config(
                    text="  ".join(self._fmt(v, ".2f") for v in euler) if euler else "--")
                motors = d.get("motors")
                if motors:
                    for i, cells in enumerate(self.motor_cells):
                        m = motors[i]
                        cells[0].config(text=self._fmt(m.get("position")))
                        cells[1].config(text=self._fmt(m.get("velocity")))
                        cells[2].config(text=self._fmt(m.get("torque")))
                        cells[3].config(text=self._fmt(m.get("temp"), ".1f"))
                if d.get("sbus"):
                    self.sbus_canvas.set(d["sbus"])
                sw, md = d.get("switched_on"), d.get("model")
                self.stat_vars["power"].config(
                    text=("开机" if sw == 1 else "关机") if sw is not None else "--",
                    fg=FG_GREEN if sw == 1 else FG_RED)
                self.stat_vars["model"].config(
                    text=("电机控制" if md == 1 else "底盘控制") if md is not None else "--")
                self.stat_vars["count"].config(text=str(d.get("count", "--")))

            for sc in self.scopes:
                sc.redraw()
            self.sbus_canvas.redraw()

            # ---- 录制状态 ----
            if self.recorder.recording:
                info = self.recorder.info()
                if info:
                    self.lbl_rec.config(
                        text=f"● 录制中 [{info['name']}]  {info['duration']:.0f}s  "
                             f"CSV {info['rows']}行  原始 {info['raw_bytes'] / 1024:.0f}KB",
                        fg=FG_RED)
        finally:
            self.root.after(REFRESH_MS, self._tick)

    # ---------------------------------------------------------------- 退出
    def on_close(self):
        if self.recorder.recording:
            if not messagebox.askyesno("退出", "正在录制, 确定退出并保存数据吗?"):
                return
            self.stop_recording(notify=False)
        self.disconnect()
        self.root.destroy()


def main():
    root = tk.Tk()
    App(root)
    root.mainloop()


if __name__ == "__main__":
    main()
