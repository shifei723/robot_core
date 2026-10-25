# -*- coding: utf-8 -*-
"""
数据录制模块 (全量录制)
========================

每次录制生成一个以「名称」命名的文件夹, 包含:

    <save_dir>/<名称>/
        ├── data.csv      解码后的全量数据 (含时间戳, 两种帧的所有字段)
        ├── raw.bin       串口原始字节流 (完整留存, 可随时重新解码/回放)
        └── meta.json     录制信息 (端口/波特率/帧统计/字段说明)

命名规则:
    - 名称为空时自动使用 rec_年月日_时分秒
    - 名称重复时自动追加 _1, _2 ...
    - 非法路径字符 (/\:* 等) 会被替换为下划线

线程安全: write_raw / write_frame 由串口读取线程调用,
          内部加锁并定期 flush, 掉电/崩溃最多丢失 1 秒数据。
"""

import csv
import json
import os
import re
import threading
import time

# CSV 列定义 (全量帧 + 精简帧的超集; 精简帧缺失字段留空)
CSV_FIELDS = (
    ["t_s", "wall_time", "frame_type", "count"]
    + ["lw_speed", "rw_speed", "speed", "angular_vel"]
    + ["acc_x", "acc_y", "acc_z", "gyro_x", "gyro_y", "gyro_z",
       "q0", "q1", "q2", "q3", "roll_deg", "pitch_deg", "yaw_deg"]
    + [f"m{i}_{k}" for i in range(6)
       for k in ("pos", "vel", "torque", "temp")]
    + [f"sbus_{i}" for i in range(10)]
    + ["status_code", "switched_on", "model"]
)

CSV_FIELD_DESC = {
    "t_s": "录制起始时刻起的秒数 (主机单调时钟)",
    "wall_time": "墙钟时间 ISO8601",
    "frame_type": "full=ROS_body_t(182B) / lite=ROS_body_t1(124B)",
    "count": "固件帧计数 (0~255 循环)",
    "lw_speed": "左轮速度 m/s", "rw_speed": "右轮速度 m/s",
    "speed": "前进速度 m/s", "angular_vel": "角速度 rad/s",
    "acc_x/y/z": "加速度计 m/s²", "gyro_x/y/z": "陀螺仪 rad/s",
    "q0..q3": "姿态四元数", "roll/pitch/yaw_deg": "由四元数解算的欧拉角 (度)",
    "m{i}_pos": "电机i位置 (度)", "m{i}_vel": "电机i速度 (度/s)",
    "m{i}_torque": "电机i力矩 (N·m)", "m{i}_temp": "电机i温度 (°C)",
    "sbus_i": "遥控器第i通道原始值",
    "status_code": "状态码 (bit0=开关机 bit1=工作模式)",
    "switched_on": "1=开机 0=关机", "model": "1=电机控制模式 0=底盘控制模式",
}

_ILLEGAL = re.compile(r'[/\\:*?"<>|\x00-\x1f]')


class RecorderError(Exception):
    pass


def sanitize_name(name: str) -> str:
    name = _ILLEGAL.sub("_", name).strip().strip(".")
    return name[:64] if name else ""


_BASELINE_KEYS = ("frames_variant", "frames_headless", "bytes_dropped")


def _baseline_of(parser):
    """记录 parser 此刻的累计统计, 供 stop() 计算录制区间内的增量"""
    if parser is None:
        return {}
    return {k: int(getattr(parser, k, 0)) for k in _BASELINE_KEYS}


def _link_health(parser, n_rows, raw_bytes, duration, baseline=None):
    """汇总链路健康指标 —— 让「解析器吞帧」这类问题不再隐形

    历史教训: meta 只记了 crc_errors, 而 FrameParser 当年把 15% 的完好帧
    当作同步垃圾丢掉时 crc_errors 仍然是 0, 问题被完全掩盖, 一度误判成
    「下位机丢帧」。丢弃字节数是唯一能暴露它的指标, 必须入库。
    """
    health = {}
    for k in _BASELINE_KEYS:
        v = int(getattr(parser, k, 0)) if parser is not None else 0
        health[k] = max(0, v - (baseline or {}).get(k, 0))
    health["bytes_dropped_pct"] = round(
        health["bytes_dropped"] / (raw_bytes or 1) * 100, 4)
    health["frame_hz"] = round(n_rows / duration, 1) if duration > 0 else 0.0
    return health


class Recorder:
    """全量录制器: CSV + 原始二进制 + 元数据"""

    def __init__(self):
        self._lock = threading.Lock()
        self.recording = False
        self.dir = None
        self.name = None
        self.csv_path = None
        self.bin_path = None
        self.meta_path = None
        self._csv_file = None
        self._csv_writer = None
        self._bin_file = None
        self._last_flush = 0.0
        # 统计
        self.frames_full = 0
        self.frames_lite = 0
        self.crc_errors = 0
        self.raw_bytes = 0
        self.csv_rows = 0
        self.t0 = None       # time.perf_counter
        self.t0_wall = None  # time.time
        self.port = ""
        self.baudrate = 0
        self.save_csv = True
        self.save_bin = True
        self._baseline = {}          # 录制起始时的 parser 统计快照

    # ------------------------------------------------------------------ 开始
    def start(self, name, save_dir, port="", baudrate=0,
              save_csv=True, save_bin=True, parser=None):
        with self._lock:
            if self.recording:
                raise RecorderError("已在录制中")
            if not (save_csv or save_bin):
                raise RecorderError("至少选择一种录制内容 (CSV 或 原始二进制)")

            name = sanitize_name(name) or time.strftime("rec_%Y%m%d_%H%M%S")
            save_dir = os.path.abspath(os.path.expanduser(save_dir))
            base = name
            n = 0
            while os.path.exists(os.path.join(save_dir, name)):
                n += 1
                name = f"{base}_{n}"
            self.dir = os.path.join(save_dir, name)
            try:
                os.makedirs(self.dir, exist_ok=True)
            except OSError as e:
                raise RecorderError(f"无法创建录制目录: {e}")

            self.name = name
            self.csv_path = os.path.join(self.dir, "data.csv")
            self.bin_path = os.path.join(self.dir, "raw.bin")
            self.meta_path = os.path.join(self.dir, "meta.json")
            self.port, self.baudrate = port, baudrate
            self.save_csv, self.save_bin = save_csv, save_bin

            self._csv_file = self._bin_file = None
            self._csv_writer = None
            try:
                if save_csv:
                    self._csv_file = open(self.csv_path, "w", newline="", buffering=1)
                    self._csv_writer = csv.writer(self._csv_file)
                    self._csv_writer.writerow(CSV_FIELDS)
                if save_bin:
                    self._bin_file = open(self.bin_path, "wb", buffering=1024 * 64)
            except OSError as e:
                self._close_files()
                raise RecorderError(f"无法创建录制文件: {e}")

            self.frames_full = self.frames_lite = self.crc_errors = 0
            self.raw_bytes = self.csv_rows = 0
            self._baseline = _baseline_of(parser)
            self.t0 = time.perf_counter()
            self.t0_wall = time.time()
            self._last_flush = self.t0
            self.recording = True
            return {"name": name, "dir": self.dir,
                    "csv": self.csv_path if save_csv else None,
                    "bin": self.bin_path if save_bin else None}

    # ------------------------------------------------------------------ 写入
    def write_raw(self, data: bytes):
        """写入串口原始字节 (读取线程调用)"""
        if not self.recording or not self.save_bin or self._bin_file is None:
            return
        with self._lock:
            self._bin_file.write(data)
            self.raw_bytes += len(data)
            self._maybe_flush()

    def write_frame(self, d: dict):
        """写入一帧解码数据 (读取线程调用)"""
        if not self.recording or not self.save_csv or self._csv_writer is None:
            return
        with self._lock:
            row = self._row_of(d, t_s=d.get("host_t", self.t0_wall) - self.t0_wall)
            self._csv_writer.writerow(row)
            self.csv_rows += 1
            if d.get("type") == "full":
                self.frames_full += 1
            elif d.get("type") == "lite":
                self.frames_lite += 1
            self._maybe_flush()

    def note_crc_error(self):
        with self._lock:
            self.crc_errors += 1

    def _maybe_flush(self):
        now = time.perf_counter()
        if now - self._last_flush >= 1.0:
            self._last_flush = now
            if self._csv_file:
                self._csv_file.flush()
            if self._bin_file:
                self._bin_file.flush()

    @staticmethod
    def _row_of(d: dict, t_s: float):
        t = d.get("host_t", 0.0)
        row = {
            "t_s": f"{t_s:.3f}",
            "wall_time": time.strftime("%Y-%m-%dT%H:%M:%S", time.localtime(t))
            + f".{int((t % 1) * 1000):03d}",
            "frame_type": d.get("type", ""), "count": d.get("count", ""),
            "lw_speed": d.get("lw_speed"), "rw_speed": d.get("rw_speed"),
            "speed": d.get("speed"), "angular_vel": d.get("angular_vel"),
            "status_code": d.get("status_code"),
            "switched_on": d.get("switched_on"), "model": d.get("model"),
        }
        acc = d.get("acc") or (None, None, None)
        gyro = d.get("gyro") or (None, None, None)
        quat = d.get("quat") or (None, None, None, None)
        euler = d.get("euler_deg") or (None, None, None)
        row.update({"acc_x": acc[0], "acc_y": acc[1], "acc_z": acc[2],
                    "gyro_x": gyro[0], "gyro_y": gyro[1], "gyro_z": gyro[2],
                    "q0": quat[0], "q1": quat[1], "q2": quat[2], "q3": quat[3],
                    "roll_deg": euler[0], "pitch_deg": euler[1], "yaw_deg": euler[2]})
        motors = d.get("motors") or [{} for _ in range(6)]
        for i in range(6):
            m = motors[i] if i < len(motors) else {}
            row[f"m{i}_pos"] = m.get("position")
            row[f"m{i}_vel"] = m.get("velocity")
            row[f"m{i}_torque"] = m.get("torque")
            row[f"m{i}_temp"] = m.get("temp")
        sbus = d.get("sbus") or [None] * 10
        for i in range(10):
            row[f"sbus_{i}"] = sbus[i]
        return ["" if row.get(k) is None else row.get(k) for k in CSV_FIELDS]

    # ------------------------------------------------------------------ 结束
    def stop(self, parser=None):
        """结束录制并写入 meta

        parser: 可选 FrameParser, 用于把链路健康指标一并入库 (见 _link_health)
        """
        with self._lock:
            if not self.recording:
                return None
            self.recording = False
            duration = time.perf_counter() - self.t0
            self._close_files()
            health = _link_health(parser, self.csv_rows,
                                  self.raw_bytes, duration, self._baseline)
            summary = {
                "name": self.name, "dir": self.dir,
                "csv": self.csv_path if self.save_csv else None,
                "bin": self.bin_path if self.save_bin else None,
                "duration_s": round(duration, 3),
                "frames_full": self.frames_full,
                "frames_lite": self.frames_lite,
                "crc_errors": self.crc_errors,
                "raw_bytes": self.raw_bytes,
                "csv_rows": self.csv_rows,
                **health,
            }
            self._write_meta(duration, health)
            return summary

    def _close_files(self):
        for f in (self._csv_file, self._bin_file):
            try:
                if f:
                    f.flush()
                    f.close()
            except Exception:
                pass
        self._csv_file = self._bin_file = self._csv_writer = None

    def _write_meta(self, duration, health=None):
        meta = {
            "name": self.name,
            "port": self.port,
            "baudrate": self.baudrate,
            "frame_filter": "auto (ROS_body_t 182B + ROS_body_t1 124B)",
            "start_wall": time.strftime("%Y-%m-%dT%H:%M:%S", time.localtime(self.t0_wall)),
            "end_wall": time.strftime("%Y-%m-%dT%H:%M:%S"),
            "duration_s": round(duration, 3),
            "frames_full": self.frames_full,
            "frames_lite": self.frames_lite,
            "crc_errors": self.crc_errors,
            "raw_bytes": self.raw_bytes,
            "csv_rows": self.csv_rows,
            **(health or {}),
            "files": {
                "csv": os.path.basename(self.csv_path) if self.save_csv else None,
                "raw": os.path.basename(self.bin_path) if self.save_bin else None,
            },
            "csv_fields": list(CSV_FIELDS),
            "csv_field_desc": CSV_FIELD_DESC,
        }
        try:
            with open(self.meta_path, "w") as f:
                json.dump(meta, f, ensure_ascii=False, indent=2)
        except OSError:
            pass

    # ------------------------------------------------------------------ 状态
    def info(self):
        if not self.recording:
            return None
        return {
            "name": self.name, "dir": self.dir,
            "duration": time.perf_counter() - self.t0,
            "rows": self.csv_rows, "raw_bytes": self.raw_bytes,
            "frames_full": self.frames_full, "frames_lite": self.frames_lite,
        }
