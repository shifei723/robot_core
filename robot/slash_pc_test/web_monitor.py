#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
轮式腿机器人 Mac 上位机 (Web 可视化版)
======================================

在浏览器中实时显示串口回传的机器人状态, 并提供「开始记录 / 结束记录」按钮
(全量录制: 解码 CSV + 串口原始二进制 + 元数据 JSON), 支持自定义命名。

    运行:  python3 web_monitor.py            # 自动打开浏览器
           python3 web_monitor.py --port 9000 --no-browser

仅依赖 Python 标准库 (无需 pip 安装), 不需要 Tk/Qt。

HTTP 接口:
    GET  /                       页面
    GET  /api/ports              串口列表
    POST /api/connect            {port, baudrate}
    POST /api/disconnect         -
    GET  /api/state?since=<seq>  连接状态 + 统计 + 最新帧 + 新增曲线采样
    POST /api/record/start       {name, dir, csv, bin}
    POST /api/record/stop        -
    POST /api/open_dir           {dir}   在 Finder 中打开目录
"""

import argparse
import collections
import json
import os
import signal
import subprocess
import sys
import threading
import time
import webbrowser
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
if SCRIPT_DIR not in sys.path:
    sys.path.insert(0, SCRIPT_DIR)

from mac_serial import SerialPort, SerialPortError, list_ports   # noqa: E402
from recorder import Recorder, RecorderError, sanitize_name      # noqa: E402
from robot_protocol import (FRAME_FULL_LEN, FRAME_LITE_LEN,      # noqa: E402
                            MOTOR_NAMES, FrameParser, decode_frame)

HTML_PATH = os.path.join(SCRIPT_DIR, "web", "index.html")
DEFAULT_LOG_DIR = os.path.join(SCRIPT_DIR, "logs")
DEFAULT_BAUD = 2000000
SAMPLE_BUFFER = 900          # 曲线环形缓冲点数 (~10s @90Hz)

# 曲线通道定义 (顺序即前端通道 key)
CHANNELS = (["lw", "rw", "spd", "av"]
            + ["roll", "pitch", "yaw"]
            + [f"p{i}" for i in range(6)]
            + [f"q{i}" for i in range(6)])


def _f(v, default=0.0):
    """浮点安全转换 (None/非数 -> default)"""
    try:
        v = float(v)
        return v if v == v and abs(v) != float("inf") else default
    except (TypeError, ValueError):
        return default


# ================================================================== 串口链路
class RobotLink:
    """串口读取 + 解码 + 曲线采样 + 录制 (线程安全)"""

    def __init__(self):
        self.lock = threading.RLock()
        self.ser = None
        self.thread = None
        self.stop_event = threading.Event()
        self.parser = FrameParser()
        self.recorder = Recorder()
        self.latest = None            # 最新解码帧 (dict)
        self.samples = collections.deque(maxlen=SAMPLE_BUFFER)
        self.seq = 0
        self.connected = False
        self.port = ""
        self.baudrate = DEFAULT_BAUD
        self.t_connect = None
        self.frame_types = set()
        self.error = None             # 最近一次错误文本
        self._hz = 0.0
        self._hz_mark = (time.perf_counter(), 0)

    # ---------------------------------------------------------------- 连接
    def connect(self, port, baudrate=DEFAULT_BAUD):
        with self.lock:
            if self.connected:
                self._close_locked()
            if not port:
                return {"ok": False, "msg": "未指定串口"}
            try:
                ser = SerialPort(port, baudrate).open()
            except SerialPortError as e:
                self.error = str(e)
                return {"ok": False, "msg": f"打开串口失败: {e}"}
            self.ser = ser
            self.port = port
            self.baudrate = baudrate
            self.parser = FrameParser((FRAME_FULL_LEN, FRAME_LITE_LEN))
            self.latest = None
            self.samples.clear()
            self.seq = 0
            self.frame_types.clear()
            self.error = None
            self.t_connect = time.perf_counter()
            self._hz_mark = (self.t_connect, 0)
            self.stop_event.clear()
            self.thread = threading.Thread(target=self._reader_loop, daemon=True)
            self.thread.start()
            self.connected = True
            return {"ok": True, "port": port, "baudrate": baudrate}

    def disconnect(self):
        with self.lock:
            if not self.connected:
                return {"ok": True}
            self._close_locked()
            return {"ok": True}

    def _close_locked(self):
        if self.recorder.recording:
            self.recorder.stop(self.parser)
        self.stop_event.set()
        if self.thread and self.thread.is_alive():
            self.thread.join(timeout=1.5)
        self.thread = None
        if self.ser:
            try:
                self.ser.close()
            except Exception:
                pass
        self.ser = None
        self.connected = False
        self._hz = 0.0

    def auto_connect(self):
        """启动时自动连接第一个可用串口"""
        ports = list_ports()
        if not ports:
            return None
        prefer = [p for p in ports if "usbmodem" in p or "usbserial" in p]
        port = prefer[0] if prefer else ports[0]
        return self.connect(port, DEFAULT_BAUD)

    # ---------------------------------------------------------------- 读取线程
    def _reader_loop(self):
        ser = self.ser
        while not self.stop_event.is_set():
            try:
                # 实测 (2026-10-25, macOS, 2 Mbps): in_waiting+sleep(0.4ms) 与
                # select+短自旋的 read_burst 收帧率无统计差别, 但后者 CPU 高 6 倍,
                # 故用前者 (旧解析器时代的 85.5Hz/12.9% vs 85.8Hz/12.4% 已不适用,
                # 锚点扫描式 FrameParser 修复后实测 99.4 Hz / 零丢帧)。
                # 注意: 真正会毁掉数据的是"同一串口被两个进程同时读取"
                # (macOS cu.* 允许多开), 已由 open() 的 TIOCEXCL 独占打开防护。
                n = ser.in_waiting()
                if n <= 0:
                    time.sleep(0.0004)
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
                    with self.lock:
                        self.latest = d
                        self.frame_types.add(d.get("type"))
                        self.seq += 1
                        self.samples.append(self._make_sample(d, self.seq))
                    self.recorder.write_frame(d)
            except SerialPortError as e:
                self.error = str(e)
                return
            except Exception as e:          # 防御: 读取线程不能崩
                self.error = f"读取线程异常: {e}"
                return

    @staticmethod
    def _make_sample(d, seq):
        m = d.get("motors") or [{}] * 6
        e = d.get("euler_deg") or (0.0, 0.0, 0.0)
        s = {
            "seq": seq,
            "t": round(d.get("host_t", time.time()), 4),
            "lw": round(_f(d.get("lw_speed")), 4),
            "rw": round(_f(d.get("rw_speed")), 4),
            "spd": round(_f(d.get("speed")), 4),
            "av": round(_f(d.get("angular_vel")), 4),
            "roll": round(_f(e[0]), 3),
            "pitch": round(_f(e[1]), 3),
            "yaw": round(_f(e[2]), 3),
        }
        for i in range(6):
            mi = m[i] if i < len(m) else {}
            s[f"p{i}"] = round(_f(mi.get("position")), 3)
            s[f"q{i}"] = round(_f(mi.get("torque")), 4)
        return s

    # ---------------------------------------------------------------- 状态
    def state(self, since=0):
        # 锁内只做"取快照"这类廉价操作 (O(1) 或 O(缓冲长度) 的指针拷贝),
        # 过滤 / 字典构造 / recorder.info() 一律放到锁外:
        # 读取线程每帧都要抢这把锁, 锁内多耗 1 ms 就等于丢一帧。
        with self.lock:
            now = time.perf_counter()
            t0, f0 = self._hz_mark
            if self.connected and now - t0 >= 0.4:
                self._hz = (self.parser.frames_ok - f0) / (now - t0)
                self._hz_mark = (now, self.parser.frames_ok)
            elif not self.connected:
                self._hz = 0.0
            snap = list(self.samples)          # 900 个指针拷贝, 微秒级
            d = self.latest
            seq = self.seq
            connected = self.connected
            port, baudrate, error = self.port, self.baudrate, self.error
            t_connect = self.t_connect
            hz, parser = self._hz, self.parser
            frame_types = set(self.frame_types)

        samples = [s for s in snap if s["seq"] > since]
        latest = None
        if d:
            motors = []
            for i in range(6):
                m = d["motors"][i] if d.get("motors") and i < len(d["motors"]) else {}
                motors.append({
                    "name": MOTOR_NAMES[i],
                    "pos": m.get("position"), "vel": m.get("velocity"),
                    "torque": m.get("torque"), "temp": m.get("temp")})
            e = d.get("euler_deg")
            latest = {
                "type": d.get("type"),
                "count": d.get("count"),
                "lw": d.get("lw_speed"), "rw": d.get("rw_speed"),
                "spd": d.get("speed"), "av": d.get("angular_vel"),
                "acc": d.get("acc"), "gyro": d.get("gyro"),
                "quat": d.get("quat"),
                "euler": list(e) if e else None,
                "motors": motors,
                "sbus": d.get("sbus"),
                "switched_on": d.get("switched_on"),
                "model": d.get("model"),
            }
        ftype = ("全量182B" if frame_types == {"full"} else "精简124B" if frame_types == {"lite"}
                 else "混合" if frame_types else "--")
        rec = self.recorder.info()
        return {
            "connected": connected,
            "port": port,
            "baudrate": baudrate,
            "error": error,
            "elapsed": (now - t_connect) if (connected and t_connect) else 0.0,
            "stats": {
                "hz": round(hz, 1),
                "frames": parser.frames_ok,
                "variant": parser.frames_variant,
                "crc_err": parser.frames_crc_err,
                "bad_len": parser.frames_bad_len,
                "dropped": parser.bytes_dropped,
                "bytes": parser.total_bytes,
                "frame_type": ftype,
            },
            "latest": latest,
            "seq": seq,
            "samples": samples,
            "recording": bool(rec),
            "record": ({"name": rec["name"], "dir": rec["dir"],
                        "duration": round(rec["duration"], 1),
                        "rows": rec["rows"], "raw_bytes": rec["raw_bytes"],
                        "frames_full": rec["frames_full"],
                        "frames_lite": rec["frames_lite"]} if rec else None),
        }

    # ---------------------------------------------------------------- 录制
    def start_record(self, name, save_dir, save_csv=True, save_bin=True):
        with self.lock:
            try:
                info = self.recorder.start(
                    name=name, save_dir=save_dir or DEFAULT_LOG_DIR,
                    parser=self.parser,
                    port=self.port, baudrate=self.baudrate,
                    save_csv=save_csv, save_bin=save_bin)
            except RecorderError as e:
                return {"ok": False, "msg": str(e)}
            return {"ok": True, **info}

    def stop_record(self):
        with self.lock:
            # 传入 parser 以便把丢弃字节/缺头帧等链路指标写入 meta
            s = self.recorder.stop(self.parser)
            if s is None:
                return {"ok": False, "msg": "当前未在录制"}
            return {"ok": True, **s}


LINK = RobotLink()


# ================================================================== HTTP
class Handler(BaseHTTPRequestHandler):
    server_version = "RobotMonitor/1.0"

    def log_message(self, fmt, *args):      # 静默访问日志
        pass

    # ------------------------------------------------------------ 工具
    def _send(self, code, obj=None, ctype="application/json; charset=utf-8"):
        if obj is None:
            body = b""
        elif isinstance(obj, (bytes, bytearray)):
            body = bytes(obj)
        elif isinstance(obj, str):
            body = obj.encode("utf-8")
        else:
            body = json.dumps(obj, ensure_ascii=False).encode("utf-8")
        self.send_response(code)
        self.send_header("Content-Type", ctype)
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        if body:
            self.wfile.write(body)

    def _body_json(self):
        n = int(self.headers.get("Content-Length") or 0)
        if n <= 0:
            return {}
        try:
            return json.loads(self.rfile.read(n).decode("utf-8"))
        except Exception:
            return {}

    # ------------------------------------------------------------ 路由
    def do_GET(self):
        path = self.path.split("?", 1)[0]
        qs = self.path.split("?", 1)[1] if "?" in self.path else ""
        if path in ("/", "/index.html"):
            try:
                with open(HTML_PATH, "rb") as f:
                    return self._send(200, f.read(), "text/html; charset=utf-8")
            except OSError as e:
                return self._send(500, f"无法读取页面 {HTML_PATH}: {e}", "text/plain; charset=utf-8")
        if path == "/api/ports":
            return self._send(200, {"ports": list_ports()})
        if path == "/api/state":
            since = 0
            for kv in qs.split("&"):
                if kv.startswith("since="):
                    try:
                        since = int(kv.split("=", 1)[1])
                    except ValueError:
                        since = 0
            return self._send(200, LINK.state(since))
        if path == "/api/config":
            return self._send(200, {"log_dir": DEFAULT_LOG_DIR,
                                    "baudrates": [9600, 115200, 230400, 460800,
                                                  921600, 1000000, 2000000, 3000000],
                                    "channels": CHANNELS})
        return self._send(404, {"ok": False, "msg": "not found"})

    def do_POST(self):
        path = self.path.split("?", 1)[0]
        data = self._body_json()
        if path == "/api/connect":
            port = data.get("port", "")
            baud = int(data.get("baudrate") or DEFAULT_BAUD)
            return self._send(200, LINK.connect(port, baud))
        if path == "/api/disconnect":
            return self._send(200, LINK.disconnect())
        if path == "/api/record/start":
            return self._send(200, LINK.start_record(
                name=data.get("name", ""),
                save_dir=data.get("dir") or DEFAULT_LOG_DIR,
                save_csv=bool(data.get("csv", True)),
                save_bin=bool(data.get("bin", True))))
        if path == "/api/record/stop":
            return self._send(200, LINK.stop_record())
        if path == "/api/open_dir":
            d = os.path.abspath(os.path.expanduser(data.get("dir", "")))
            if not os.path.isdir(d):
                return self._send(200, {"ok": False, "msg": f"目录不存在: {d}"})
            try:
                subprocess.Popen(["open", d])
            except OSError as e:
                return self._send(200, {"ok": False, "msg": str(e)})
            return self._send(200, {"ok": True, "dir": d})
        return self._send(404, {"ok": False, "msg": "not found"})


def main():
    ap = argparse.ArgumentParser(description="轮式腿机器人 Web 上位机")
    ap.add_argument("--port", type=int, default=8765, help="HTTP 端口 (默认 8765)")
    ap.add_argument("--no-browser", action="store_true", help="不自动打开浏览器")
    ap.add_argument("--no-auto-connect", action="store_true", help="启动不自动连接串口")
    args = ap.parse_args()

    if not os.path.exists(HTML_PATH):
        print(f"缺少页面文件: {HTML_PATH}")
        return 1
    os.makedirs(DEFAULT_LOG_DIR, exist_ok=True)

    httpd = ThreadingHTTPServer(("127.0.0.1", args.port), Handler)
    httpd.daemon_threads = True
    url = f"http://127.0.0.1:{args.port}/"
    print(f"上位机已启动: {url}")
    print("串口:", list_ports() or "(未检测到)")

    def _graceful_exit(signum, _frame):
        # 在独立线程里 shutdown, 避免与 serve_forever 同线程死锁
        print(f"\n收到信号 {signum}, 正在退出…", flush=True)
        threading.Thread(target=httpd.shutdown, daemon=True).start()

    for _sig in (signal.SIGINT, signal.SIGTERM):
        signal.signal(_sig, _graceful_exit)

    if not args.no_auto_connect:
        r = LINK.auto_connect()
        if r:
            print("自动连接:", "成功" if r.get("ok") else "失败", r.get("port", ""),
                  r.get("msg", ""))

    if not args.no_browser:
        threading.Timer(0.6, lambda: webbrowser.open(url)).start()

    try:
        httpd.serve_forever()
    except KeyboardInterrupt:
        print("\n正在退出…")
    finally:
        LINK.disconnect()
        httpd.server_close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
