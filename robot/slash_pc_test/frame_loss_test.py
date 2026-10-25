#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""串口丢帧率实测：区分「PC 端读取被拖慢」与「下位机/链路本身丢帧」

背景
----
真机录制曾出现约 15% 丢帧（缺口都是 10 ms 的整数倍，即整控制周期丢失）。
2026-10-25 已定位：**责任方是 FrameParser 的重同步逻辑**（链路每丢 1 个标准帧首字节
就错位一次，旧实现随即以 180 字节为单位丢弃数据，平均吞掉 2 个完好帧），
而非下位机。修复后同一份录制可从 1274 帧恢复到 1503 帧（99.4 Hz，零丢帧）。

本工具仍可用于区分下列各档负载是否有影响：

  baseline  只读 + CRC 解析，不录盘、不起 HTTP   → 链路与解析的极限性能
  record    再加 bin/CSV 落盘（等价现行录制路径） → 写盘的影响
  http      再加 HTTP 轮询负载（等价浏览器开着）  → GIL 抢占的影响

判读
----
- baseline 就丢帧      → 链路确有字节丢失 / 解析未完全恢复，看 bytes_dropped 与
                          frames_headless（缺头帧数应与链路丢字节次数相当）
- baseline 不丢、record 丢 → 落盘阻塞读取线程，需把写盘移到独立线程
- baseline/record 不丢、http 丢 → HTTP 负载抢占 GIL，需削减 /api/state 负载
- 全模式都不丢          → 当前链路与代码健康

用法
----
    python3 frame_loss_test.py [秒数] [模式 ...]
    python3 frame_loss_test.py 15                 # baseline/record/http 都跑
    python3 frame_loss_test.py 15 baseline record

注意
----
* 运行前必须确认没有其他进程在读同一串口 (本服务已用 TIOCEXCL 独占,
  第二个读取器会被拒绝); 两个读取器会互相撕碎字节流, 测得的结果全废。
* 实测结论 (2026-10-25, macOS): 修复前丢帧约 13% 且与读取循环写法无关
  (in_waiting+sleep 与 select+自旋无统计差别)。该结论当时被误读成「下位机
  发送侧丢字节」, 实测并非如此: 链路只丢了 0.04% 的字节 (每次仅 1 字节,
  且总是某个标准帧的首字节 0x7B), 丢帧是 FrameParser 错位后的 180 字节
  粗粒度丢弃造成的。解析器改为锚点扫描式后, 实测可达 99.4 Hz / 零丢帧。
"""

import json
import sys
import threading
import time
import urllib.request
from collections import Counter
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

from mac_serial import SerialPort, list_ports
from robot_protocol import FrameParser, decode_frame

BAUD = 2000000


class RecorderStub:
    """最小录制桩：只累计字节，不落盘"""

    def __init__(self):
        self.n_bytes = 0

    def write_raw(self, data):
        self.n_bytes += len(data)

    def write_frame(self, d):
        pass

    def close(self):
        pass


class DiskRecorder:
    """等价现行录制路径：每读一段写 bin、每帧写一行 CSV（写临时文件）"""

    def __init__(self, tag):
        self.f_bin = open(f"/tmp/floss_{tag}.bin", "wb")
        self.f_csv = open(f"/tmp/floss_{tag}.csv", "w")
        self.f_csv.write("t_s,count,pitch,speed\n")
        self.n_rows = 0

    def write_raw(self, data):
        self.f_bin.write(data)

    def write_frame(self, d):
        self.f_csv.write("%.4f,%d,%.4f,%.4f\n" % (
            d.get("host_t", 0.0), d.get("count", 0),
            (d.get("euler_deg") or (0, 0, 0))[1], d.get("speed", 0.0)))
        self.n_rows += 1

    def close(self):
        self.f_bin.close()
        self.f_csv.close()


class HttpLoad:
    """模拟浏览器轮询：每次请求序列化一份较大 JSON（与现行 /api/state 同量级）"""

    def __init__(self, samples):
        self.samples = samples
        self.hits = 0

        class H(BaseHTTPRequestHandler):
            def do_GET(inner):
                body = json.dumps({"samples": samples[-3000:]}).encode()
                inner.send_response(200)
                inner.send_header("Content-Type", "application/json")
                inner.send_header("Content-Length", str(len(body)))
                inner.end_headers()
                inner.wfile.write(body)
                self.hits += 1

            def log_message(inner, *a):
                pass

        self.srv = ThreadingHTTPServer(("127.0.0.1", 8791), H)
        self.srv.daemon_threads = True
        threading.Thread(target=self.srv.serve_forever, daemon=True).start()

    def poll_loop(self, stop):
        while not stop.is_set():
            try:
                urllib.request.urlopen("http://127.0.0.1:8791/api/state",
                                       timeout=2).read()
            except Exception:
                pass
            time.sleep(0.2)          # 5 Hz，与前端轮询同量级

    def stop(self):
        self.srv.shutdown()


def run(port, secs, mode, samples):
    ser = SerialPort(port, BAUD).open()
    parser = FrameParser()
    rec = DiskRecorder(mode) if mode == "record" else RecorderStub()
    http = None
    stop = threading.Event()
    if mode == "http":
        http = HttpLoad(samples)
        threading.Thread(target=http.poll_loop, args=(stop,), daemon=True).start()

    t0 = time.perf_counter()
    dts, counts = [], []
    try:
        while time.perf_counter() - t0 < secs:
            n = ser.in_waiting()
            if n <= 0:
                time.sleep(0.0004)
                continue
            t_read = time.perf_counter()
            chunk = ser.read(n)
            if not chunk:
                continue
            rec.write_raw(chunk)
            for _len, frame, ok in parser.feed(chunk):
                if not ok:
                    continue
                d = decode_frame(frame)
                if d is None:
                    continue
                d["host_t"] = t_read
                samples.append(d)
                rec.write_frame(d)
                counts.append(d.get("count", 0) & 0xFFFF)
                dts.append(t_read)
    finally:
        stop.set()
        if http:
            http.stop()
        ser.close()
        rec.close()

    elapsed = time.perf_counter() - t0
    n = len(counts)
    # ---- 丢帧率：用固件 count 计数器跨度 ----
    span, anomaly = 0, 0
    for i in range(n - 1):
        diff = (counts[i + 1] - counts[i]) % 65536
        if diff > 32768:
            anomaly += 1
        else:
            span += diff
    nominal = span + 1 if span else n
    loss_cnt = 1 - n / nominal if nominal else 0.0
    # ---- 丢帧率：按标称 100 Hz 计时 ----
    loss_t = 1 - n / (elapsed * 100)
    # ---- 到达间隔 ----
    iv = [(dts[i + 1] - dts[i]) * 1000 for i in range(n - 1)]
    med = sorted(iv)[len(iv) // 2] if iv else 0.0
    big = Counter(round(x / 10) * 10 for x in iv if x > 15)

    print(f"\n[{mode}] {secs:.0f}s  收帧 {n}（{n / elapsed:.1f} Hz）  "
          f"字节 {parser.total_bytes}")
    print(f"  CRC 错 {parser.frames_crc_err}   丢弃字节 {parser.bytes_dropped}"
          f"（{parser.bytes_dropped / max(1, parser.total_bytes) * 100:.1f}%）")
    print(f"  count 跨度 {span}（计数器回跳 {anomaly} 次）→ 丢帧 "
          f"{loss_cnt * 100:.1f}%   按 100 Hz 计时丢帧 {loss_t * 100:.1f}%")
    print(f"  到达间隔中位 {med:.1f} ms；>15 ms 大间隔 "
          f"{sum(1 for x in iv if x > 15)} 次：{dict(sorted(big.items())[:8])}")
    if http:
        print(f"  HTTP 轮询请求 {http.hits} 次")
    if mode == "record":
        print(f"  CSV 行数 {rec.n_rows}（/tmp/floss_record.csv）")
    return loss_cnt


def main():
    argv = sys.argv[1:]
    secs = float(argv[0]) if argv and not argv[0].isalpha() else 15.0
    modes = [a for a in argv if a.isalpha()] or ["baseline", "record", "http"]
    ports = list_ports()
    port = next((p for p in ports if "usbmodem" in p or "usbserial" in p),
                ports[0] if ports else None)
    if not port:
        print("未检测到串口，请确认机器人已连接且 USB 已分配给本机"
              "（UTM 虚拟机可能在占用该 USB 设备）")
        return 1
    print(f"串口 {port} @ {BAUD}，时长 {secs:.0f}s，模式 {modes}")
    samples = []
    res = {}
    for m in modes:
        res[m] = run(port, secs, m, samples)
        time.sleep(0.5)
    print("\n==== 汇总 ====")
    for m, v in res.items():
        print(f"  {m:<9} 丢帧 {v * 100:5.1f}%")
    return 0


if __name__ == "__main__":
    sys.exit(main())
