# -*- coding: utf-8 -*-
"""
机器人串口协议解析库
====================

与下位机固件 (LeggedRobot.ino / LeggedRobot.h) 及上位机参考代码
(robot_core/robot/wheeled_legged_pkg/src/robot_functions.cpp) 严格对齐:

帧格式 (小端):
    +--------+--------+----------+------------------+----------+
    | 0x7B   | 0x2D   | len:u16  | payload          | crc:u16  |
    +--------+--------+----------+------------------+----------+
    帧头1=123 帧头2=45  帧总长度   数据(含count)      CRC16

CRC16: Modbus 风格 (初值 0xFFFF, 多项式反转 0xA001),
       覆盖除 CRC 字段外的全部字节 (即 frame[0 : len-2])

帧类型:
    ROS_body_t  182 字节  全量帧 (里程计 + IMU + 6电机反馈 + SBUS + 状态)
    ROS_body_t1 124 字节  精简帧 (6电机反馈 + SBUS)

帧.type (实测固件会以三种等价形态发送同一份数据, 本解析器全部识别):
    1. 标准帧  182B: [7B 2D][B6 00][payload 176B][count:u16][crc:u16]
    2. 变体帧  181B: [BD 4B 1B][payload 176B][count][crc]
       —— 固件的第二条发送路径使用 3 字节魔数代替 4 字节帧头,
          载荷/count/CRC 与标准帧完全一致, 去掉魔数后按标准帧头重建即可校验。
    3. 缺头帧  181B: [2D B6 00][payload][count][crc]
       —— 标准帧被链路吞掉首字节 0x7B 后的形态 (实测链路平均每次只丢 1 字节,
          且丢失位置总是某个标准帧的第一个字节)。补回 0x7B 后同样可校验。
    三种形态互斥地散布在字节流中, 每个控制周期只发其中一种, 总帧率 100 Hz。

字段偏移 (已用 clang static_assert 依据 robot_structs.h 验证):
    ROS_body_t : milemeter@4, ImuData@20, MIT_Feedback@60, SBUS@156,
                 Status@176, count@178, crc@180
    ROS_body_t1: MIT_Feedback@4, SBUS@100, count@120, crc@122
"""

import glob
import json
import math
import os
import struct
import time

HEADER1 = 0x7B  # 123
HEADER2 = 0x2D  # 45
HEADER = bytes((HEADER1, HEADER2))

# 固件第二发送路径的 3 字节魔数 (实测稳定出现在变体帧首位)
VARIANT_MAGIC = b"\xbd\x4b\x1b"

FRAME_FULL_LEN = 182   # ROS_body_t
FRAME_LITE_LEN = 124   # ROS_body_t1
FRAME_VARIANT_LEN = 181  # 变体帧: 3B 前缀 + ROS_body_t 载荷(去帧头)

FRAME_FULL_NAME = "ROS_body_t"
FRAME_LITE_NAME = "ROS_body_t1"

MOTOR_NAMES = ("左前", "左后", "右前", "右后", "左轮", "右轮")
MOTOR_NAMES_EN = ("left_front", "left_rear", "right_front",
                  "right_rear", "left_wheel", "right_wheel")


def crc16(data) -> int:
    """CRC16 (Modbus), 与固件/上位机 C++ 实现完全一致"""
    crc = 0xFFFF
    for b in data:
        crc ^= b
        for _ in range(8):
            if crc & 0x0001:
                crc = (crc >> 1) ^ 0xA001
            else:
                crc >>= 1
    return crc & 0xFFFF


def quat_to_euler_deg(q0, q1, q2, q3):
    """四元数 -> 欧拉角(度), 与 robot_functions.cpp printImuData 一致"""
    roll = math.atan2(2.0 * (q0 * q1 + q2 * q3), 1.0 - 2.0 * (q1 * q1 + q2 * q2))
    pitch = math.asin(max(-1.0, min(1.0, 2.0 * (q0 * q2 - q3 * q1))))
    yaw = math.atan2(2.0 * (q0 * q3 + q1 * q2), 1.0 - 2.0 * (q2 * q2 + q3 * q3))
    return (math.degrees(roll), math.degrees(pitch), math.degrees(yaw))


class _Pattern:
    """一种「帧起点形态」的描述

    pat        在字节流中用于定位的字节序列
    prefix     重建完整帧时使用的前缀 (可能比 pat 长, 如补回丢失的 0x7B)
    stream_len 该形态在字节流中实际占据的字节数
    kind       统计归类: full / headless / variant
    """

    __slots__ = ("pat", "prefix", "stream_len", "kind")

    def __init__(self, pat, prefix, stream_len, kind):
        self.pat = pat
        self.prefix = prefix
        self.stream_len = stream_len
        self.kind = kind


class FrameParser:
    """字节流帧解析器 —— 锚点扫描式, 天然免疫「丢字节造成的错位」

    为什么不用 SEARCH→HEADER→BODY 状态机
    -----------------------------------
    旧实现一旦因为「链路丢了 1 个字节」而与帧边界错位, 恢复方式是:
    攒满 181 字节赌一次变体 CRC, 赌错就把缓冲区砍到只剩最后 1 字节
    (一次丢 180 字节) 继续爬。实测 (2026-10-25, 两份各 15~17 s 录制):
    链路总共只丢了 115 / 143 个字节 (0.04%), 但每次错位平均被这种
    「180 字节一跳」的爬行吞掉 2 个完好帧 → 丢帧率 15.2% / 17.9%。

    本实现改为先定位「可信锚点」再用 CRC 确认:
        * 每种合法帧长度 L 生成 3 种锚点形态 (完整帧头 / 缺首字节 / 变体魔数)
        * 用 bytes.find 在缓冲区里找最靠前的锚点 (C 层实现, 很快)
        * 数据够就 CRC 校验; 通过整帧原子消费, 不通过就从锚点后继续找
        * 整个缓冲区都找不到锚点时, 只裁到「可能是不完整锚点前缀」的尾巴
    缓冲区因此始终维持在一帧量级, 且永远不会「跨过」任何一个合法帧。
    """

    SEARCH, HEADER, BODY = 0, 1, 2      # 仅为兼容旧引用保留

    def __init__(self, allowed_lengths=(FRAME_FULL_LEN, FRAME_LITE_LEN),
                 enable_variant=True):
        self.allowed = tuple(sorted(set(allowed_lengths)))
        self.enable_variant = enable_variant
        self.buf = bytearray()
        self.patterns = self._build_patterns()
        # 最短锚点 3 字节 → 至少保留 2 字节, 防止裁掉跨 chunk 的不完整锚点
        self._keep_tail = max(len(p.pat) for p in self.patterns) - 1
        self._min_len = min(p.stream_len for p in self.patterns)
        # 统计
        self.total_bytes = 0
        self.frames_ok = 0
        self.frames_crc_err = 0
        self.frames_bad_len = 0
        self.frames_variant = 0
        self.frames_headless = 0        # 链路缺头后补回 0x7B 恢复的帧
        self.bytes_dropped = 0

    def _build_patterns(self):
        pats = []
        for L in self.allowed:
            len_bytes = bytes((L & 0xFF, L >> 8))
            full_head = HEADER + len_bytes
            pats.append(_Pattern(full_head, full_head, L, "full"))
            # 标准帧被链路吞掉首字节 0x7B: 补回去即可正常校验
            pats.append(_Pattern(full_head[1:], full_head, L - 1, "headless"))
        if self.enable_variant and FRAME_FULL_LEN in self.allowed:
            head = HEADER + bytes((FRAME_FULL_LEN & 0xFF, FRAME_FULL_LEN >> 8))
            pats.append(_Pattern(VARIANT_MAGIC, head,
                                 FRAME_FULL_LEN - 1, "variant"))
        # 同一位置上更长的锚点优先 (避免把完整帧头误判成缺头帧)
        pats.sort(key=lambda p: -len(p.pat))
        return pats

    def reset_stats(self):
        self.total_bytes = 0
        self.frames_ok = 0
        self.frames_crc_err = 0
        self.frames_bad_len = 0
        self.frames_variant = 0
        self.frames_headless = 0
        self.bytes_dropped = 0

    def _locate(self, buf):
        """找最靠前的可用锚点

        返回 (pos, pattern, frame, ok):
            frame 为 None 表示锚点已定位但数据未到齐, 需继续等待
            ok    为 CRC 校验结果 (frame 为 None 时无意义)
        返回 None 表示缓冲区中不存在任何锚点。
        """
        start = 0
        while True:
            hit_pos, hit_pat = None, None
            for pat in self.patterns:
                i = buf.find(pat.pat, start)
                if i < 0:
                    continue
                # 位置相同时取更长的锚点 (patterns 已按长度降序, 先到先得)
                if hit_pos is None or i < hit_pos:
                    hit_pos, hit_pat = i, pat
            if hit_pos is None:
                return None
            # 无论成败都推进, 保证前进性 (CRC 不过的锚点会被跳过)
            start = hit_pos + 1
            end = hit_pos + hit_pat.stream_len
            if end > len(buf):
                return (hit_pos, hit_pat, None, False)   # 数据未齐, 等待
            frame = hit_pat.prefix + bytes(buf[hit_pos + len(hit_pat.pat):end])
            ok = crc16(frame[:-2]) == (frame[-2] | (frame[-1] << 8))
            return (hit_pos, hit_pat, frame, ok)

    def feed(self, data: bytes):
        """喂入一段字节, 返回 [(length, frame_bytes, crc_ok), ...]"""
        frames = []
        self.total_bytes += len(data)
        buf = self.buf
        buf.extend(data)
        while True:
            if len(buf) < self._min_len:
                break                     # 还装不下一帧, 等待更多数据
            hit = self._locate(buf)
            if hit is None:
                # 毫无锚点: 只保留可能构成未完成锚点的尾巴
                keep = min(self._keep_tail, len(buf))
                if len(buf) > keep:
                    self.bytes_dropped += len(buf) - keep
                    del buf[:len(buf) - keep]
                break
            pos, pat, frame, ok = hit
            if pos:
                self.bytes_dropped += pos
                del buf[:pos]
            if frame is None:
                break                     # 锚点已在 buf[0], 等待数据补齐
            del buf[:pat.stream_len]
            if ok:
                self.frames_ok += 1
                if pat.kind == "variant":
                    self.frames_variant += 1
                elif pat.kind == "headless":
                    self.frames_headless += 1
            else:
                self.frames_crc_err += 1
            frames.append((len(frame), frame, ok))
        return frames


def _decode_motors(f, off):
    motors = []
    for _ in range(6):
        pos, vel, torque, temp = struct.unpack_from("<4f", f, off)
        motors.append({"position": pos, "velocity": vel,
                       "torque": torque, "temp": temp})
        off += 16
    return motors


def _decode_status(status_code):
    return {"status_code": status_code,
            "switched_on": status_code & 0x1,
            "model": (status_code >> 1) & 0x1}


def decode_full(frame: bytes):
    """解码 ROS_body_t (182B)"""
    if len(frame) != FRAME_FULL_LEN:
        return None
    d = {"type": "full", "type_name": FRAME_FULL_NAME,
         "length": len(frame), "host_t": time.time()}
    lw, rw, spd, av = struct.unpack_from("<4f", frame, 4)
    d["lw_speed"], d["rw_speed"], d["speed"], d["angular_vel"] = lw, rw, spd, av
    ax, ay, az, gx, gy, gz, q0, q1, q2, q3 = struct.unpack_from("<10f", frame, 20)
    d["acc"] = (ax, ay, az)
    d["gyro"] = (gx, gy, gz)
    d["quat"] = (q0, q1, q2, q3)
    d["euler_deg"] = quat_to_euler_deg(q0, q1, q2, q3)
    d["motors"] = _decode_motors(frame, 60)
    d["sbus"] = list(struct.unpack_from("<10h", frame, 156))
    d.update(_decode_status(struct.unpack_from("<H", frame, 176)[0]))
    d["count"] = struct.unpack_from("<H", frame, 178)[0]
    d["crc"] = struct.unpack_from("<H", frame, 180)[0]
    return d


def decode_lite(frame: bytes):
    """解码 ROS_body_t1 (124B)"""
    if len(frame) != FRAME_LITE_LEN:
        return None
    d = {"type": "lite", "type_name": FRAME_LITE_NAME,
         "length": len(frame), "host_t": time.time()}
    d["motors"] = _decode_motors(frame, 4)
    d["sbus"] = list(struct.unpack_from("<10h", frame, 100))
    d["count"] = struct.unpack_from("<H", frame, 120)[0]
    d["crc"] = struct.unpack_from("<H", frame, 122)[0]
    # 精简帧无以下字段
    for k in ("lw_speed", "rw_speed", "speed", "angular_vel", "acc", "gyro",
              "quat", "euler_deg", "status_code", "switched_on", "model"):
        d[k] = None
    return d


def decode_frame(frame: bytes):
    """按长度字段自动选择解码器"""
    length = frame[2] | (frame[3] << 8)
    if length == FRAME_FULL_LEN:
        return decode_full(frame)
    if length == FRAME_LITE_LEN:
        return decode_lite(frame)
    return None


def build_full_frame(lw=0.1, rw=0.1, speed=0.1, av=0.0,
                     acc=(0.0, 0.0, 9.8), gyro=(0.0, 0.0, 0.0),
                     quat=(1.0, 0.0, 0.0, 0.0),
                     motors=None, sbus=None, status=0, count=0):
    """构造一帧 ROS_body_t (用于离线自测 / 回放)"""
    f = bytearray(FRAME_FULL_LEN)
    f[0], f[1] = HEADER1, HEADER2
    struct.pack_into("<H", f, 2, FRAME_FULL_LEN)
    struct.pack_into("<4f", f, 4, lw, rw, speed, av)
    struct.pack_into("<10f", f, 20, *acc, *gyro, *quat)
    if motors is None:
        motors = [(float(i), 0.0, 0.0, 25.0) for i in range(6)]
    off = 60
    for (p, v, t, temp) in motors:
        struct.pack_into("<4f", f, off, p, v, t, temp)
        off += 16
    if sbus is None:
        sbus = [1024] * 10
    struct.pack_into("<10h", f, 156, *sbus)
    struct.pack_into("<H", f, 176, status)
    struct.pack_into("<H", f, 178, count)
    struct.pack_into("<H", f, 180, crc16(bytes(f[:-2])))
    return bytes(f)


def _lossless_regression():
    """端到端回归: 用仓库内真实录制的 raw.bin 验证「零丢帧」

    判据不是帧数多寡, 而是恢复出的每一帧 firmware count 必须严格 +1 ——
    只要有一帧没被找回来, count 就会出现断裂。
    目录不存在时返回 None (跳过)。
    """
    here = os.path.dirname(os.path.abspath(__file__))
    bins = sorted(glob.glob(os.path.join(here, "录制数据", "*", "raw.bin")))
    if not bins:
        return None
    report = []
    for path in bins:
        with open(path, "rb") as fh:
            raw = fh.read()
        p = FrameParser()
        got = []
        for i in range(0, len(raw), 64):
            for _l, fr, ok in p.feed(raw[i:i + 64]):
                if ok:
                    got.append(fr[178] | (fr[179] << 8))
        tag = os.path.basename(os.path.dirname(path))
        brk = sum(1 for i in range(len(got) - 1)
                  if ((got[i + 1] - got[i]) & 0xFF) != 1)
        assert brk == 0, f"{tag}: count 断裂 {brk} 处, 仍有丢帧"
        assert p.frames_crc_err == 0, f"{tag}: 出现 {p.frames_crc_err} 次 CRC 错误"
        assert p.bytes_dropped <= len(raw) * 0.001, \
            f"{tag}: 丢弃字节 {p.bytes_dropped} 超出预期"
        dur = None
        meta = os.path.join(os.path.dirname(path), "meta.json")
        if os.path.exists(meta):
            with open(meta, encoding="utf-8") as fh:
                dur = json.load(fh).get("duration_s")
        report.append((tag, len(got), dur))
    return report


def _selftest():
    """合成帧编解码 + CRC 一致性自测"""
    # CRC 已知向量: 与 C++ 实现互导验证
    assert crc16(b"123456789") == 0x4B37, "CRC16 标准向量校验失败"
    frame = build_full_frame(lw=0.25, rw=0.25, speed=0.25, av=0.01,
                             acc=(0.1, 0.2, 9.8), gyro=(0.01, 0.0, 0.02),
                             quat=(0.996, 0.0, 0.087, 0.0),
                             motors=[(96.25, 1.0, 0.5, 30.0),
                                     (96.25, 1.0, 0.5, 30.0),
                                     (96.25, 1.0, 0.5, 30.0),
                                     (96.25, 1.0, 0.5, 30.0),
                                     (10.0, 2.0, 0.3, 28.0),
                                     (10.0, 2.0, 0.3, 28.0)],
                             sbus=[200, 1024, 1024, 1024, 1024, 1024, 1024, 1024, 1024, 1024],
                             status=0b11, count=7)
    p = FrameParser()
    frames = p.feed(frame)
    assert len(frames) == 1 and frames[0][0] == FRAME_FULL_LEN and frames[0][2], "整帧解析失败"
    d = decode_full(frame)
    assert abs(d["speed"] - 0.25) < 1e-6
    assert d["motors"][0]["position"] == 96.25
    assert d["switched_on"] == 1 and d["model"] == 1
    assert d["count"] == 7 and len(d["sbus"]) == 10
    roll, pitch, yaw = d["euler_deg"]
    assert abs(pitch - 9.98) < 0.05, f"pitch={pitch}"  # quat(0.996,0,0.087,0) ≈ 10° pitch
    # 分片 + 帧间噪声测试: 任意 chunk 边界 + 帧前后噪声, 两帧均应正确解析
    p2 = FrameParser()
    out = []
    noise = b"\x00\x11\x22\x33"
    stream = noise + frame + noise + frame + noise
    for i in range(0, len(stream), 7):
        out += p2.feed(stream[i:i + 7])
    assert len(out) == 2 and all(f[2] for f in out), "分片流解析失败"
    # 尾部不足变体帧长度的噪声会暂留缓冲区 (等待更多数据), 已丢弃 4+4=8
    assert p2.bytes_dropped == 8 and len(p2.buf) == 4, \
        f"丢弃字节数异常: {p2.bytes_dropped}, buf={len(p2.buf)}"
    # 帧中噪声: 该帧必被 CRC 捕获 (与固件状态机行为一致), 且后续帧能恢复同步
    p3 = FrameParser()
    out3 = []
    stream3 = frame[:50] + noise + frame[50:] + frame
    for i in range(0, len(stream3), 13):
        out3 += p3.feed(stream3[i:i + 13])
    assert len(out3) == 2 and not out3[0][2] and out3[1][2], "帧中污染恢复同步失败"
    assert p3.frames_crc_err == 1
    # 变体帧: [3B 前缀 + 标准帧去掉4B帧头] (实测固件每个循环发两帧)
    variant = b"\xbd\x4b\x1b" + frame[4:]
    p4 = FrameParser()
    out4 = []
    stream4 = frame + variant + frame + variant
    for i in range(0, len(stream4), 91):  # 任意分片
        out4 += p4.feed(stream4[i:i + 91])
    assert len(out4) == 4 and all(f[2] for f in out4), "变体帧解析失败"
    assert p4.frames_variant == 2, p4.frames_variant
    d4 = decode_frame(out4[1][1])
    assert d4["motors"][0]["position"] == 96.25 and d4["count"] == 7
    # 纯标准帧流 (无变体) 不得误识别
    p5 = FrameParser()
    out5 = []
    stream5 = frame * 5
    for i in range(0, len(stream5), 37):
        out5 += p5.feed(stream5[i:i + 37])
    assert len(out5) == 5 and all(f[2] for f in out5) and p5.frames_variant == 0
    # 缺头帧恢复: 链路吞掉标准帧首字节 0x7B (实测唯一的错位来源, 约 1 次/2372 字节)
    # 旧实现在这里会丢失整帧并为后续连锁丢帧; 新实现必须一帧不落且 count 连续
    src = [build_full_frame(count=c) for c in range(20)]
    stream6, n_headless = b"", 0
    for k, f in enumerate(src):
        if k % 3 == 0:                      # 每 3 帧挖掉一个首字节
            stream6 += f[1:]
            n_headless += 1
        else:
            stream6 += f
    p6 = FrameParser()
    out6 = []
    for i in range(0, len(stream6), 64):
        out6 += p6.feed(stream6[i:i + 64])
    assert len(out6) == 20 and all(f[2] for f in out6), "缺头帧恢复失败"
    assert p6.frames_headless == n_headless, f"缺头帧计数异常 {p6.frames_headless}"
    assert [(f[1][178] | (f[1][179] << 8)) for f in out6] == list(range(20)), \
        "恢复帧 count 不连续"
    assert p6.frames_crc_err == 0 and p6.bytes_dropped == 0
    reg = _lossless_regression()
    if reg:
        lines = "; ".join(f"{name} {n}帧" + (f" @ {n / d:.1f}Hz" if d else "")
                          for name, n, d in reg)
        tail = f"；真实录制零丢帧 [{lines}]"
    else:
        tail = "；未找到录制数据, 跳过端到端回归"
    print("robot_protocol 自测通过: CRC/整帧/分片/帧间噪声/帧中污染恢复/"
          "变体帧/无变体防误报/缺头帧恢复/欧拉角 全部正确" + tail)


if __name__ == "__main__":
    _selftest()
