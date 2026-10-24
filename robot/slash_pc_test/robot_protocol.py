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

变体帧 (实测固件存在, 本解析器自动识别):
    181 字节: [3 字节前缀][178 字节 = ROS_body_t 去掉4字节帧头后的载荷(含CRC)]
    机器人每个循环发送两帧: 一帧标准 182B + 一帧此变体 (数据内容相同)。
    识别方式: 完整帧之后, 尝试将后续 181 字节按变体解释, 并用标准帧头
    重建后做 CRC16 校验, 通过才认定 (误识别概率 ~1/65536, 且无害)。

字段偏移 (已用 clang static_assert 依据 robot_structs.h 验证):
    ROS_body_t : milemeter@4, ImuData@20, MIT_Feedback@60, SBUS@156,
                 Status@176, count@178, crc@180
    ROS_body_t1: MIT_Feedback@4, SBUS@100, count@120, crc@122
"""

import math
import struct
import time

HEADER1 = 0x7B  # 123
HEADER2 = 0x2D  # 45
HEADER = bytes((HEADER1, HEADER2))

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


class FrameParser:
    """字节流帧解析器 (状态机与固件 SERIAL_RXT 对齐)

    状态流转: SEARCH(找帧头) -> HEADER(收长度字段) -> BODY(收载荷至 length) -> SEARCH
    """

    SEARCH, HEADER, BODY = 0, 1, 2

    def __init__(self, allowed_lengths=(FRAME_FULL_LEN, FRAME_LITE_LEN),
                 enable_variant=True):
        self.allowed = tuple(sorted(set(allowed_lengths)))
        self.enable_variant = enable_variant
        self.buf = bytearray()
        self.state = self.SEARCH
        self.need = 0
        # 统计
        self.total_bytes = 0
        self.frames_ok = 0
        self.frames_crc_err = 0
        self.frames_bad_len = 0
        self.frames_variant = 0
        self.bytes_dropped = 0

    def reset_stats(self):
        self.total_bytes = 0
        self.frames_ok = 0
        self.frames_crc_err = 0
        self.frames_bad_len = 0
        self.frames_variant = 0
        self.bytes_dropped = 0

    def feed(self, data: bytes):
        """喂入一段字节, 返回 [(length, frame_bytes, crc_ok), ...]"""
        frames = []
        self.total_bytes += len(data)
        self.buf.extend(data)
        buf = self.buf
        while True:
            if self.state == self.SEARCH:
                # 变体帧探测: 缓冲区不以标准帧头开头时, 尝试按
                # [3B 前缀 + 178B 标准载荷(含CRC)] 解释并用标准帧头重建校验
                if self.enable_variant and buf[:2] != HEADER:
                    if len(buf) < FRAME_VARIANT_LEN:
                        # 数据不足: 等待更多字节 (避免在变体帧内部搜到伪帧头)
                        break
                    payload = bytes(buf[3:FRAME_VARIANT_LEN])
                    frame = (HEADER
                             + bytes((FRAME_FULL_LEN & 0xFF, FRAME_FULL_LEN >> 8))
                             + payload)
                    if (crc16(frame[:-2])
                            == (frame[-2] | (frame[-1] << 8))):
                        del buf[:FRAME_VARIANT_LEN]
                        self.frames_ok += 1
                        self.frames_variant += 1
                        frames.append((FRAME_VARIANT_LEN, frame, True))
                        continue
                idx = buf.find(HEADER)
                if idx < 0:
                    # 无帧头: 丢弃除末尾 1 字节外的全部 (帧头可能被 chunk 边界截断)
                    if len(buf) > 1:
                        self.bytes_dropped += len(buf) - 1
                        del buf[:-1]
                    break
                if idx > 0:
                    self.bytes_dropped += idx
                    del buf[:idx]
                self.state = self.HEADER
                continue
            if self.state == self.HEADER:
                if len(buf) < 4:
                    break
                length = buf[2] | (buf[3] << 8)
                if length not in self.allowed:
                    # 伪帧头, 丢掉 2 字节继续同步
                    self.frames_bad_len += 1
                    del buf[:2]
                    self.state = self.SEARCH
                    continue
                self.need = length
                self.state = self.BODY
                continue
            # self.state == self.BODY
            if len(buf) < self.need:
                break  # 帧未收完整, 等待更多数据 (载荷中即使含 0x7B2D 也不搜帧头)
            frame = bytes(buf[:self.need])
            del buf[:self.need]
            crc_calc = crc16(frame[:-2])
            crc_recv = frame[-2] | (frame[-1] << 8)
            ok = (crc_calc == crc_recv)
            if ok:
                self.frames_ok += 1
            else:
                self.frames_crc_err += 1
            frames.append((self.need, frame, ok))
            self.state = self.SEARCH
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
    print("robot_protocol 自测通过: CRC/整帧/分片/帧间噪声/帧中污染恢复/变体帧/无变体防误报/欧拉角 全部正确")


if __name__ == "__main__":
    _selftest()
