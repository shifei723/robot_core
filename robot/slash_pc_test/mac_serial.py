# -*- coding: utf-8 -*-
"""
macOS 串口模块 (纯标准库实现, 无需安装 pyserial)
================================================

基于 termios + fcntl 直接操作串口设备, 配置与下位机/上位机 C++ 代码
(openSerialPort) 完全一致: 8N1, 无校验, 无流控, raw 模式。

波特率说明:
    macOS 的 Python termios 模块通常只导出到 B230400, 更高波特率
    (如固件使用的 2000000) 通过 IOSSIOSPEED ioctl 直接设置,
    与 pyserial 在 macOS 上的做法相同。
"""

import fcntl
import glob
import os
import struct
import sys
import termios

# macOS: 设置任意波特率的 ioctl (pyserial 同款)
IOSSIOSPEED = 0x80045402
# macOS: 读取/清除 Modem 控制线
TIOCMGET = 0x4004746D
TIOCMBIC = 0x80047467
TIOCM_DTR = 0x002
TIOCM_RTS = 0x004

# termios 中可用的标准波特率常量 (如 B9600=9600)
_STANDARD_RATES = sorted(
    (int(name[1:]), getattr(termios, name))
    for name in vars(termios)
    if name.startswith("B") and name[1:].isdigit()
)


class SerialPortError(Exception):
    pass


def list_ports():
    """列出 macOS 可用串口 (优先 cu.* 设备, 排除蓝牙/调试口)"""
    exclude = ("bluetooth", "debug", "wlan")
    ports = []
    for p in sorted(glob.glob("/dev/cu.*")):
        low = os.path.basename(p).lower()
        if not any(x in low for x in exclude):
            ports.append(p)
    return ports


class SerialPort:
    """极简串口: open / close / read / write / in_waiting"""

    def __init__(self, port, baudrate=2000000):
        self.port = port
        self.baudrate = int(baudrate)
        self.fd = -1
        self._orig_attrs = None

    # ------------------------------------------------------------------ 打开
    def open(self):
        if self.fd != -1:
            raise SerialPortError("串口已打开")
        try:
            # O_NONBLOCK: 非阻塞读, 配合 in_waiting 使用
            self.fd = os.open(self.port, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
        except OSError as e:
            raise SerialPortError(f"无法打开串口 {self.port}: {e.strerror} (可能被其他程序占用?)")

        try:
            self._orig_attrs = termios.tcgetattr(self.fd)
            attrs = termios.tcgetattr(self.fd)
            iflag, oflag, cflag, lflag, ispeed, ospeed, cc = attrs
            cc = list(cc)

            # 8N1, 无流控, 本地模式 (与 C++ openSerialPort 一致)
            cflag &= ~(termios.PARENB | termios.CSTOPB | termios.CSIZE | termios.CRTSCTS)
            cflag |= termios.CS8 | termios.CLOCAL | termios.CREAD
            iflag &= ~(termios.IGNBRK | termios.BRKINT | termios.PARMRK | termios.ISTRIP |
                       termios.INLCR | termios.IGNCR | termios.ICRNL |
                       termios.IXON | termios.IXOFF | termios.IXANY | termios.INPCK)
            oflag &= ~termios.OPOST
            lflag &= ~(termios.ICANON | termios.ECHO | termios.ECHOE |
                       termios.ECHONL | termios.ISIG | termios.IEXTEN)
            cc[termios.VMIN] = 0
            cc[termios.VTIME] = 0

            # 关闭串口时不挂断线路 (清 HUPCL, 关闭时不翻转 DTR)
            cflag &= ~termios.HUPCL

            # 先设为不超过目标波特率的标准值 (macOS 高波特率常量可能缺失)
            best = termios.B9600
            for rate, const in _STANDARD_RATES:
                if rate <= self.baudrate:
                    best = const
            ispeed = ospeed = best

            termios.tcsetattr(self.fd, termios.TCSANOW,
                              [iflag, oflag, cflag, lflag, ispeed, ospeed, cc])

            # macOS: 用 ioctl 设置精确波特率 (支持 2000000 等非标准值)
            if sys.platform == "darwin":
                fcntl.ioctl(self.fd, IOSSIOSPEED, struct.pack("I", self.baudrate))

            # 立即清除 DTR/RTS, 避免误触 ESP32 等板的自动下载/复位序列
            try:
                bits = struct.unpack(
                    "I", fcntl.ioctl(self.fd, TIOCMGET, struct.pack("I", 0)))[0]
                fcntl.ioctl(self.fd, TIOCMBIC,
                            struct.pack("I", bits & ~(TIOCM_DTR | TIOCM_RTS)))
            except OSError:
                pass

            termios.tcflush(self.fd, termios.TCIFLUSH)
        except Exception as e:
            try:
                os.close(self.fd)
            except OSError:
                pass
            self.fd = -1
            raise SerialPortError(f"配置串口失败: {e}")
        return self

    # ------------------------------------------------------------------ 关闭
    def close(self):
        if self.fd != -1:
            try:
                termios.tcflush(self.fd, termios.TCIOFLUSH)
                if self._orig_attrs is not None:
                    termios.tcsetattr(self.fd, termios.TCSANOW, self._orig_attrs)
            except Exception:
                pass
            try:
                os.close(self.fd)
            except OSError:
                pass
            self.fd = -1

    # ------------------------------------------------------------------ 读写
    def in_waiting(self):
        """串口接收缓冲区中的字节数"""
        try:
            return struct.unpack("I", fcntl.ioctl(
                self.fd, termios.FIONREAD, struct.pack("I", 0)))[0]
        except OSError:
            return 0

    def read(self, n=1):
        """非阻塞读, 无数据返回 b''"""
        try:
            return os.read(self.fd, max(1, n))
        except BlockingIOError:
            return b""
        except OSError as e:
            raise SerialPortError(f"读取串口失败: {e.strerror}")

    def write(self, data):
        try:
            return os.write(self.fd, data)
        except OSError as e:
            raise SerialPortError(f"写入串口失败: {e.strerror}")

    @property
    def is_open(self):
        return self.fd != -1


if __name__ == "__main__":
    print("可用串口:")
    for p in list_ports():
        print(" ", p)
