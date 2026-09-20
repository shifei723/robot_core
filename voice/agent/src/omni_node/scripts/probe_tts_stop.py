#!/usr/bin/env python3
"""probe_tts_stop.py — 验证 hobot_tts 的真实打断（而非静音）

流程: 发 5 句长内容 → 播 4 秒 → 发 /tts_stop → 再发一句新内容
预期: 打断日志出现且 dropped_pcm>0；之后念的是新内容，不接着念旧的。
"""
import os
import sys
import threading
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class Pub(Node):
    def __init__(self):
        super().__init__("tts_stop_probe")
        self.text = self.create_publisher(String, "/tts_text", 10)
        self.stop = self.create_publisher(String, "/tts_stop", 10)

    def wait_ready(self, timeout=20.0):
        end = time.time() + timeout
        while time.time() < end:
            if (self.text.get_subscription_count() > 0
                    and self.stop.get_subscription_count() > 0):
                return True
            time.sleep(0.2)
        return False


def main():
    rclpy.init()
    node = Pub()
    threading.Thread(target=rclpy.spin, args=(node,), daemon=True).start()

    if not node.wait_ready():
        print(f"hobot_tts 未就绪 (text订阅={node.text.get_subscription_count()} "
              f"stop订阅={node.stop.get_subscription_count()})")
        os._exit(1)
    print(f"hobot_tts 已就绪 (text/stop 订阅者均 >0)")

    print("\n>>> 发送 5 句长内容")
    for i in range(1, 6):
        node.text.publish(String(
            data=f"第{i}句，这是一段用来测试打断功能的比较长的语音内容，请注意听。"))
        time.sleep(0.15)

    print(">>> 等待 4 秒（让它开始播放）")
    time.sleep(4)

    print("\n>>> 发送打断 /tts_stop")
    node.stop.publish(String(data="wakeword"))
    time.sleep(3)

    print("\n>>> 打断后发送新内容")
    node.text.publish(String(data="打断成功，这是全新的内容。"))
    time.sleep(7)

    print("\n完成，请查看 hobot_tts 日志")
    sys.stdout.flush()
    os._exit(0)


if __name__ == "__main__":
    main()
