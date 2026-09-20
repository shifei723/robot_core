#!/usr/bin/env python3
"""probe_interrupt.py — 验证唤醒打断：任务执行中途发打断，检查是否及时停下

场景：注入 4 个任务（导航→点头→导航→摇头），在点头进行到一半时发打断信号，
应观察到：动作提前回中位、后续任务被丢弃、不再播报"全部任务已完成"。
"""
import os
import sys
import threading
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import String

JSON_CMD = ('{"tasks":[{"action":"nod"},'
            '{"action":"shake"},'
            '{"action":"navigate","target":"kitchen"}]}')


class Probe(Node):
    def __init__(self):
        super().__init__("interrupt_probe")
        self.text_pub = self.create_publisher(String, "/omni/output_text", 10)
        self.state_pub = self.create_publisher(String, "/omni/node_state", 10)
        self.intr_pub = self.create_publisher(String, "/robot/interrupt", 10)
        self.speech = []
        self.att = []
        self.create_subscription(String, "/robot/speech", self._on_speech, 10)
        self.create_subscription(JointState, "/cmd_posture", self._on_att, 50)

    def _on_speech(self, m):
        self.speech.append((time.time(), m.data))
        print(f"[{time.strftime('%H:%M:%S')}] [播报] {m.data}", flush=True)

    def _on_att(self, m):
        if len(m.position) >= 3:
            self.att.append((time.time(), round(m.position[1], 3),
                             round(m.position[2], 3)))


def main():
    rclpy.init()
    node = Probe()
    threading.Thread(target=rclpy.spin, args=(node,), daemon=True).start()

    for _ in range(100):
        if node.text_pub.get_subscription_count() >= 2:
            break
        time.sleep(0.2)
    print(f"订阅者数: {node.text_pub.get_subscription_count()}")

    print("\n>>> 注入 3 个任务: 点头 → 摇头 → 去厨房\n")
    node.text_pub.publish(String(data=f"[sid:1] {JSON_CMD}"))
    time.sleep(0.5)
    node.state_pub.publish(String(data="Infer finished"))

    # 等点头动作跑起来（约 1 秒后处于动作中段）
    time.sleep(2.0)
    t_intr = time.time()
    print(f"\n>>> [{time.strftime('%H:%M:%S')}] 发送唤醒打断信号\n")
    node.intr_pub.publish(String(data="wakeword"))

    time.sleep(8)   # 若打断失效，剩余任务会继续跑

    print("\n═══ 打断效果 ═══")
    after = [a for a in node.att if a[0] > t_intr]
    print(f"打断后仍发出的姿态帧: {len(after)} 帧 "
          f"(应很少，仅回中位的几帧)")
    if after:
        last = after[-1]
        print(f"最后一帧: roll={last[1]:+.2f}° pitch={last[2]:+.2f}° "
              f"(应为 0/0)")
        print(f"末帧距打断: {last[0]-t_intr:.2f}s (应 <1s)")
    sp_after = [s for _, s in [(t, d) for t, d in node.speech if t > t_intr]]
    print(f"打断后的播报: {sp_after if sp_after else '(无，符合预期)'}")
    done = any("全部任务已完成" in s for s in sp_after)
    print(f"是否误报'全部任务已完成': {'是(BUG)' if done else '否(正确)'}")
    sys.stdout.flush()
    os._exit(0)


if __name__ == "__main__":
    main()
