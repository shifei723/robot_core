#!/usr/bin/env python3
"""probe_toolcall.py — 注入工具调用 JSON，验证 tool_agent 的多任务顺序执行

仅用于联调，不参与正式链路运行。
"""
import os
import sys
import threading
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import String

JSON_CMD = ('{"tasks":[{"action":"navigate","target":"kitchen"},'
            '{"action":"nod"},'
            '{"action":"navigate","target":"卧室"},'
            '{"action":"shake"}]}')


class Probe(Node):
    def __init__(self):
        super().__init__("toolcall_probe")
        self.text_pub = self.create_publisher(String, "/omni/output_text", 10)
        self.state_pub = self.create_publisher(String, "/omni/node_state", 10)
        self.speech = []
        self.att = []
        self.create_subscription(String, "/robot/speech", self._on_speech, 10)
        self.create_subscription(JointState, "/cmd_posture", self._on_att, 50)

    def _on_speech(self, m):
        self.speech.append(m.data)
        print(f"[{time.strftime('%H:%M:%S')}] [播报] {m.data}", flush=True)

    def _on_att(self, m):
        # position = [高度(米), roll(度), pitch(度)]
        if len(m.position) >= 3:
            self.att.append((round(m.position[1], 3), round(m.position[2], 3)))


def main():
    rclpy.init()
    node = Probe()
    threading.Thread(target=rclpy.spin, args=(node,), daemon=True).start()

    # 等 tool_agent 与 tts_bridge 两个订阅者都被发现，否则可能发早了消息丢失
    for _ in range(100):
        if node.text_pub.get_subscription_count() >= 2:
            break
        time.sleep(0.2)
    cnt = node.text_pub.get_subscription_count()
    print(f"订阅 /omni/output_text 的节点数: {cnt} (期望 2: tool_agent + tts_bridge)")
    if cnt < 2:
        print("警告: 订阅者不足，测试结果可能不准")

    print("\n>>> 注入指令: 去厨房 → 点头 → 去卧室 → 摇头\n")
    node.text_pub.publish(String(data=f"[sid:1] {JSON_CMD}"))
    time.sleep(0.5)
    node.state_pub.publish(String(data="Infer finished"))

    time.sleep(22)   # 3s导航+点头2s+3s导航+摇头2s ≈ 10s，留足余量

    rolls = [a[0] for a in node.att]
    pitches = [a[1] for a in node.att]
    print("\n═══ 姿态指令统计 (/cmd_posture) ═══")
    print(f"总轨迹点数: {len(node.att)}")
    if pitches:
        print(f"pitch(点头) 范围: {min(pitches):+.2f}° ~ {max(pitches):+.2f}°")
    if rolls:
        print(f"roll(摇头)  范围: {min(rolls):+.2f}° ~ {max(rolls):+.2f}°")
    if node.att:
        print(f"最后一帧回中位: {node.att[-1] == (0.0, 0.0)}")
    print(f"播报条数: {len(node.speech)}")
    # 直接退出: rclpy.spin 还在守护线程里，调 shutdown 会让它抛异常并触发 abort
    sys.stdout.flush()
    os._exit(0)


if __name__ == "__main__":
    main()
