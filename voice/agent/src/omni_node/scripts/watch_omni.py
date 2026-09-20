#!/usr/bin/env python3
"""
Omni 推理结果实时查看器

功能：
1. 订阅 /omni/output_text，把同一轮对话（相同 sid）的流式分段聚合成完整回答。
2. 订阅 /omni/node_state，显示推理进度（收到音频/图像匹配/推理中/完成）。
3. 终端彩色输出 + 追加保存到 /tmp/pipeline_logs/omni_results.txt。

用法（前台运行，Ctrl+C 退出）：
    python3 watch_omni.py
    或
    ros2 run omni_node watch_omni.py
"""

import re
from datetime import datetime

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

# omni_node 输出格式: "[sid:123] 文本分段"
SID_RE = re.compile(r"^\[sid:(\d+)\]\s*(.*)", re.S)

RESULT_FILE = "/tmp/pipeline_logs/omni_results.txt"

# 终端颜色
C_CYAN = "\033[36m"
C_GREEN = "\033[32m"
C_YELLOW = "\033[33m"
C_BOLD = "\033[1m"
C_END = "\033[0m"


class WatchOmniNode(Node):
    """聚合并展示 omni 推理结果。"""

    def __init__(self):
        super().__init__("watch_omni_node")

        self.declare_parameter("text_topic", "/omni/output_text")
        self.declare_parameter("state_topic", "/omni/node_state")

        text_topic = self.get_parameter("text_topic").value
        state_topic = self.get_parameter("state_topic").value

        self.text_sub = self.create_subscription(
            String, text_topic, self._on_text, 10
        )
        self.state_sub = self.create_subscription(
            String, state_topic, self._on_state, 10
        )

        self.cur_sid = None    # 当前正在接收的会话 id
        self.cur_parts = []    # 当前会话已收到的分段

        print(f"{C_BOLD}══════ Omni 推理结果监控 ══════{C_END}")
        print(f"订阅: {text_topic} / {state_topic}")
        print(f"结果存档: {RESULT_FILE}")
        print("等待推理结果...（说 \"你好小飞\" 唤醒后提问）\n")

    # ── 状态消息：显示链路进度 ────────────────────────────────
    def _on_state(self, msg: String):
        state = msg.data
        ts = datetime.now().strftime("%H:%M:%S")
        if "Audio received" in state:
            print(f"{C_YELLOW}[{ts}] ▶ 收到命令音频，等待图像配对...{C_END}")
        elif "Image matched" in state:
            print(f"{C_YELLOW}[{ts}] ▶ 图像已配对，进入推理队列{C_END}")
        elif "running inference" in state:
            print(f"{C_YELLOW}[{ts}] ▶ 推理中...{C_END}")
        elif "Infer finished" in state:
            self._flush_answer()
        elif "timeout" in state or "failed" in state:
            print(f"{C_YELLOW}[{ts}] ▶ {state}{C_END}")

    # ── 文本消息：按 sid 聚合流式分段 ─────────────────────────
    def _on_text(self, msg: String):
        m = SID_RE.match(msg.data)
        if m:
            sid, piece = m.group(1), m.group(2)
        else:
            sid, piece = "?", msg.data

        # 新一轮对话开始
        if sid != self.cur_sid:
            self._flush_answer()
            self.cur_sid = sid
            self.cur_parts = []
            ts = datetime.now().strftime("%H:%M:%S")
            print(f"{C_BOLD}[{ts}] ── 第 {sid} 轮回答 ──{C_END}")

        self.cur_parts.append(piece)
        # 流式实时打印分段
        print(f"  {C_CYAN}{piece}{C_END}", flush=True)

    # ── 一轮结束：输出完整回答并存档 ──────────────────────────
    def _flush_answer(self):
        if self.cur_sid is None or not self.cur_parts:
            return
        full = "".join(self.cur_parts)
        ts = datetime.now().strftime("%F %T")
        print(f"{C_GREEN}{C_BOLD}[完整回答 sid:{self.cur_sid}] {full}{C_END}\n")
        try:
            with open(RESULT_FILE, "a") as f:
                f.write(f"[{ts}] [sid:{self.cur_sid}] {full}\n")
        except OSError:
            pass
        self.cur_sid = None
        self.cur_parts = []


def main(args=None):
    rclpy.init(args=args)
    node = WatchOmniNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        node._flush_answer()
        print("\n退出监控")
    finally:
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    main()
