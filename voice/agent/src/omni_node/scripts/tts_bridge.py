#!/usr/bin/env python3
"""
Omni 推理结果 → TTS 桥接节点（支持双后端切换）

功能：
1. 订阅 /omni/output_text（omni_node 的流式推理文本，按标点分段）。
2. 剥离 "[sid:x] " 前缀后，根据 backend 参数转发给对应 TTS：
   - matcha：ZMQ PUB → tts_stream（sherpa-onnx Matcha，22kHz）
   - hobot：发布 /tts_text 话题 → hobot_tts（WeTTS vits，16kHz）

架构：
    /omni/output_text (ROS2) --> tts_bridge --+--> tcp://127.0.0.1:5555 (ZMQ) --> tts_stream
                                              +--> /tts_text (ROS2) --> hobot_tts

用法：
    python3 tts_bridge.py                                    # 默认 matcha
    python3 tts_bridge.py --ros-args -p backend:=hobot       # 切 hobot_tts

注意：matcha 后端时本节点作为 ZMQ PUB 端 bind 地址，tts_stream 作为 SUB 端 connect，
      两者的启动顺序不影响（ZMQ 自动重连），但 PUB 端只能有一个。
"""

import re
import subprocess
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

import zmq

# omni_node 输出的文本前缀格式: "[sid:123] 你好..."
SID_PREFIX_RE = re.compile(r"^\[sid:\d+\]\s*")
# 工具调用 JSON 不能当成话术读出来，由 tool_agent 负责执行
TOOLCALL_HINT_RE = re.compile(r'[{}]|"tasks"|"action"')


def _unmute_all_tts():
    """启动时解除残留的静音。

    早期版本用 pactl 静音来做打断，若进程异常退出可能把流留在静音状态，
    而 PulseAudio 的 stream-restore 会记住该设置，导致下次启动仍无声。
    现在打断走 /tts_stop 真正取消，不再静音，但启动时清一次历史状态。
    """
    try:
        out = subprocess.run(["pactl", "list", "short", "sink-inputs"],
                             capture_output=True, text=True, timeout=3).stdout
        for line in out.splitlines():
            idx = line.split("\t")[0].strip()
            if idx.isdigit():
                subprocess.run(["pactl", "set-sink-input-mute", idx, "0"],
                               timeout=3, check=False)
    except (OSError, subprocess.SubprocessError):
        pass


class TtsBridgeNode(Node):
    """订阅 omni 推理文本，转发到 tts_stream 的 ZMQ 端口。"""

    def __init__(self):
        super().__init__("tts_bridge_node")

        # ── 参数声明 ──────────────────────────────────────────────
        self.declare_parameter("text_topic", "/omni/output_text")
        self.declare_parameter("speech_topic", "/robot/speech")
        self.declare_parameter("interrupt_topic", "/robot/interrupt")
        self.declare_parameter("state_topic", "/omni/node_state")
        self.declare_parameter("backend", "matcha")  # matcha=tts_stream / hobot=hobot_tts
        self.declare_parameter("zmq_addr", "tcp://127.0.0.1:5555")
        self.declare_parameter("hobot_topic", "/tts_text")
        self.declare_parameter("tts_stop_topic", "/tts_stop")
        # 唤醒应答语：收到打断（=唤醒词被识别）后说一句，置空则禁用。
        # 必须配合 kws_vad_demo 的 --ack-blank-ms，否则无 AEC 的设备会把
        # 这句应答当成用户命令录进去。
        self.declare_parameter("wake_ack_text", "我在")
        # 应答延迟：要等 /tts_stop 先被 hobot_tts 处理完，
        # 否则应答可能反过来被那次清空连带丢掉
        self.declare_parameter("wake_ack_delay", 0.3)
        self.declare_parameter("min_length", 2)  # 过滤纯标点/过短的碎片文本
        # 打断后最长静音时长：超过它没有新推理就自动恢复。
        # 必要的兼底：用户唤醒后却没说话时 VAD 会超时，latest_command.wav
        # 未更新，transformVL 就不会推送，也就永远等不到 "Audio received"，
        # 若无此兼底，TTS 会被永久静音。
        self.declare_parameter("unmute_timeout", 20.0)

        self.text_topic = self.get_parameter("text_topic").value
        self.speech_topic = self.get_parameter("speech_topic").value
        self.interrupt_topic = self.get_parameter("interrupt_topic").value
        self.state_topic = self.get_parameter("state_topic").value
        self.backend = self.get_parameter("backend").value
        self.zmq_addr = self.get_parameter("zmq_addr").value
        self.hobot_topic = self.get_parameter("hobot_topic").value
        self.tts_stop_topic = self.get_parameter("tts_stop_topic").value
        self.wake_ack_text = self.get_parameter("wake_ack_text").value
        self.wake_ack_delay = self.get_parameter("wake_ack_delay").value
        self.min_length = self.get_parameter("min_length").value
        self.unmute_timeout = self.get_parameter("unmute_timeout").value

        # ── 按后端初始化发送通道 ────────────────────────
        self.zmq_ctx = None
        self.zmq_sock = None
        self.hobot_pub = None
        self.stop_pub = None
        if self.backend == "hobot":
            self.hobot_pub = self.create_publisher(String, self.hobot_topic, 10)
            # 打断时通知 hobot_tts 真正丢弃队列（需改造版 hobot_tts）
            self.stop_pub = self.create_publisher(
                String, self.tts_stop_topic, 10)
        else:
            # 默认 matcha：ZMQ PUB（bind 端，tts_stream connect 过来）
            self.zmq_ctx = zmq.Context()
            self.zmq_sock = self.zmq_ctx.socket(zmq.PUB)
            self.zmq_sock.bind(self.zmq_addr)

        # ── 订阅推理文本 ─────────────────────────────────────────
        self.text_sub = self.create_subscription(
            String, self.text_topic, self._on_text, 10
        )
        # ── 订阅机器人主动播报（tool_agent 的任务进度反馈）──────
        # 这路不做 JSON 过滤，因为内容本身就是现成的自然语句
        self.speech_sub = self.create_subscription(
            String, self.speech_topic, self._on_speech, 10
        )
        # ── 唤醒打断 ───────────────────────────────────────
        # 收到打断后: 立即静音音频流 + 关闭文本闸门（丢弃本轮剩下的文本），
        # 下一轮推理开始（收到新音频）时自动恢复。
        # 之所以能安全恢复：从打断到下一轮回答至少隔着一次 VAD 录音+推理
        # (数秒)，远长于 TTS 队列里残留音频的长度，旧音频已在静音下消耗完。
        self._gate_open = True
        self._interrupt_at = None       # 打断时刻，用于超时自动恢复
        self._ack_timer = None          # 唤醒应答的一次性定时器
        self.interrupt_sub = self.create_subscription(
            String, self.interrupt_topic, self._on_interrupt, 10
        )
        self.state_sub = self.create_subscription(
            String, self.state_topic, self._on_state, 10
        )
        self.create_timer(1.0, self._check_unmute_timeout)
        # 清除可能残留的历史静音状态
        _unmute_all_tts()

        self.sent_count = 0
        dest = self.hobot_topic if self.backend == "hobot" else f"ZMQ {self.zmq_addr}"
        self.get_logger().info(
            f"TtsBridge ready | backend={self.backend} | "
            f"{self.text_topic} + {self.speech_topic} -> {dest}"
        )

    def _on_interrupt(self, msg: String):
        # 1. 真正取消: 通知 hobot_tts 丢弃全部未播内容并中断当前播放
        #    （不能只静音：静音只是听不见，队列里的内容仍在，
        #      解除静音后会接着念旧内容）
        if self.stop_pub is not None:
            self.stop_pub.publish(String(data=msg.data))
        # 2. 关闭文本闸门: 丢弃本轮推理剩下的文本，不再往 TTS 送
        self._gate_open = False
        self._interrupt_at = time.time()
        self.get_logger().info(
            f"[打断] 来源={msg.data} → 已通知 TTS 清空队列并关闭文本闸门")

        # 3. 唤醒应答“我在”：延迟发，确保排在清空指令之后
        if self.wake_ack_text and msg.data == "wakeword":
            self._schedule_ack()

    def _schedule_ack(self):
        """延迟发一句唤醒应答（一次性定时器）。"""
        if self._ack_timer is not None:
            self._ack_timer.cancel()
        self._ack_timer = self.create_timer(self.wake_ack_delay, self._fire_ack)

    def _fire_ack(self):
        if self._ack_timer is not None:
            self._ack_timer.cancel()
            self._ack_timer = None
        # 绕过闸门直发：此时闸门正关着，但应答必须出声
        self._deliver(self.wake_ack_text)
        self.get_logger().info(f"[唤醒应答] {self.wake_ack_text}")

    def _reopen(self, reason: str):
        self._gate_open = True
        self._interrupt_at = None
        self.get_logger().info(f"[打断] {reason}，恢复转发")

    def _check_unmute_timeout(self):
        """兼底: 打断后过久没有新推理就重新开闸，避免永久不出声。

        必要的兼底：用户唤醒后却没说话时 VAD 会超时，latest_command.wav
        未更新，transformVL 就不会推送，也就永远等不到 "Audio received"。
        """
        if self._interrupt_at is None:
            return
        if time.time() - self._interrupt_at > self.unmute_timeout:
            self._reopen(f"打断后 {self.unmute_timeout:.0f}s 无新推理")

    def _on_state(self, msg: String):
        # 新一轮推理开始 → 打开闸门并取消静音
        if "Audio received" in msg.data and not self._gate_open:
            self._reopen("新一轮推理开始")

    def _on_text(self, msg: String):
        # 剥离 [sid:x] 前缀，再去掉首尾标点外的空白
        text = SID_PREFIX_RE.sub("", msg.data).strip()
        # 纯标点或过短碎片不发音
        if len(text) < self.min_length or not re.search(r"[\w\u4e00-\u9fff]", text):
            return
        # 工具调用 JSON 不读（避免把大括号、tasks 字段当成话念出来）
        if TOOLCALL_HINT_RE.search(text):
            self.get_logger().info(f"[跳过工具调用] {text[:60]}")
            return
        self._send(text)

    def _on_speech(self, msg: String):
        text = msg.data.strip()
        if not text:
            return
        self._send(text)

    def _send(self, text: str):
        if not self._gate_open:
            self.get_logger().info(f"[已打断丢弃] {text[:40]}")
            return
        self._deliver(text)

    def _deliver(self, text: str):
        """真正送往 TTS 后端（不看闸门，供唤醒应答使用）。"""
        if self.backend == "hobot":
            self.hobot_pub.publish(String(data=text))
        else:
            self.zmq_sock.send_string(text)
        self.sent_count += 1
        self.get_logger().info(f"[TTS#{self.sent_count}] {text}")


def main(args=None):
    rclpy.init(args=args)
    node = TtsBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("TtsBridge 退出")
    finally:
        if node.zmq_sock is not None:
            node.zmq_sock.close()
            node.zmq_ctx.term()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
