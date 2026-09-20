#!/usr/bin/env python3
"""
KWS 唤醒 → Omni 推理桥接节点

功能：
1. 轮询 KWS 状态文件（commands/kws_status），检测状态下降沿（1 → 0）。
   - 状态 = 1：唤醒词检测到，VAD 正在录音，音频尚未完成
   - 状态 = 0（从 1 回到 0）：VAD 结束，命令音频已保存，此时触发推送
2. 读取 commands/latest_command.wav 作为命令音频。
3. 订阅相机图像话题，始终缓存最新一帧。
4. 将音频（UInt8MultiArray）和图像（sensor_msgs/Image）按序推送到 omni 节点对应的话题，
   触发多模态推理。

用法：
    ros2 run omni_node transformVL.py
    或
    python3 transformVL.py

话题：
    订阅: /vision/image  (sensor_msgs/Image)
    发布: /asr/wav_audio (std_msgs/UInt8MultiArray)
          /vision/image  (sensor_msgs/Image)  —— 转发给 omni 节点
"""

import os
import time
import threading

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import UInt8MultiArray, String


class TransformVLNode(Node):
    """监听 KWS 状态文件，唤醒后将最新命令音频 + 缓存图像推送给 omni 节点。"""

    def __init__(self):
        super().__init__("transform_vl_node")

        # ── 参数声明 ──────────────────────────────────────────────
        self.declare_parameter("audio_topic", "/asr/wav_audio")
        self.declare_parameter("image_topic", "/vision/image")
        self.declare_parameter(
            "commands_dir",
            os.path.join(
                os.path.expanduser("~"),
                "sherpa-onnx-kws", "commands",
            ),
        )
        self.declare_parameter("status_filename", "kws_status")
        self.declare_parameter("wav_filename", "latest_command.wav")
        self.declare_parameter("poll_interval", 0.1)  # 状态轮询间隔（秒）
        self.declare_parameter("interrupt_topic", "/robot/interrupt")

        self.audio_topic = self.get_parameter("audio_topic").value
        self.image_topic = self.get_parameter("image_topic").value
        self.commands_dir = self.get_parameter("commands_dir").value
        self.status_file = os.path.join(
            self.commands_dir, self.get_parameter("status_filename").value
        )
        self.wav_file = os.path.join(
            self.commands_dir, self.get_parameter("wav_filename").value
        )
        self.poll_interval = self.get_parameter("poll_interval").value
        self.interrupt_topic = self.get_parameter("interrupt_topic").value

        # ── QoS（与 omni 节点保持一致）─────────────────────────────
        img_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.VOLATILE,
            depth=1,
        )

        # ── 发布器 ────────────────────────────────────────────────
        self.audio_pub = self.create_publisher(
            UInt8MultiArray, self.audio_topic, 10
        )
        self.image_pub = self.create_publisher(
            Image, self.image_topic, qos_profile=img_qos
        )
        # 唤醒打断信号: 唤醒词一被识别（上升沿 0→1）就立即广播，
        # 不等 VAD 录完，让 TTS 停口、正在执行的任务中止
        self.interrupt_pub = self.create_publisher(String, self.interrupt_topic, 10)

        # ── 订阅相机图像（缓存最新帧）──────────────────────────────
        self.latest_image: Image | None = None
        self.image_lock = threading.Lock()
        self.image_sub = self.create_subscription(
            Image,
            self.image_topic,
            self._on_image,
            qos_profile=img_qos,
        )

        # ── KWS 状态追踪 ─────────────────────────────────────────
        self._prev_status = 0          # 上一次读到的状态值
        self._wav_mtime_sent = 0.0     # 上次已发送的 wav 文件修改时间

        # ── 轮询定时器 ───────────────────────────────────────────
        self.poll_timer = self.create_timer(self.poll_interval, self._poll_status)

        self.get_logger().info(
            f"TransformVL ready | audio={self.audio_topic} image={self.image_topic}"
        )
        self.get_logger().info(f"Status file : {self.status_file}")
        self.get_logger().info(f"WAV file    : {self.wav_file}")
        self.get_logger().info(f"Poll interval: {self.poll_interval}s")

    # ──────────────────────────────────────────────────────────────
    # 图像回调：只缓存最新帧
    # ──────────────────────────────────────────────────────────────
    def _on_image(self, msg: Image):
        with self.image_lock:
            self.latest_image = msg

    # ──────────────────────────────────────────────────────────────
    # 状态文件轮询
    # ──────────────────────────────────────────────────────────────
    def _read_status(self) -> int:
        """读取状态文件，返回 0 或 1；文件不存在或读取出错返回 0。"""
        try:
            with open(self.status_file, "r") as f:
                val = f.read().strip()
            return 1 if val == "1" else 0
        except (FileNotFoundError, OSError):
            return 0

    def _poll_status(self):
        """定时器回调：上升沿发打断，下降沿推送命令。"""
        current = self._read_status()

        # 0 → 1 上升沿：唤醒词刚被识别，立即打断当前播报与任务
        if current == 1 and self._prev_status == 0:
            self.get_logger().info("[KWS] 检测到唤醒词 → 广播打断信号")
            self.interrupt_pub.publish(String(data="wakeword"))

        # 只在 1 → 0 的下降沿触发推送：此时命令音频已完整写入 latest_command.wav
        if current == 0 and self._prev_status == 1:
            self.get_logger().info("[KWS] VAD 结束，命令音频已就绪，准备推送")
            self._trigger_send()

        self._prev_status = current

    # ──────────────────────────────────────────────────────────────
    # 推送音频 + 图像
    # ──────────────────────────────────────────────────────────────
    def _trigger_send(self):
        """读取最新 WAV 并连同缓存图像一起发布。"""
        # 1. 读取 WAV 文件
        wav_bytes = self._load_wav()
        if not wav_bytes:
            self.get_logger().warn("WAV 文件为空或不存在，跳过本次推送")
            return

        # 2. 获取缓存图像
        with self.image_lock:
            img = self.latest_image

        if img is None:
            self.get_logger().warn("尚未收到图像帧，跳过本次推送")
            return

        # 3. 先发布音频
        audio_msg = UInt8MultiArray()
        audio_msg.data = wav_bytes
        self.audio_pub.publish(audio_msg)
        self.get_logger().info(
            f"[SEND] 音频已发布 ({len(wav_bytes)} bytes)"
        )

        # 短暂延时，让 omni 节点有时间处理音频到达
        time.sleep(0.05)

        # 4. 再发布图像
        self.image_pub.publish(img)
        self.get_logger().info(
            f"[SEND] 图像已发布 ({img.width}x{img.height}, {img.encoding})"
        )
        self.get_logger().info("[SEND] 音频+图像已推送至 omni 节点，等待推理...")

    def _load_wav(self) -> list[int]:
        """读取 WAV 文件为字节列表，失败返回空列表。"""
        if not os.path.exists(self.wav_file):
            self.get_logger().error(f"WAV 文件不存在: {self.wav_file}")
            return []
        try:
            # 记录文件修改时间，防止重复发送同一份文件
            mtime = os.path.getmtime(self.wav_file)
            if mtime <= self._wav_mtime_sent:
                self.get_logger().info("WAV 文件未更新，跳过")
                return []
            self._wav_mtime_sent = mtime

            with open(self.wav_file, "rb") as f:
                data = list(f.read())
            self.get_logger().info(f"读取 WAV: {self.wav_file} ({len(data)} bytes)")
            return data
        except Exception as e:
            self.get_logger().error(f"读取 WAV 失败: {e}")
            return []


def main(args=None):
    rclpy.init(args=args)
    node = TransformVLNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("TransformVL 退出")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
