#!/usr/bin/env python3
"""
相机图像发布节点

功能：
1. 通过 OpenCV 打开 USB 相机（/dev/video0），周期性采集图像。
2. 转换为 sensor_msgs/Image (rgb8) 发布到 /vision/image。
3. 保证宽高为偶数（omni 节点 NV12 转换要求）。

用法：
    ros2 run omni_node camera_pub.py
    或
    python3 camera_pub.py
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy
from sensor_msgs.msg import Image

import cv2


class CameraPubNode(Node):
    """USB 相机采集并发布 /vision/image（rgb8，宽高偶数）。"""

    def __init__(self):
        super().__init__("camera_pub_node")

        # ── 参数声明 ──────────────────────────────────────────────
        self.declare_parameter("image_topic", "/vision/image")
        self.declare_parameter("camera_index", 0)
        self.declare_parameter("width", 640)
        self.declare_parameter("height", 480)
        self.declare_parameter("fps", 2.0)  # 发布频率（推理只取最新帧，无需高帧率）

        self.image_topic = self.get_parameter("image_topic").value
        self.camera_index = self.get_parameter("camera_index").value
        self.width = self.get_parameter("width").value
        self.height = self.get_parameter("height").value
        self.fps = self.get_parameter("fps").value

        # ── QoS（与 omni 节点保持一致）─────────────────────────────
        img_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.VOLATILE,
            depth=1,
        )
        self.image_pub = self.create_publisher(Image, self.image_topic, img_qos)

        # ── 打开相机 ─────────────────────────────────────────────
        self.cap = cv2.VideoCapture(self.camera_index)
        if not self.cap.isOpened():
            raise RuntimeError(f"无法打开相机 /dev/video{self.camera_index}")
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)

        self.timer = self.create_timer(1.0 / self.fps, self._capture_and_publish)
        self.frame_count = 0

        self.get_logger().info(
            f"CameraPub ready | /dev/video{self.camera_index} "
            f"{self.width}x{self.height} @{self.fps}Hz -> {self.image_topic}"
        )

    def _capture_and_publish(self):
        ok, frame = self.cap.read()
        if not ok:
            self.get_logger().warn("相机读帧失败")
            return

        # BGR -> RGB，并保证宽高为偶数（NV12 要求）
        frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        h, w = frame.shape[:2]
        w -= w % 2
        h -= h % 2
        frame = frame[:h, :w]

        msg = Image()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "camera"
        msg.width = w
        msg.height = h
        msg.encoding = "rgb8"
        msg.is_bigendian = 0
        msg.step = w * 3
        msg.data = frame.tobytes()
        self.image_pub.publish(msg)

        self.frame_count += 1
        if self.frame_count % 20 == 1:
            self.get_logger().info(f"已发布 {self.frame_count} 帧 ({w}x{h})")


def main(args=None):
    rclpy.init(args=args)
    try:
        node = CameraPubNode()
    except RuntimeError as e:
        print(f"[ERROR] {e}")
        rclpy.shutdown()
        return
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("CameraPub 退出")
    finally:
        node.cap.release()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
