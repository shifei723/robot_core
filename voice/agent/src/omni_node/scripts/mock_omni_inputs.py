#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy
from std_msgs.msg import UInt8MultiArray, String
from sensor_msgs.msg import Image
import time
from PIL import Image as PILImage
import os

class MockInputNode(Node):
    def __init__(self):
        super().__init__("mock_omni_input")

        # 话题配置
        self.declare_parameter("audio_topic", "/asr/wav_audio")
        self.declare_parameter("image_topic", "/vision/image")
        self.audio_topic = self.get_parameter("audio_topic").value
        self.image_topic = self.get_parameter("image_topic").value

        # 图片QoS 和主节点一致
        img_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.VOLATILE,
            depth=1
        )

        # 发布器 & 订阅器
        self.audio_pub = self.create_publisher(UInt8MultiArray, self.audio_topic, 10)
        self.image_pub = self.create_publisher(Image, self.image_topic, qos_profile=img_qos)

        self.result_sub = self.create_subscription(
            String, "/omni/output_text", self.on_result, 10
        )
        self.state_sub = self.create_subscription(
            String, "/omni/node_state", self.on_state, 10
        )

        # 本地文件路径（当前scripts目录）
        self.script_dir = os.path.dirname(os.path.abspath(__file__))
        self.wav_path = os.path.join(self.script_dir, "hangzhou.wav")   # 选用 hangzhou.wav
        self.img_path = os.path.join(self.script_dir, "image.jpg")      # 选用 image.jpg

        self.get_logger().info(f"Mock input ready")
        self.get_logger().info(f"Audio topic: {self.audio_topic}")
        self.get_logger().info(f"Image topic: {self.image_topic}")
        self.get_logger().info(f"WAV file: {self.wav_path}")
        self.get_logger().info(f"Image file: {self.img_path}")

    def on_result(self, msg: String):
        self.get_logger().info(f"[RESULT] {msg.data}")

    def on_state(self, msg: String):
        self.get_logger().info(f"[STATE] {msg.data}")

    def load_wav(self) -> list:
        """读取本地wav为字节流"""
        if not os.path.exists(self.wav_path):
            self.get_logger().error(f"WAV file not found: {self.wav_path}")
            return []
        try:
            with open(self.wav_path, "rb") as f:
                return list(f.read())
        except Exception as e:
            self.get_logger().error(f"Read wav failed: {str(e)}")
            return []

    def load_jpg_to_ros_image(self) -> Image:
        """读取jpg图片 → 转为ROS Image(rgb8)，保证宽高为偶数"""
        ros_img = Image()
        ros_img.header.stamp = self.get_clock().now().to_msg()

        try:
            pil_img = PILImage.open(self.img_path).convert("RGB")
            w, h = pil_img.size

            # 确保宽高是偶数（NV12要求）
            if w % 2 != 0:
                w -= 1
            if h % 2 != 0:
                h -= 1
            pil_img = pil_img.resize((w, h))

            ros_img.width = w
            ros_img.height = h
            ros_img.encoding = "rgb8"
            ros_img.is_bigendian = 0
            ros_img.step = w * 3
            ros_img.data = list(pil_img.tobytes())
            return ros_img
        except Exception as e:
            self.get_logger().error(f"Read image failed: {str(e)}")
            return Image()

    def send_once(self):
        # 加载音频
        wav_data = self.load_wav()
        if not wav_data:
            self.get_logger().error("Empty wav data, exit send")
            return

        # 加载图片
        ros_img = self.load_jpg_to_ros_image()
        if ros_img.width == 0 or ros_img.height == 0:
            self.get_logger().error("Image load failed")
            return

        # 先发音频
        audio_msg = UInt8MultiArray()
        audio_msg.data = wav_data
        self.audio_pub.publish(audio_msg)
        self.get_logger().info("Published audio(hangzhou.wav)")
        time.sleep(0.2)

        # 再发图片
        self.image_pub.publish(ros_img)
        self.get_logger().info("Published image(image.jpg), waiting inference...")

def main(args=None):
    rclpy.init(args=args)
    mock_node = MockInputNode()

    # 发送一组测试数据
    mock_node.send_once()

    try:
        rclpy.spin(mock_node)
    except KeyboardInterrupt:
        mock_node.get_logger().info("Exit mock input")
    finally:
        mock_node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()