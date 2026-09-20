#!/usr/bin/env python3
"""
腿高指令 → 真机底盘姿态指令 桥接节点
=====================================
规划器侧 closed_loop_controller 把机体高度以 leg_height_cmd (Float32, 米) 发出，
而真机底盘 wl_base_node 只认 /cmd_posture (sensor_msgs/JointState)。
仿真里这一段由 slash_leg_controller.py 承接，真机模式下它不启动
(run.launch.py 的 `if not is_real`)，所以需要本节点补位。

底盘侧的三个约束决定了这里的实现方式:
  1. name 必须同时含 joint_height / joint_roll / joint_pitching,
     缺一个整条消息会被 joint_state_callback 直接丢弃;
  2. leg_length 会被 clamp 到 [0.14, 0.36], 超出部分静默截断,
     所以在本节点就钳到同一区间, 避免规划器以为高度已生效;
  3. 底盘有 1 秒无指令自动回中位的保护, 因此必须定时重发最新值,
     只在数值变化时发会让机器人周期性起落。
"""

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32

# 与 wl_base_node.cpp 的 LEG_MIN_LOW / LEG_MAX_HIGH 保持一致
CHASSIS_HEIGHT_MIN = 0.14
CHASSIS_HEIGHT_MAX = 0.36

JOINT_NAMES = ["joint_height", "joint_roll", "joint_pitching"]


class LegHeightToPosture(Node):
    def __init__(self):
        super().__init__("leg_height_to_posture")

        self.rate = self.declare_parameter("rate", 20.0).value
        self.height_min = self.declare_parameter("height_min", CHASSIS_HEIGHT_MIN).value
        self.height_max = self.declare_parameter("height_max", CHASSIS_HEIGHT_MAX).value

        # 收到首个指令前保持 None, 不发布任何姿态,
        # 免得节点一起来就把机器人从当前姿态强行拉走
        self.target_h = None
        self.clamped_warned = False

        self.height_sub = self.create_subscription(
            Float32, "leg_height_cmd", self._height_callback, 10)
        self.posture_pub = self.create_publisher(JointState, "/cmd_posture", 10)
        self.timer = self.create_timer(1.0 / self.rate, self._publish)

        self.get_logger().info(
            f"leg_height_cmd → /cmd_posture 已就绪 "
            f"(限幅 {self.height_min:.2f}~{self.height_max:.2f}m, 重发 {self.rate:.0f}Hz)")

    def _height_callback(self, msg):
        h = float(msg.data)
        clamped = min(max(h, self.height_min), self.height_max)
        if abs(clamped - h) > 1e-6 and not self.clamped_warned:
            self.get_logger().warn(
                f"规划器请求腿高 {h:.3f}m 超出底盘范围, 已钳到 {clamped:.3f}m")
            self.clamped_warned = True
        self.target_h = clamped

    def _publish(self):
        if self.target_h is None:
            return
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = JOINT_NAMES
        # roll / pitch 单位是度, 导航过程中不做姿态倾斜, 保持水平
        msg.position = [self.target_h, 0.0, 0.0]
        self.posture_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = LegHeightToPosture()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        try:
            node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()
        except (KeyboardInterrupt, ExternalShutdownException):
            # launch/timeout 可能在清理期间再次发送 SIGINT
            pass


if __name__ == "__main__":
    main()
