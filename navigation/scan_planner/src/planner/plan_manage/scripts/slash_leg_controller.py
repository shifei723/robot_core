#!/usr/bin/env python3
"""
Slash 双轮足机器人 — 腿高控制 ROS2 节点 (简化版)
=================================================
仅实现腿高控制，无速度环。

控制流程:
  leg_height_cmd → leg_height_ramp → current_h (腿高)
  right_ik / left_ik (x=0) → 四髋关节角度
  five_bar_knee_angles_closed → 膝关节角度 (闭环约束)
  → joint_states (RViz 可视化)
"""

import math
import os
import sys

import rclpy
from rclpy.executors import ExternalShutdownException
from std_msgs.msg import Float32
from rclpy.node import Node
from sensor_msgs.msg import JointState

# 添加 scripts/ 目录到 path，以便导入 leg_height_control
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from leg_height_control import (
    DEFAULT_HEIGHT, HEIGHT_RAMP_RATE, MIN_LOW, MAX_HIGH,
    right_ik, left_ik,
    firmware_to_mujoco_left, firmware_to_mujoco_right,
    leg_height_ramp, five_bar_knee_angles_closed,
    _clamp,
)

# ==================== ROS2 节点 ====================

JOINT_NAMES = [
    "lf_joint1", "lf_joint2",
    "lb_joint1", "lb_joint2",
    "rf_joint1", "rf_joint2",
    "rb_joint1", "rb_joint2",
    "l_wheel_joint", "r_wheel_joint",
]


class SlashLegController(Node):
    def __init__(self):
        super().__init__("slash_leg_controller")

        # 参数
        self.ctrl_rate = self.declare_parameter("rate", 50.0).value
        self.target_h = self.declare_parameter("target_height", DEFAULT_HEIGHT).value
        self.ramp_rate = self.declare_parameter("ramp_rate", HEIGHT_RAMP_RATE).value

        # 状态
        self.current_h = DEFAULT_HEIGHT

        # 腿高指令订阅 (Float32, 单位: 米)
        self.height_sub = self.create_subscription(
            Float32, "leg_height_cmd", self._height_callback, 10)

        # 发布
        self.joint_pub = self.create_publisher(JointState, "joint_states", 10)

        # 定时器 (50Hz 控制环)
        dt = 1.0 / max(1.0, self.ctrl_rate)
        self.timer = self.create_timer(dt, self._timer_callback)

        # 关节消息初始化
        self.joint_msg = JointState()
        self.joint_msg.name = JOINT_NAMES
        self.joint_msg.position = [0.0] * len(JOINT_NAMES)
        self.joint_msg.velocity = [0.0] * len(JOINT_NAMES)

        # 初始站立姿态
        (q_lf, q_lf2, q_lb, q_lb2), (q_rf, q_rf2, q_rb, q_rb2) = self._compute_leg_ctrl(0.0)
        self._fill_joint_msg(q_lf, q_lf2, q_lb, q_lb2, q_rf, q_rf2, q_rb, q_rb2)

        self.get_logger().info(
            f"Slash leg controller ready (rate={self.ctrl_rate}Hz, h={self.target_h}m)")
        self.get_logger().info(
            f"Initial pose: q_lf={q_lf:.4f}, q_lf2={q_lf2:.4f}, "
            f"q_lb={q_lb:.4f}, q_lb2={q_lb2:.4f}")

        # 立即发布一次初始姿态
        self.joint_msg.header.stamp = self.get_clock().now().to_msg()
        self.joint_pub.publish(self.joint_msg)

    def _height_callback(self, msg: Float32):
        """接收腿高指令 (米)

        钳位必须与机构真实行程 [MIN_LOW, MAX_HIGH] 一致。
        若这里用更窄的范围, 深蹲时视觉腿长会比仿真假设的更长,
        轮子会穿透地面。
        """
        h = _clamp(msg.data, MIN_LOW, MAX_HIGH)
        self.target_h = h
        self.get_logger().debug(f"Leg height cmd: {h:.3f} m")

    def _compute_leg_ctrl(self, dt: float):
        """
        一步腿高控制计算 (纯腿高，无前后偏移)
        :return: ((q_lf, q_lf2, q_lb, q_lb2), (q_rf, q_rf2, q_rb, q_rb2))
        """
        # 1. 腿高斜坡插值
        self.current_h = leg_height_ramp(
            self.current_h, self.target_h, dt, self.ramp_rate)

        # 2. IK (x=0, 无俯仰, 纯腿高控制)
        ik_x = 0.0
        pitch_sp = 0.0
        
        h_l = _clamp(self.current_h, MIN_LOW, MAX_HIGH)
        h_r = _clamp(self.current_h, MIN_LOW, MAX_HIGH)

        alpha_r, beta_r, err_r = right_ik(ik_x, h_r, -pitch_sp)
        alpha_l, beta_l, err_l = left_ik(ik_x, h_l, -pitch_sp)

        # 3. 角度映射 (joint1 = 髋关节)
        q_lf, q_lb = firmware_to_mujoco_left(alpha_l, beta_l)
        q_rf, q_rb = firmware_to_mujoco_right(alpha_r, beta_r)
        q_lf = _clamp(q_lf, 0.0, 1.54)
        q_lb = _clamp(q_lb, 0.0, 1.54)
        q_rf = _clamp(q_rf, 0.0, 1.54)
        q_rb = _clamp(q_rb, 0.0, 1.54)

        # 4. 膝关节角度 (由髋关节角精确求解五连杆闭环)
        q_lf2, q_lb2 = five_bar_knee_angles_closed(q_lf, q_lb)
        q_rf2, q_rb2 = five_bar_knee_angles_closed(q_rf, q_rb)

        # 调试日志 (每 50 次打印一次)
        if not hasattr(self, '_debug_cnt'):
            self._debug_cnt = 0
        self._debug_cnt += 1
        if self._debug_cnt % 50 == 1:
            self.get_logger().info(
                f"h={self.current_h:.3f} alpha_l={alpha_l:.1f} beta_l={beta_l:.1f} "
                f"q_lf={q_lf:.3f} q_lf2={q_lf2:.3f} q_lb={q_lb:.3f} q_lb2={q_lb2:.3f}")

        return (q_lf, q_lf2, q_lb, q_lb2), (q_rf, q_rf2, q_rb, q_rb2)

    def _fill_joint_msg(self, q_lf, q_lf2, q_lb, q_lb2, q_rf, q_rf2, q_rb, q_rb2):
        """填充关节消息"""
        # 髋关节位置: index 0, 2, 4, 6
        self.joint_msg.position[0] = q_lf   # lf_joint1
        self.joint_msg.position[2] = q_lb   # lb_joint1
        self.joint_msg.position[4] = q_rf   # rf_joint1
        self.joint_msg.position[6] = q_rb   # rb_joint1
        # 膝关节位置: index 1, 3, 5, 7
        self.joint_msg.position[1] = q_lf2  # lf_joint2
        self.joint_msg.position[3] = q_lb2  # lb_joint2
        self.joint_msg.position[5] = q_rf2  # rf_joint2
        self.joint_msg.position[7] = q_rb2  # rb_joint2
        # 轮子位置: index 8, 9
        self.joint_msg.position[8] = 0.0
        self.joint_msg.position[9] = 0.0
        # 速度全部为 0
        self.joint_msg.velocity = [0.0] * len(JOINT_NAMES)

    def _timer_callback(self):
        stamp = self.get_clock().now().to_msg()
        dt = 1.0 / max(1.0, self.ctrl_rate)

        # 计算腿高控制
        (q_lf, q_lf2, q_lb, q_lb2), (q_rf, q_rf2, q_rb, q_rb2) = self._compute_leg_ctrl(dt)

        # 填充并发布
        self.joint_msg.header.stamp = stamp
        self._fill_joint_msg(q_lf, q_lf2, q_lb, q_lb2, q_rf, q_rf2, q_rb, q_rb2)
        self.joint_pub.publish(self.joint_msg)


def main(args=None):
    rclpy.init(args=args)
    node = SlashLegController()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
