#!/usr/bin/env python3
"""mock_wl_base.py — 模拟 wheeled_legged_pkg 的 wl_base_node，用于验证姿态指令

复刻真实节点 (wl_base_node.cpp) 的接收判定逻辑，用来检查我们发的指令合不合规：
1. /cmd_posture (sensor_msgs/JointState)
   - 必须同时含 joint_height / joint_roll / joint_pitching，缺一个整条丢弃 (L665)
   - position = [高度(米), roll(度), pitch(度)]
   - 限幅: 高度 0.14~0.36, roll ±15, pitch ±35 (L674-676)
   - 超过 1 秒未更新 → 自动回到默认高度 0.25 且 roll/pitch 归零 (L377-381)
2. /cmd_vel (geometry_msgs/Twist) —— 记录 Nav2 下发的速度，确认与姿态互不干扰

退出时打印统计报告，便于判断动作幅度、频率、回中位是否正确。
"""
import math
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState

REQUIRED = {"joint_height", "joint_roll", "joint_pitching"}
DEFAULT_HEIGHT = 0.25
HEIGHT_MIN, HEIGHT_MAX = 0.14, 0.36
ROLL_LIMIT, PITCH_LIMIT = 15.0, 35.0
TIMEOUT_SEC = 1.0


class MockWlBase(Node):
    def __init__(self):
        super().__init__("mock_wl_base")

        # 当前"下位机"实际生效的姿态
        self.leg_length = DEFAULT_HEIGHT
        self.roll = 0.0
        self.pitch = 0.0

        # 统计
        self.accepted = 0
        self.rejected = 0
        self.clamped = 0
        self.timeouts = 0
        self.roll_seen = []
        self.pitch_seen = []
        self.height_seen = []
        self.vel_count = 0
        self.last_posture_time = None
        self.first_posture_time = None
        self.intervals = []      # 同一动作段内的帧间隔

        self.create_subscription(JointState, "/cmd_posture", self._on_posture, 20)
        self.create_subscription(Twist, "/cmd_vel", self._on_vel, 10)
        # 20Hz 检查超时，对应真实节点的定时回中位保护
        self.create_timer(0.05, self._check_timeout)

        self.get_logger().info(
            "MockWlBase 就绪 | 监听 /cmd_posture (JointState) 与 /cmd_vel (Twist)")

    def _on_posture(self, msg: JointState):
        names = set(msg.name)
        if not REQUIRED.issubset(names) or len(msg.position) < 3:
            self.rejected += 1
            self.get_logger().warn(
                f"[丢弃] 关节名不全或数据不足: name={msg.name}")
            return

        idx = {n: i for i, n in enumerate(msg.name)}
        h = msg.position[idx["joint_height"]]
        r = msg.position[idx["joint_roll"]]
        p = msg.position[idx["joint_pitching"]]

        # 复刻下位机的 clamp
        hc = min(max(h, HEIGHT_MIN), HEIGHT_MAX)
        rc = min(max(r, -ROLL_LIMIT), ROLL_LIMIT)
        pc = min(max(p, -PITCH_LIMIT), PITCH_LIMIT)
        if (hc, rc, pc) != (h, r, p):
            self.clamped += 1
            self.get_logger().warn(
                f"[限幅] 收到 h={h:.3f} r={r:.2f} p={p:.2f} "
                f"-> 截断为 h={hc:.3f} r={rc:.2f} p={pc:.2f}")

        self.leg_length, self.roll, self.pitch = hc, rc, pc
        self.accepted += 1
        self.height_seen.append(hc)
        self.roll_seen.append(rc)
        self.pitch_seen.append(pc)

        now = time.time()
        if self.first_posture_time is None:
            self.first_posture_time = now
        # 按“动作段”统计间隔：两帧相隔 >0.5s 视为新一段（中间隔了导航等其他任务），
        # 不计入频率，否则导航耗时会把平均频率拉低造成误导
        if self.last_posture_time is not None:
            gap = now - self.last_posture_time
            if gap <= 0.5:
                self.intervals.append(gap)
        self.last_posture_time = now

    def _on_vel(self, msg: Twist):
        self.vel_count += 1

    def _check_timeout(self):
        if self.last_posture_time is None:
            return
        if time.time() - self.last_posture_time > TIMEOUT_SEC:
            if (self.roll, self.pitch) != (0.0, 0.0) or \
                    abs(self.leg_length - DEFAULT_HEIGHT) > 1e-9:
                self.timeouts += 1
                self.leg_length, self.roll, self.pitch = DEFAULT_HEIGHT, 0.0, 0.0
                self.get_logger().info("[超时保护] 1秒无新姿态，已回到默认高度且姿态归零")

    def report(self):
        print("\n╔══════════ MockWlBase 统计报告 ══════════")
        print(f"║ /cmd_posture 接收: {self.accepted} 条, 丢弃: {self.rejected} 条, "
              f"触发限幅: {self.clamped} 条")
        if self.intervals:
            avg = sum(self.intervals) / len(self.intervals)
            print(f"║ 动作段内实际下发频率: {1.0 / avg:.1f} Hz "
                  f"(需 >1Hz 才不会被超时保护打断)")
        if self.pitch_seen:
            print(f"║ pitch(点头) 实际范围: {min(self.pitch_seen):+.2f}° ~ "
                  f"{max(self.pitch_seen):+.2f}°  (限 ±{PITCH_LIMIT}°)")
        if self.roll_seen:
            print(f"║ roll(摇头)  实际范围: {min(self.roll_seen):+.2f}° ~ "
                  f"{max(self.roll_seen):+.2f}°  (限 ±{ROLL_LIMIT}°)")
        if self.height_seen:
            print(f"║ 高度范围: {min(self.height_seen):.3f} ~ "
                  f"{max(self.height_seen):.3f} m  (限 {HEIGHT_MIN}~{HEIGHT_MAX})")
        print(f"║ 结束时生效姿态: 高度={self.leg_length:.3f}m "
              f"roll={self.roll:+.2f}° pitch={self.pitch:+.2f}°")
        print(f"║ 超时回中位次数: {self.timeouts}")
        print(f"║ /cmd_vel 收到: {self.vel_count} 条")
        print("╚═════════════════════════════════════════")


def main():
    rclpy.init()
    node = MockWlBase()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.report()
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    main()
