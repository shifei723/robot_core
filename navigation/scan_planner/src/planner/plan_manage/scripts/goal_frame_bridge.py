#!/usr/bin/env python3
"""
目标点坐标系桥接 + 里程计发散看门狗
====================================
两个职责都围绕"规划跑在 VINS 的 world 系"这一前提:

一、目标点转换
  规划器的 rvizGoalCallback (scan_replan_fsm.cpp) 完全不看
  header.frame_id, 拿到坐标就当 world 系用。而 rviz2 里通常以 rtabmap 的
  map 系为固定帧点目标, 两系之间差着重定位修正量, 直接喂进去会按该偏差跑偏。
  本节点把任意帧的目标先经 tf2 转到 world 再转发。

二、发散看门狗
  VINS 发散时 body_pose 会飞, 规划器会据此输出无意义轨迹。检测到里程计
  超时或位姿瞬跳后, 持续发零速压住底盘, 比等人去按急停可靠。
  仅在收到过首帧里程计之后才启用, 免得底盘未上电时空刷告警。
"""

import math

import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
import tf2_geometry_msgs  # noqa: F401  注册 PoseStamped 的 tf2 转换支持
from tf2_ros import Buffer, TransformListener


class GoalFrameBridge(Node):
    def __init__(self):
        super().__init__("goal_frame_bridge")

        self.target_frame = self.declare_parameter("target_frame", "world").value
        self.tf_timeout = self.declare_parameter("tf_timeout", 0.3).value
        # 里程计静默多久算失联
        self.odom_timeout = self.declare_parameter("odom_timeout", 0.5).value
        # 相邻两帧等效速度超过此值判为发散跳变
        self.max_jump_speed = self.declare_parameter("max_jump_speed", 5.0).value
        self.enable_watchdog = self.declare_parameter("enable_watchdog", True).value

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.goal_sub = self.create_subscription(
            PoseStamped, "goal_in", self._goal_callback, 1)
        # RViz2 新版默认使用 /goal_pose，旧版/项目自带 slash.rviz 使用
        # /move_base_simple/goal；由 launch 分别重映射，兼容两种配置。
        self.goal_sub_legacy = self.create_subscription(
            PoseStamped, "goal_in_legacy", self._goal_callback, 1)
        self.goal_pub = self.create_publisher(PoseStamped, "goal_out", 1)

        self.last_odom_time = None
        self.last_odom_pos = None
        self.diverged = False
        # 位姿瞬跳/NaN 说明 VINS 本身已经坏了, 锁死等人工介入;
        # 单纯的里程计断流多见于启动阶段或偶发丢帧, 数据回来即自行解除
        self.latched = False

        if self.enable_watchdog:
            self.odom_sub = self.create_subscription(
                Odometry, "body_pose", self._odom_callback, 10)
            self.cmd_vel_pub = self.create_publisher(Twist, "cmd_vel", 20)
            self.watchdog_timer = self.create_timer(0.02, self._watchdog)

        self.get_logger().info(
            f"目标点桥接已就绪 (goal_in → {self.target_frame} → goal_out"
            f"{', 看门狗开启' if self.enable_watchdog else ''})")

    # ---------- 目标点转换 ----------

    def _goal_callback(self, msg):
        if self.enable_watchdog:
            if self.last_odom_time is None:
                self.get_logger().error("目标点丢弃: 尚未收到有效里程计")
                return
            age = (self.get_clock().now() - self.last_odom_time).nanoseconds * 1e-9
            if self.diverged or age > self.odom_timeout:
                self.get_logger().error(
                    f"目标点丢弃: 定位异常或里程计已静默 {age:.2f}s")
                return

        src = msg.header.frame_id or self.target_frame
        if src == self.target_frame:
            # 规划器 rvizGoalCallback 会用 rviz_goal_height_ 覆盖目标 z, 且当传入
            # 目标 z < -0.1 时静默丢弃(scan_replan_fsm.cpp:149)。world 系经地面锚定
            # 后 z 基准可能为负, 这里统一把 z 归零, 保证目标不被误丢(高度由规划器定)。
            msg.pose.position.z = 0.0
            self.goal_pub.publish(msg)
            self.get_logger().info(
                f"目标 ({msg.pose.position.x:.2f}, {msg.pose.position.y:.2f}) "
                f"已在 {self.target_frame} 系, 直接转发")
            return

        try:
            out = self.tf_buffer.transform(
                msg, self.target_frame,
                timeout=Duration(seconds=float(self.tf_timeout)))
        except Exception as exc:  # tf2 的异常类型较杂, 统一按查不到处理
            self.get_logger().warn(
                f"目标点丢弃: 无法把 {src} 转到 {self.target_frame} ({exc})")
            return

        out.header.frame_id = self.target_frame
        out.pose.position.z = 0.0  # 同上: 避免 world 系负 z 触发规划器 z<-0.1 静默丢弃
        self.goal_pub.publish(out)
        self.get_logger().info(
            f"目标 {src}({msg.pose.position.x:.2f}, {msg.pose.position.y:.2f}) → "
            f"{self.target_frame}({out.pose.position.x:.2f}, {out.pose.position.y:.2f})")

    # ---------- 发散看门狗 ----------

    def _odom_callback(self, msg):
        now = self.get_clock().now()
        pos = msg.pose.pose.position
        cur = (pos.x, pos.y, pos.z)

        if not all(math.isfinite(v) for v in cur):
            self._trip("里程计出现 NaN/Inf", latch=True)
            return

        jumped = False
        if self.last_odom_pos is not None:
            dt = (now - self.last_odom_time).nanoseconds * 1e-9
            if dt > 1e-6:
                dist = math.dist(cur, self.last_odom_pos)
                if dist / dt > self.max_jump_speed:
                    self._trip(f"里程计瞬跳 {dist:.2f}m / {dt * 1000:.0f}ms", latch=True)
                    jumped = True

        self.last_odom_time = now
        self.last_odom_pos = cur

        if self.diverged and not self.latched and not jumped:
            self.diverged = False
            self.get_logger().info("里程计已恢复, 解除零速压制")

    def _watchdog(self):
        if self.last_odom_time is None:
            return  # 首帧未到, 底盘可能还没上电, 不介入

        age = (self.get_clock().now() - self.last_odom_time).nanoseconds * 1e-9
        if age > self.odom_timeout:
            self._trip(f"里程计已静默 {age:.2f}s")

        if self.diverged:
            self.cmd_vel_pub.publish(Twist())  # 全零, 压住底盘

    def _trip(self, reason, latch=False):
        if latch:
            self.latched = True
        if not self.diverged:
            self.diverged = True
            tail = ", 需排查 VINS 后重启本节点" if latch else ""
            self.get_logger().error(f"定位异常, 持续下发零速: {reason}{tail}")
        else:
            self.get_logger().warn(f"定位仍异常: {reason}", throttle_duration_sec=2.0)


def main(args=None):
    rclpy.init(args=args)
    node = GoalFrameBridge()
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
