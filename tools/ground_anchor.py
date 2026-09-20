#!/usr/bin/env python3
"""ground_anchor.py — 自动标定地面零点 + 输出 RealSense 离地高度可视化。

诉求(简化版): 建图时自动感知地面高度、让 base_link 反映离地关系; 定位时地图贴合真实
地面从而得到 RealSense 空间定位。核心只需要一个"地面零点"——不做逐点邻域搜索。

原理:
  VINS 的 world 系 z 轴重力对齐, 室内地面就是一个固定水平面, 因此地面在 map 系里是
  一个常数 z。取 rtabmap 分割出的 /cloud_ground 全部点 z 的中位数即得该常数 ground_z。
  离地高度 h = base_z - ground_z (base_z 由 TF map->base_link 得到), 与锚定值无关, 从
  第一帧就正确。

自动锚定(免手工回填):
  odom_to_tf 用 initial_base_height=A 把首帧 base 锚到 A。若 A 不等于真实挂载高度 H,
  地面就不在 z=0 (ground_z = A - H != 0)。本节点据此自动求真值:
        A_new = A_current - ground_z   (=> 下次 ground_z -> 0, 收敛到 A=H)
  并把 A_new 原子写入 anchor_file。两个脚本启动时自动读该文件当锚定值, 无需人工测量/回填。
  建图侧 write_anchor=true 负责写; 定位侧 write_anchor=false 只读该锚定、只做显示。

发布:
  /realsense/height_above_ground (std_msgs/Float32)
  /realsense/height_viz (visualization_msgs/MarkerArray): 竖直线 + 地面交点 + 文字(x,y,h)

用法:
  python3 ground_anchor.py --ros-args -p current_anchor:=0.28 -p anchor_file:=/path/ground_anchor.txt -p write_anchor:=true
"""
import os

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from std_msgs.msg import Float32
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker, MarkerArray
from tf2_ros import Buffer, TransformListener, LookupException, \
    ConnectivityException, ExtrapolationException


class GroundAnchor(Node):
    def __init__(self):
        super().__init__('ground_anchor')
        self.base_frame = self.declare_parameter('base_frame', 'base_link').value
        self.world_frame = self.declare_parameter('world_frame', 'map').value
        self.ground_topic = self.declare_parameter('ground_topic', '/cloud_ground').value
        rate = float(self.declare_parameter('rate', 10.0).value)
        self.height_topic = self.declare_parameter(
            'height_topic', '/realsense/height_above_ground').value
        self.viz_topic = self.declare_parameter('viz_topic', '/realsense/height_viz').value
        # 自动锚定
        self.current_anchor = float(self.declare_parameter('current_anchor', 0.28).value)
        self.anchor_file = self.declare_parameter('anchor_file', '').value
        self.write_anchor = bool(self.declare_parameter('write_anchor', False).value)
        self.min_points = int(self.declare_parameter('min_ground_points', 200).value)

        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
        cloud_qos = QoSProfile(depth=5, reliability=ReliabilityPolicy.RELIABLE,
                               history=HistoryPolicy.KEEP_LAST)
        self.create_subscription(PointCloud2, self.ground_topic, self.on_cloud, cloud_qos)
        self.pub_h = self.create_publisher(Float32, self.height_topic, 10)
        self.pub_viz = self.create_publisher(MarkerArray, self.viz_topic, 10)
        self.timer = self.create_timer(1.0 / max(rate, 1.0), self.tick)

        self.ground_z_pts = None      # 最新 /cloud_ground 的 z 数组
        self.cloud_frame = self.world_frame
        self.ground_z = None          # 平滑后的全局地面 z
        self.written_anchor = None    # 最近写入文件的锚定值
        self.last_write = self.get_clock().now()
        self.warned_tf = False

        self.get_logger().info(
            f'ground_anchor: 地面取 {self.ground_topic} 全局中位数, '
            f'当前锚定 A={self.current_anchor:.3f}m, '
            f'{"自动标定并写入 " + self.anchor_file if self.write_anchor and self.anchor_file else "只读锚定/仅显示"}。')

    def on_cloud(self, msg: PointCloud2):
        self.cloud_frame = msg.header.frame_id or self.world_frame
        zs = [p[2] for p in point_cloud2.read_points(
            msg, field_names=('x', 'y', 'z'), skip_nans=True)]
        self.ground_z_pts = np.asarray(zs, dtype=np.float32) if zs else None

    def lookup_base(self):
        try:
            tf = self.buffer.lookup_transform(self.cloud_frame, self.base_frame,
                                              rclpy.time.Time())
        except (LookupException, ConnectivityException, ExtrapolationException):
            if not self.warned_tf:
                self.get_logger().warn(
                    f'{self.cloud_frame}->{self.base_frame} TF 尚不可用，等待 TF 树就绪...')
                self.warned_tf = True
            return None
        t = tf.transform.translation
        return (t.x, t.y, t.z)

    def tick(self):
        base = self.lookup_base()
        if base is None:
            return
        bx, by, bz = base

        # 全局地面 z: 所有地面点 z 的中位数(点太少先不更新)
        if self.ground_z_pts is not None and len(self.ground_z_pts) >= self.min_points:
            g = float(np.median(self.ground_z_pts))
            # 轻度平滑，抑制帧间抖动
            self.ground_z = g if self.ground_z is None else 0.8 * self.ground_z + 0.2 * g
        if self.ground_z is None:
            return

        height = bz - self.ground_z
        self.pub_h.publish(Float32(data=float(height)))
        self.publish_viz(bx, by, bz, self.ground_z, height)
        self.maybe_write_anchor(bx, by, height)

    def maybe_write_anchor(self, bx, by, height):
        now = self.get_clock().now()
        if (now - self.last_write).nanoseconds < 3e9:
            return
        self.last_write = now
        # A_new = A_current - ground_z (驱动 ground_z -> 0)
        a_new = self.current_anchor - self.ground_z
        msg = (f'离地高度 h={height:.3f}m  平面坐标 ({bx:.2f}, {by:.2f})  '
               f'地面z={self.ground_z:+.3f}  锚定建议 A={a_new:.3f}')
        if not (self.write_anchor and self.anchor_file):
            self.get_logger().info(msg + ' (只读，未写文件)')
            return
        # 变化 >5mm 才写，减少抖动
        if self.written_anchor is not None and abs(a_new - self.written_anchor) < 0.005:
            return
        try:
            tmp = self.anchor_file + '.tmp'
            with open(tmp, 'w') as f:
                f.write(f'{a_new:.4f}\n')
            os.replace(tmp, self.anchor_file)
            self.written_anchor = a_new
            self.get_logger().info(msg + f' → 已写入 {self.anchor_file}')
        except OSError as e:
            self.get_logger().warn(f'写锚定文件失败: {e}')

    def publish_viz(self, x, y, bz, gz, height):
        arr = MarkerArray()
        stamp = self.get_clock().now().to_msg()
        frame = self.cloud_frame

        line = Marker()
        line.header.frame_id = frame
        line.header.stamp = stamp
        line.ns = 'height'
        line.id = 0
        line.type = Marker.LINE_LIST
        line.action = Marker.ADD
        line.scale.x = 0.02
        line.color.a = 1.0
        line.color.r, line.color.g, line.color.b = 0.1, 0.9, 1.0
        line.points = [self._pt(x, y, bz), self._pt(x, y, gz)]
        arr.markers.append(line)

        dot = Marker()
        dot.header.frame_id = frame
        dot.header.stamp = stamp
        dot.ns = 'height'
        dot.id = 1
        dot.type = Marker.SPHERE
        dot.action = Marker.ADD
        dot.pose.position = self._pt(x, y, gz)
        dot.pose.orientation.w = 1.0
        dot.scale.x = dot.scale.y = dot.scale.z = 0.06
        dot.color.a = 1.0
        dot.color.r, dot.color.g, dot.color.b = 1.0, 0.9, 0.0
        arr.markers.append(dot)

        txt = Marker()
        txt.header.frame_id = frame
        txt.header.stamp = stamp
        txt.ns = 'height'
        txt.id = 2
        txt.type = Marker.TEXT_VIEW_FACING
        txt.action = Marker.ADD
        txt.pose.position = self._pt(x, y, bz + 0.15)
        txt.pose.orientation.w = 1.0
        txt.scale.z = 0.12
        txt.color.a = 1.0
        txt.color.r = txt.color.g = txt.color.b = 1.0
        txt.text = f'x={x:.2f} y={y:.2f}\nh={height:.2f}m'
        arr.markers.append(txt)

        self.pub_viz.publish(arr)

    @staticmethod
    def _pt(x, y, z):
        p = Point()
        p.x, p.y, p.z = float(x), float(y), float(z)
        return p


def main():
    rclpy.init()
    node = GroundAnchor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
