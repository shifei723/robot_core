#!/usr/bin/env python3
"""
地面平面发布器 — 在 RViz 中显示暗色实心地面 + 浅色网格线
发布 visualization_msgs/MarkerArray 到 /ground_plane 话题
  - id 0: CUBE     实心地面
  - id 1: LINE_LIST 网格线 (便于目测尺度与机器人贴地情况)
"""

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA
from geometry_msgs.msg import Point
from builtin_interfaces.msg import Duration


class GroundPlanePublisher(Node):
    def __init__(self):
        super().__init__("ground_plane_publisher")

        self.frame_id = self.declare_parameter("frame_id", "world").value
        self.x_size = self.declare_parameter("x_size", 20.0).value
        self.y_size = self.declare_parameter("y_size", 20.0).value
        self.publish_rate = self.declare_parameter("publish_rate", 1.0).value
        self.grid_step = self.declare_parameter("grid_step", 1.0).value

        # 颜色参数 (默认: 暗灰实心)
        self.r = self.declare_parameter("color.r", 0.16).value
        self.g = self.declare_parameter("color.g", 0.16).value
        self.b = self.declare_parameter("color.b", 0.18).value
        self.a = self.declare_parameter("color.a", 1.0).value

        self.marker_pub = self.create_publisher(MarkerArray, "ground_plane", 10)
        self.array = MarkerArray()
        self.array.markers.append(self._build_plane())
        self.array.markers.append(self._build_grid())

        self.timer = self.create_timer(1.0 / max(0.1, self.publish_rate), self._publish)

        self.get_logger().info(
            f"Ground plane publisher ready: {self.x_size}x{self.y_size}m, "
            f"grid={self.grid_step}m, color=({self.r},{self.g},{self.b},{self.a})")

        self._publish()

    def _build_plane(self):
        m = Marker()
        m.header.frame_id = self.frame_id
        m.ns = "ground"
        m.id = 0
        m.type = Marker.CUBE
        m.action = Marker.ADD
        # z = -0.005 略低于 z=0, 防止与轮子/点云 z-fighting
        m.pose.position = Point(x=0.0, y=0.0, z=-0.005)
        m.pose.orientation.w = 1.0
        m.scale.x = float(self.x_size)
        m.scale.y = float(self.y_size)
        m.scale.z = 0.01
        m.color = ColorRGBA(r=float(self.r), g=float(self.g),
                            b=float(self.b), a=float(self.a))
        m.lifetime = Duration(sec=0, nanosec=0)
        return m

    def _build_grid(self):
        m = Marker()
        m.header.frame_id = self.frame_id
        m.ns = "ground_grid"
        m.id = 1
        m.type = Marker.LINE_LIST
        m.action = Marker.ADD
        m.pose.orientation.w = 1.0
        m.scale.x = 0.012
        m.color = ColorRGBA(r=0.35, g=0.35, b=0.40, a=0.7)
        m.lifetime = Duration(sec=0, nanosec=0)

        hx, hy = self.x_size / 2.0, self.y_size / 2.0
        step = max(0.1, float(self.grid_step))
        z = 0.002  # 略高于地面实体, 保证网格可见

        n = int(hx / step)
        for i in range(-n, n + 1):
            x = i * step
            m.points.append(Point(x=x, y=-hy, z=z))
            m.points.append(Point(x=x, y=hy, z=z))
        n = int(hy / step)
        for i in range(-n, n + 1):
            y = i * step
            m.points.append(Point(x=-hx, y=y, z=z))
            m.points.append(Point(x=hx, y=y, z=z))
        return m

    def _publish(self):
        stamp = self.get_clock().now().to_msg()
        for m in self.array.markers:
            m.header.stamp = stamp
        self.marker_pub.publish(self.array)


def main(args=None):
    rclpy.init(args=args)
    node = GroundPlanePublisher()
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
