#!/usr/bin/env python3
"""tf_to_odom.py — 把 TF 链上的 parent→child 变换采样成 nav_msgs/Odometry 发出来。

用途: VINS 联合定位链路里，rtabmap 重定位时发布 map→world 修正 TF，而
odom_to_tf.py 发布 world→base_link。二者串起来 map→base_link 才是"吃到了
重定位修正的全局位姿"。但 rtabmap 只把它挂在 TF 上，没有对应的 odometry 话题。
本节点定时查询 map→base_link，重新打包成 Odometry(header.frame_id=map,
child_frame_id=base_link) 发布，供需要全局位姿话题的下游(如规划器/监控)订阅。

★ 重要取舍(务必先读):
  map→world 是重定位一次性跳变的修正量——匹配上已知地点的瞬间会阶跃。
  因此本话题是"全局一致但会跳"的位姿，和 /odometry_base"局部连续但会漂"正好互补。
  直接拿它当局部规划器的位姿输入，会让规划起点在重定位瞬间瞬移、轨迹被打断，
  这也是 run.launch.py 里 "规划全部跑在 VINS 的 world 系" 注释的由来。
  推荐把它用于: 全局位姿监控、把目标点/关键点锚定到 map 系(配合 goal_frame_bridge)。

用法:
  python3 tf_to_odom.py                                          # map→base_link → /odometry_global
  python3 tf_to_odom.py --ros-args -p parent_frame:=map -p child_frame:=base_link
  python3 tf_to_odom.py --ros-args -p out_topic:=/odometry_global -p rate:=30.0
"""
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from nav_msgs.msg import Odometry
from tf2_ros import Buffer, TransformListener, LookupException, \
    ConnectivityException, ExtrapolationException


class TfToOdom(Node):
    def __init__(self):
        super().__init__('tf_to_odom')
        self.parent = self.declare_parameter('parent_frame', 'map').value
        self.child = self.declare_parameter('child_frame', 'base_link').value
        out_topic = self.declare_parameter('out_topic', '/odometry_global').value
        rate = self.declare_parameter('rate', 30.0).value

        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
        qos = QoSProfile(depth=20,
                         reliability=ReliabilityPolicy.RELIABLE,
                         history=HistoryPolicy.KEEP_LAST)
        self.pub = self.create_publisher(Odometry, out_topic, qos)
        self.timer = self.create_timer(1.0 / max(rate, 1.0), self.tick)
        self.count = 0
        self.warned = False
        self.get_logger().info(
            f'tf_to_odom: {self.parent}→{self.child} 采样为 Odometry 发到 {out_topic}'
            f' ({rate:.0f} Hz)。注意该位姿全局一致但重定位时会跳，勿直接当局部规划输入。')

    def tick(self):
        try:
            # 取最新可用变换(Time() = 时间 0 表示"最近的")
            tf = self.buffer.lookup_transform(
                self.parent, self.child, rclpy.time.Time())
        except (LookupException, ConnectivityException, ExtrapolationException):
            # 重定位尚未归位时 map→world 还不存在，属正常，节流提示一次即可
            if not self.warned:
                self.get_logger().warn(
                    f'{self.parent}→{self.child} 尚不可用，等待重定位归位/ TF 就绪...')
                self.warned = True
            return

        msg = Odometry()
        msg.header.stamp = tf.header.stamp
        msg.header.frame_id = self.parent
        msg.child_frame_id = self.child
        msg.pose.pose.position.x = tf.transform.translation.x
        msg.pose.pose.position.y = tf.transform.translation.y
        msg.pose.pose.position.z = tf.transform.translation.z
        msg.pose.pose.orientation = tf.transform.rotation
        # twist 无法从单帧 TF 稳定求出，留空(全 0)。需要速度请用 /odometry_base 的 twist。
        self.pub.publish(msg)

        self.count += 1
        if self.count == 1:
            p = tf.transform.translation
            self.get_logger().info(
                f'首帧全局位姿已发布: {self.parent}→{self.child}'
                f' ({p.x:.2f}, {p.y:.2f}, {p.z:.2f})')


def main():
    rclpy.init()
    node = TfToOdom()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
