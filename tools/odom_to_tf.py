#!/usr/bin/env python3
"""odom_to_tf.py — 把 VINS 的 Odometry 转成 TF，并重定向到 REP-103 的 base frame。

做两件事:

1. 补 TF。VINS-Fusion 的 pubTF() 第一行就是 `return; // tmp.`，完全不发 TF
   （且其函数体内用了未初始化的 TransformBroadcaster 指针，去掉 return 会崩），
   只发 /odometry 消息。而 rtabmap 需要 TF 才能把里程计位姿插值到图像时刻
   （approx_sync 下 odom 15Hz / 图像 30Hz，时间戳对不齐）。

2. 把 base frame 换成 REP-103。VINS 的 body 帧就是 IMU 光学系(x右 y下 z前)，
   而 rtabmap 的 Grid 地面分割假定 base frame 是 REP-103(x前 y左 z上):
   它从位姿里取 roll/pitch 把点云"水平化"后再分离地面与障碍
   （见 LocalGridMaker.cpp 中 Grid/MapFrameProjection 的分支）。
   拿 body 直接当 base frame 时，相机水平前视的 roll 恒为 -90°（实测数据库里
   中位数 -89.1°），水平化等于把每帧点云又绕 x 轴转了 89°，2D 栅格图彻底错乱。
   而 3D 点云不经过这段代码，所以看着完全正常——很容易误判成 TF 绑错。
   所以这里额外发布一个与 body 同原点、姿态转成 REP-103 的 base_link:
       T_world_base = T_world_body · T_body_base   (T_body_base 为纯旋转)
   并把里程计重发到 out_topic（child_frame_id=base_link）供 rtabmap 订阅。
   注意 rtabmap 并不校验 odom 的 child_frame_id（CoreWrapper 里只在发布时用到
   该字段），它直接把消息位姿当成 map→frame_id，所以话题与 frame_id 必须成对改。

   twist 原样透传: VINS 填的 twist.linear 是 world 系速度(estimator.Vs)，本就不符
   ROS "twist 表达在 child frame" 的约定，与 base frame 选哪个无关；
   而 twist.angular 和 covariance VINS 都没填(全 0)。

内置发散保护: VINS-Fusion 没有零速约束，相机长时间静止时加速度计 bias
      与重力方向不可解耦，bias 一漂位置就会二次积分爆炸（实测静止
      29 秒后开始发散，5 分钟跑到 1500 米）。发散后若继续转发 TF，
      rtabmap 会把节点撑成几百米范围的栅格图（实测 7699x10832 格），
      rviz 内存吃到 2.7GB、整机 load 上 9。所以一旦测到超物理上限的
      速度就停止转发，让 TF 树主动断开以保护下游。

用法:
  python3 odom_to_tf.py                                     # /odometry → /odometry_base
  python3 odom_to_tf.py --ros-args -p odom_topic:=/xxx      # 指定输入话题
  python3 odom_to_tf.py --ros-args -p base_frame:=''        # 关闭重定向，原样转发 body
  python3 odom_to_tf.py --ros-args -p initial_base_height:=0.28  # 首帧 z 锚定到机体高度
  python3 odom_to_tf.py --ros-args -p max_speed:=0.0        # 0 = 关闭发散保护
"""
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster

# body(光学系 x右 y下 z前) → base_link(REP-103 x前 y左 z上) 的纯旋转:
# base 的三轴在 body 中依次是 z、-x、-y。四元数按 (x,y,z,w) 排列。
Q_BODY_BASE = (0.5, -0.5, 0.5, 0.5)


def quat_mul(a, b):
    """四元数乘 a⊗b，分量顺序均为 (x,y,z,w)。"""
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (aw * bx + ax * bw + ay * bz - az * by,
            aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw,
            aw * bw - ax * bx - ay * by - az * bz)


class OdomToTf(Node):
    def __init__(self):
        super().__init__('odom_to_tf')
        topic = self.declare_parameter('odom_topic', '/odometry').value
        # 置空则不做重定向，退回“原样转发 body”的行为
        self.base_frame = self.declare_parameter('base_frame', 'base_link').value
        out_topic = self.declare_parameter('out_topic', '/odometry_base').value
        # 手持步行 < 2 m/s，快跑 ~4 m/s；取 5 m/s 作为“物理上不可能”的判定线
        self.max_speed = self.declare_parameter('max_speed', 5.0).value
        # 发散判定的最小评估时间窗(s)。高频里程计(如 200Hz 的 /imu_propagate)下
        # 逐帧 dt≈5ms，优化修正的 mm~cm 级微跳会把瞬时速度冲破阈值造成误判发散；
        # 累计到该窗口再评估速度，可避免误锁又不削弱对真实发散(持续高速)的保护。
        self.min_speed_dt = self.declare_parameter('min_speed_dt', 0.05).value
        # VINS 的 world 原点位于启动时的 IMU，z≈0；导航规划器则要求
        # base_link.z 表示相对地面的机体高度。正值时把首帧 z 锚定到该高度，
        # 仅改变坐标原点，不改变后续垂向位移；负值保持原始行为。
        self.initial_base_height = self.declare_parameter(
            'initial_base_height', -1.0).value
        self.initial_input_z = None
        self.br = TransformBroadcaster(self)
        # VINS 用默认 QoS(reliable) 发布；队列给大些防丢
        qos = QoSProfile(depth=50,
                         reliability=ReliabilityPolicy.RELIABLE,
                         history=HistoryPolicy.KEEP_LAST)
        self.pub = (self.create_publisher(Odometry, out_topic, qos)
                    if self.base_frame else None)
        self.create_subscription(Odometry, topic, self.cb, qos)
        self.count = 0
        self.prev = None      # (t, x, y, z)
        self.diverged = False
        if self.base_frame:
            self.get_logger().info(
                f'odom_to_tf: {topic} → TF + {out_topic}'
                f' (base={self.base_frame}, 姿态已转成 REP-103;'
                f' 发散阈值 {self.max_speed} m/s;'
                f' 初始高度 {self.initial_base_height if self.initial_base_height >= 0 else "原始z"})')
        else:
            self.get_logger().info(f'odom_to_tf: {topic} → TF 原样转发'
                                  f' (发散阈值 {self.max_speed} m/s)')

    def cb(self, msg):
        if self.diverged:
            return
        p = msg.pose.pose.position
        t_now = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if self.initial_input_z is None:
            self.initial_input_z = p.z
        out_z = (p.z - self.initial_input_z + self.initial_base_height
                 if self.initial_base_height >= 0.0 else p.z)
        if self.max_speed > 0:
            if self.prev is None:
                self.prev = (t_now, p.x, p.y, p.z)
            else:
                dt = t_now - self.prev[0]
                # 仅在累计时间窗 >= min_speed_dt 时评估一次速度并推进参考点，
                # 避免高频输入下单帧微跳被误判为发散（见 min_speed_dt 说明）。
                if dt >= self.min_speed_dt:
                    dist = math.dist((p.x, p.y, p.z), self.prev[1:])
                    if dist / dt > self.max_speed:
                        self.diverged = True
                        self.get_logger().error(
                            f'里程计已发散: {dist / dt:.1f} m/s '
                            f'(位置 {p.x:.1f},{p.y:.1f},{p.z:.1f})。'
                            f'停止转发 TF 以保护下游建图。'
                            f'原因通常是相机长时间静止（VIO 无零速约束），'
                            f'请重启并全程手持移动。')
                        return
                    self.prev = (t_now, p.x, p.y, p.z)

        q = msg.pose.pose.orientation
        child, quat = msg.child_frame_id, (q.x, q.y, q.z, q.w)
        if self.base_frame:
            # 与 body 同原点，所以只右乘旋转、平移保持不变
            child, quat = self.base_frame, quat_mul(quat, Q_BODY_BASE)

        t = TransformStamped()
        t.header = msg.header
        t.child_frame_id = child
        t.transform.translation.x = p.x
        t.transform.translation.y = p.y
        t.transform.translation.z = out_z
        t.transform.rotation.x = quat[0]
        t.transform.rotation.y = quat[1]
        t.transform.rotation.z = quat[2]
        t.transform.rotation.w = quat[3]
        self.br.sendTransform(t)

        if self.pub is not None:
            out = Odometry()
            out.header = msg.header
            out.child_frame_id = child
            out.pose.pose.position.x = p.x
            out.pose.pose.position.y = p.y
            out.pose.pose.position.z = out_z
            out.pose.pose.orientation = t.transform.rotation
            out.pose.covariance = msg.pose.covariance
            out.twist = msg.twist          # world 系速度，见文件头说明
            self.pub.publish(out)

        self.count += 1
        if self.count == 1:
            self.get_logger().info(
                f'首帧已转发: {msg.header.frame_id} → {child}'
                f' (输入z={p.z:.3f}, 输出z={out_z:.3f})')


def main():
    rclpy.init()
    node = OdomToTf()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
