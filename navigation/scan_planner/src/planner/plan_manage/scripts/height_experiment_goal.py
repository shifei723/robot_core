#!/usr/bin/env python3
"""Send a single simulation goal only after perception and odometry are ready."""
import time
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2


class ExperimentGoal(Node):
    def __init__(self):
        super().__init__('height_experiment_goal')
        self.x = self.declare_parameter('goal_x', 0.0).value
        self.y = self.declare_parameter('goal_y', 2.0).value
        self.odom = None
        self.clouds = self.maps = 0
        self.sent = False
        self.started = time.monotonic()
        self.pub = self.create_publisher(PoseStamped, '/move_base_simple/goal', 10)
        self.create_subscription(Odometry, '/quad_0/body_pose', self.pose, qos_profile_sensor_data)
        self.create_subscription(PointCloud2, '/quad_0/cloud', self.cloud, qos_profile_sensor_data)
        self.create_subscription(PointCloud2, '/grid_map/occupancy', self.map, qos_profile_sensor_data)
        self.create_timer(.1, self.tick)

    def pose(self, msg):
        self.odom = msg

    def cloud(self, msg):
        self.clouds += 1

    def map(self, msg):
        self.maps += 1

    def tick(self):
        if (self.sent or time.monotonic()-self.started < 4.0 or self.odom is None
                or min(self.clouds, self.maps) < 10 or not self.pub.get_subscription_count()):
            return
        msg = PoseStamped()
        msg.header.frame_id = 'world'
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.position.x, msg.pose.position.y = self.x, self.y
        msg.pose.position.z = self.odom.pose.pose.position.z
        msg.pose.orientation.w = 1.0
        self.pub.publish(msg)
        self.sent = True
        self.get_logger().info(f'Perception ready; sent experiment goal ({self.x}, {self.y})')


def main():
    rclpy.init()
    node = ExperimentGoal()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
