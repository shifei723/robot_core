#!/usr/bin/python3
"""Timestamped map->camera_link TF from real VINS output, without fake odometry."""
import rclpy
import argparse
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.qos import qos_profile_sensor_data
from tf2_ros import TransformBroadcaster


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--odom-topic', default='/vins/odometry')
    args = parser.parse_args()
    rclpy.init()
    node = rclpy.create_node('vins_foxglove_tf')
    broadcaster = TransformBroadcaster(node)
    def callback(msg):
        tf = TransformStamped()
        tf.header = msg.header
        tf.child_frame_id = 'camera_link'
        tf.transform.translation.x = msg.pose.pose.position.x
        tf.transform.translation.y = msg.pose.pose.position.y
        tf.transform.translation.z = msg.pose.pose.position.z
        tf.transform.rotation = msg.pose.pose.orientation
        broadcaster.sendTransform(tf)
    subscription = node.create_subscription(Odometry, args.odom_topic, callback, qos_profile_sensor_data)
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
