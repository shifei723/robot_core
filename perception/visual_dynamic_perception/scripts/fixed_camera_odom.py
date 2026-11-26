#!/usr/bin/python3
"""Explicit fixed-camera experiment: identity pose is a mounting constraint.

Do not use this mode for a moving camera. It is not a VINS fallback.
"""
import rclpy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image
from rclpy.qos import qos_profile_sensor_data


def main():
    rclpy.init()
    node = rclpy.create_node('fixed_detection_camera_pose')
    node.get_logger().warn('FIXED-CAMERA EXPERIMENT: keep camera stationary; identity pose is not estimated odometry')
    publisher = node.create_publisher(Odometry, '/detection/odometry', 10)
    def callback(image):
        msg = Odometry()
        msg.header.stamp = image.header.stamp
        msg.header.frame_id = 'map'
        msg.child_frame_id = 'camera_link'
        msg.pose.pose.orientation.w = 1.
        publisher.publish(msg)
    subscription = node.create_subscription(Image, '/camera/depth/image_raw', callback, qos_profile_sensor_data)
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
