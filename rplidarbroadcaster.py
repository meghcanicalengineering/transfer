#!/usr/bin/env python3

import rclpy
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
import math

def main():
    rclpy.init()
    node = rclpy.create_node('static_transform_publisher')

    broadcaster = TransformBroadcaster(node)

    static_transformStamped = TransformStamped()
    static_transformStamped.header.stamp = node.get_clock().now().to_msg()
    static_transformStamped.header.frame_id = 'map'
    static_transformStamped.child_frame_id = 'laser'
    static_transformStamped.transform.translation.x = 0.0
    static_transformStamped.transform.translation.y = 0.0
    static_transformStamped.transform.translation.z = 0.0
    quat = tf.transformations.quaternion_from_euler(0, 0, 0)
    static_transformStamped.transform.rotation.x = quat[0]
    static_transformStamped.transform.rotation.y = quat[1]
    static_transformStamped.transform.rotation.z = quat[2]
    static_transformStamped.transform.rotation.w = quat[3]

    broadcaster.sendTransform(static_transformStamped)

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
