#!/usr/bin/env python

import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # Define parameters
    bus_number = '1'

    # Launch the IMU publisher node
    imu_publisher_node = Node(
        package='publishIMU',
        executable='imu_publisher_node.py',
        name='imu_publisher_node',
        output='screen',
        parameters=[{'bus_number': bus_number}]
    )

    return LaunchDescription([imu_publisher_node])

if __name__ == '__main__':
    generate_launch_description()
