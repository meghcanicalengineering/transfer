#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    channel_type =  DeclareLaunchArgument('channel_type', default_value='serial', description='Specifying channel type of lidar')
    serial_port = DeclareLaunchArgument('serial_port', default_value='/dev/ttyUSB0', description='Specifying usb port to connected lidar')
    serial_baudrate = DeclareLaunchArgument('serial_baudrate', default_value='115200', description='Specifying usb port baudrate to connected lidar')
    frame_id = DeclareLaunchArgument('frame_id', default_value='laser', description='Specifying frame_id of lidar')
    inverted = DeclareLaunchArgument('inverted', default_value='false', description='Specifying whether or not to invert scan data')
    angle_compensate = DeclareLaunchArgument('angle_compensate', default_value='true', description='Specifying whether or not to enable angle_compensate of scan data')
    scan_mode = DeclareLaunchArgument('scan_mode', default_value='Sensitivity', description='Specifying scan mode of lidar')
    
    return LaunchDescription([
        channel_type,
        serial_port,
        serial_baudrate,
        frame_id,
        inverted,
        angle_compensate,
        scan_mode,

        Node(
            package='rplidar_ros',
            executable='rplidar_node',
            name='rplidar_node',
            parameters=[{'channel_type': LaunchConfiguration('channel_type'),
                         'serial_port': LaunchConfiguration('serial_port'),
                         'serial_baudrate': LaunchConfiguration('serial_baudrate'),
                         'frame_id': LaunchConfiguration('frame_id'),
                         'inverted': LaunchConfiguration('inverted'),
                         'angle_compensate': LaunchConfiguration('angle_compensate')}],
            output='screen'
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([os.path.join(get_package_share_directory('your_package_name'), 'launch', 'static_transform_publisher_launch.py')]),
            launch_arguments={'static_transform_publisher_script': os.path.join(get_package_share_directory('your_package_name'), 'scripts', 'static_transform_publisher.py')}.items(),
        ),
    ])
