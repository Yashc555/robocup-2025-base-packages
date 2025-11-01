#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Define LiDAR configurations
    channel_type = LaunchConfiguration('channel_type', default='serial')
    serial_port = LaunchConfiguration('serial_port', default='/dev/ttyUSB0')
    serial_baudrate = LaunchConfiguration('serial_baudrate', default='115200')
    frame_id = LaunchConfiguration('frame_id', default='laser')
    inverted = LaunchConfiguration('inverted', default='false')
    angle_compensate = LaunchConfiguration('angle_compensate', default='true')
    scan_mode = LaunchConfiguration('scan_mode', default='Standard')
    
    # RViz config path
    rviz_config_dir = os.path.join(
        get_package_share_directory('sllidar_ros2'),
        'rviz',
        'sllidar_ros2_final.rviz'
    )

    return LaunchDescription([
        # Launch arguments
        DeclareLaunchArgument('channel_type', default_value=channel_type, description='Specifying channel type of lidar'),
        DeclareLaunchArgument('serial_port', default_value=serial_port, description='Specifying USB port for lidar'),
        DeclareLaunchArgument('serial_baudrate', default_value=serial_baudrate, description='Specifying USB port baudrate for lidar'),
        DeclareLaunchArgument('frame_id', default_value=frame_id, description='Specifying frame_id of lidar'),
        DeclareLaunchArgument('inverted', default_value=inverted, description='Invert scan data'),
        DeclareLaunchArgument('angle_compensate', default_value=angle_compensate, description='Enable angle compensation for scan data'),
        DeclareLaunchArgument('scan_mode', default_value=scan_mode, description='Specifying scan mode of lidar'),

        # LiDAR node
        Node(
            package='sllidar_ros2',
            executable='sllidar_node',
            name='sllidar_node',
            parameters=[{
                'channel_type': channel_type,
                'serial_port': serial_port,
                'serial_baudrate': serial_baudrate,
                'frame_id': frame_id,
                'inverted': inverted,
                'angle_compensate': angle_compensate,
                'scan_mode': scan_mode,
                'queue_size': 10  # Increase queue size (default is 10, try increasing it)
            }],
            output='screen',
            remappings=[('/scan', '/laser_scan')]
        ),

        # Static transforms (LiDAR positioning)
        # Node(
        #     package='tf2_ros',
        #     executable='static_transform_publisher',
        #     name='base_footprint_to_base_link',
        #     arguments=['0', '0', '0', '0', '0', '0', 'odom', 'base_footprint']
        # ),
        # Static transforms
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_footprint_to_base_link',
            arguments=['0', '0', '0', '0', '0', '0', 'base_footprint', 'base_link']
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_link_to_laser',
            arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'laser']
        ),

        # RViz node
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_dir],
            output='screen'
        ),
    ])
