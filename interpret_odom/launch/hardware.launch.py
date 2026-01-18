import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    my_pkg_dir = get_package_share_directory('interpret_odom')
    sll_dir = get_package_share_directory('sllidar_ros2')

    slam_params = os.path.join(my_pkg_dir, 'config', 'mapper_params_online_async.yaml')
    rviz_config = os.path.join(sll_dir, 'rviz', 'sllidar_ros2_final.rviz')

    # Include Hardware
    hardware_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(my_pkg_dir, 'launch', 'hardware.launch.py'))
    )

    slam_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(sll_dir, 'launch', 'online_async_launch.py')),
        launch_arguments={'params_file': slam_params, 'use_sim_time': 'False'}.items()
    )

    rviz_node = Node(
        package='rviz2', executable='rviz2',
        arguments=['-d', rviz_config]
    )

    # Configs
    lidar_filter_config = os.path.join(my_pkg_dir, 'config', 'lidar_filter.yaml')
    serial_port = LaunchConfiguration('serial_port')

    # Arguments
    declare_serial_port = DeclareLaunchArgument('serial_port', default_value='/dev/ttyUSB0')

    # Nodes
    lidar_node = Node(
        package='sllidar_ros2', executable='sllidar_node',
        parameters=[{'serial_port': serial_port, 'serial_baudrate': 115200, 
                     'frame_id': 'laser', 'scan_mode': 'Boost'}],
        remappings=[('scan', 'scan_raw')]
    )

    filter_node = Node(
        package='laser_filters', executable='scan_to_scan_filter_chain',
        parameters=[lidar_filter_config],
        remappings=[('scan', 'scan_raw'), ('scan_filtered', 'scan')]
    )

    # Static Transforms (Base -> Footprint, Base -> Laser)
    tf_footprint = Node(
        package='tf2_ros', executable='static_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'base_footprint', 'base_link']
    )
    
    tf_laser = Node(
        package='tf2_ros', executable='static_transform_publisher',
        arguments=['0.385', '0', '-0.08', '0', '0', '0', 'base_link', 'laser']
    )

    return LaunchDescription([
        declare_serial_port,
        lidar_node,
        filter_node,
        tf_footprint,
        tf_laser,
        hardware_launch,
        slam_node,
        rviz_node
    ])