import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node

def generate_launch_description():
    my_pkg_dir = get_package_share_directory('interpret_odom')
    sll_dir = get_package_share_directory('sllidar_ros2')

    channel_type = LaunchConfiguration('channel_type', default='serial')
    serial_port = LaunchConfiguration('serial_port', default='/dev/lidar')
    serial_baudrate = LaunchConfiguration('serial_baudrate', default='115200')
    frame_id = LaunchConfiguration('frame_id', default='laser')
    inverted = LaunchConfiguration('inverted', default='true')
    angle_compensate = LaunchConfiguration('angle_compensate', default='true')
    scan_mode = LaunchConfiguration('scan_mode', default='Boost')
    slam_toolbox_launch_file = os.path.join(
        get_package_share_directory('sllidar_ros2'),
        'launch',
        'online_async_launch.py'
    )
    slam_params_file = os.path.join(
        get_package_share_directory('sllidar_ros2'),
        'config',
        'mapper_params_online_async.yaml'
    )

    filter_config_file = os.path.join(
        get_package_share_directory('sllidar_ros2'),
        'config',
        'lidar_filter.yaml'
    )
  

    declare_map_cmd = DeclareLaunchArgument('map', default_value=os.path.join(my_pkg_dir, 'maps', 'my_map.yaml'))

    rviz_config_dir = os.path.join(sll_dir, 'rviz', 'sllidar_ros2_final.rviz')



    # tf_footprint = Node(
    #     package='tf2_ros', executable='static_transform_publisher',
    #     arguments=['--x', '0', '--y', '0', '--z', '0', '--yaw', '0', '--pitch', '0', '--roll', '0', '--frame-id', 'base_footprint', '--child-frame-id', 'base_link']
    # )

    tf_footprint = Node(
    package='tf2_ros', executable='static_transform_publisher',
    # Swapped frame-id and child-frame-id
    arguments=['--x', '0', '--y', '0', '--z', '0', '--yaw', '0', '--pitch', '0', '--roll', '0', 
               '--frame-id', 'base_link', '--child-frame-id', 'base_footprint']
)
    
    tf_laser = Node(
        package='tf2_ros', executable='static_transform_publisher',
        arguments=['--x', '0.385', '--y', '0', '--z', '-0.08', '--yaw', '0', '--pitch', '0', '--roll', '0', '--frame-id', 'base_link', '--child-frame-id', 'laser']
    )

    # SLAM (Only runs if slam:=True)
    slam_toolbox_node =         IncludeLaunchDescription(
            PythonLaunchDescriptionSource(slam_toolbox_launch_file),
            launch_arguments={
                'use_sim_time': 'false',
                'params_file': slam_params_file
            }.items()
        )

    filter_node=Node(
            package='laser_filters',
            executable='scan_to_scan_filter_chain',
            name='scan_to_scan_filter_chain',
            parameters=[filter_config_file],
            # CHANGED: 'scan_filtered' is now remapped to 'scan'
            remappings=[
                ('scan', 'scan_raw'), 
                ('scan_filtered', 'scan')
            ],
            output='screen'
        )

    lidar_node =         Node(
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
                'queue_size': 10
            }],
            remappings=[('scan', 'scan_raw')],
            output='screen'
        )
    
    rviz_node = Node(
        package='rviz2', executable='rviz2', arguments=['-d', rviz_config_dir]
    )

    return LaunchDescription([
        SetEnvironmentVariable('RCUTILS_LOGGING_BUFFERED_STREAM', '1'),
        declare_map_cmd,
        lidar_node,
        filter_node,
        tf_footprint,
        tf_laser,
        slam_toolbox_node,
        rviz_node
    ])