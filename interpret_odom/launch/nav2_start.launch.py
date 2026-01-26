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
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')

    # Path to YAML containing ALL params (AMCL, MPPI, Planner, etc.)
    default_params_file = os.path.join(my_pkg_dir, 'config', 'nav2_params.yaml')

    # Launch Configurations
    slam_mode = LaunchConfiguration('slam')
    map_file = LaunchConfiguration('map')
    params_file = LaunchConfiguration('params_file')
    serial_port = LaunchConfiguration('serial_port')

    # Declare Arguments
    declare_slam_cmd = DeclareLaunchArgument('slam', default_value='False')
    declare_map_cmd = DeclareLaunchArgument('map', default_value=os.path.join(my_pkg_dir, 'maps', 'my_map.yaml'))
    declare_params_cmd = DeclareLaunchArgument('params_file', default_value=default_params_file)
    declare_serial_port_cmd = DeclareLaunchArgument('serial_port', default_value='/dev/ttyUSB0')

    lidar_filter_config = os.path.join(my_pkg_dir, 'config', 'lidar_filter.yaml')
    slam_toolbox_params = os.path.join(my_pkg_dir, 'config', 'mapper_params_online_async.yaml')
    rviz_config_dir = os.path.join(sll_dir, 'rviz', 'sllidar_ros2_final.rviz')

    # Hardware & Transforms
    lidar_node = Node(
        package='sllidar_ros2', executable='sllidar_node',
        parameters=[{'serial_port': serial_port, 'serial_baudrate': 115200, 'frame_id': 'laser', 'scan_mode': 'Boost'}],
        remappings=[('scan', 'scan_raw')]
    )

    filter_node = Node(
        package='laser_filters', executable='scan_to_scan_filter_chain',
        parameters=[lidar_filter_config],
        remappings=[('scan', 'scan_raw'), ('scan_filtered', 'scan')]
    )

    tf_footprint = Node(
        package='tf2_ros', executable='static_transform_publisher',
        arguments=['--x', '0', '--y', '0', '--z', '0', '--yaw', '0', '--pitch', '0', '--roll', '0', '--frame-id', 'base_footprint', '--child-frame-id', 'base_link']
    )
    
    tf_laser = Node(
        package='tf2_ros', executable='static_transform_publisher',
        arguments=['--x', '0.385', '--y', '0', '--z', '-0.08', '--yaw', '0', '--pitch', '0', '--roll', '0', '--frame-id', 'base_link', '--child-frame-id', 'laser']
    )

    # SLAM (Only runs if slam:=True)
    slam_toolbox_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(sll_dir, 'launch', 'online_async_launch.py')),
        condition=IfCondition(slam_mode),
        launch_arguments={'params_file': slam_toolbox_params, 'use_sim_time': 'False'}.items()
    )

    nav2_bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(nav2_bringup_dir, 'launch', 'bringup_launch.py')),
        launch_arguments={
            'slam': slam_mode,
            'map': map_file,
            'use_sim_time': 'False',
            'params_file': params_file,
            'autostart': 'True',
            'use_composition': 'True', # Combines nodes into one process to save CPU
        }.items()
    )

    rviz_node = Node(
        package='rviz2', executable='rviz2', arguments=['-d', rviz_config_dir]
    )

    return LaunchDescription([
        SetEnvironmentVariable('RCUTILS_LOGGING_BUFFERED_STREAM', '1'),
        declare_slam_cmd,
        declare_map_cmd,
        declare_params_cmd,
        declare_serial_port_cmd,
        lidar_node,
        filter_node,
        tf_footprint,
        tf_laser,
        slam_toolbox_node,
        nav2_bringup_launch,
        rviz_node
    ])