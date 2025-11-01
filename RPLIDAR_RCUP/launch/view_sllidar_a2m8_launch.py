import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Define configurations
    channel_type = LaunchConfiguration('channel_type', default='serial')
    serial_port = LaunchConfiguration('serial_port', default='/dev/ttyUSB0')
    serial_baudrate = LaunchConfiguration('serial_baudrate', default='115200')
    frame_id = LaunchConfiguration('frame_id', default='laser')
    inverted = LaunchConfiguration('inverted', default='false')
    angle_compensate = LaunchConfiguration('angle_compensate', default='true')
    # Available scan modes: 'Standard', 'Express', 'Boost', 'Sensitivity'
    scan_mode = LaunchConfiguration('scan_mode', default='Boost')

    # RViz config path
    rviz_config_dir = os.path.join(
        get_package_share_directory('sllidar_ros2'),
        'rviz',
        'sllidar_ros2_final.rviz'
    )

    # Paths to additional launch files
    # nav2_launch_dir = os.path.join(
    #     get_package_share_directory('nav2_bringup'),
    #     'launch',
    #     'navigation_launch.py'
    # )
    slam_toolbox_launch_file = os.path.join(
        get_package_share_directory('sllidar_ros2'),
        'launch',
        'online_async_launch.py'
    )


    # Path to SLAM parameters file
    slam_params_file = os.path.join(
        get_package_share_directory('sllidar_ros2'),
        'config',
        'mapper_params_online_async.yaml'
    )

    # Path to Laser Filters YAML config
    filter_config_file = os.path.join(
        get_package_share_directory('sllidar_ros2'),
        'config',
        'lidar_filter.yaml'
    )
    
    ekf_config_file = os.path.join(
        get_package_share_directory('sllidar_ros2'),  # Replace with your actual package
        'config',
        'ekf.yaml'
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

        # LiDAR node (publishes raw data to /scan_raw)
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
                'queue_size': 10
            }],
            remappings=[('scan', 'scan_raw')],  # Publish full 360° scan as /scan_raw
            output='screen'
        ),

        # Laser Scan Filter Node (filters to 270° and publishes on /scan)
        Node(
            package='laser_filters',
            executable='scan_to_scan_filter_chain',
            name='scan_to_scan_filter_chain',
            parameters=[filter_config_file],
            remappings=[('scan', 'scan_raw'), ('scan_filtered', 'scan_raw_filtered')],
            output='screen'
        ),
        
        Node(
            package='serial_motor_demo',  # 👉 replace this with your actual package name
            executable='scan_timestamp_fixer',
            name='scan_timestamp_fixer',
            output='screen'
        ),

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
        
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[ekf_config_file],
            remappings=[('/odometry/filtered', '/odom')]
        ),

        # SLAM Toolbox
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(slam_toolbox_launch_file),
            launch_arguments={
                'use_sim_time': 'false',
                'params_file': slam_params_file
            }.items()
        ),
        # # Nav2 bringup
        # IncludeLaunchDescription(
        #     PythonLaunchDescriptionSource(nav2_launch_dir),
        #     launch_arguments={'use_sim_time': 'false'}.items()
        # ),

        # RViz node
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_dir],
            output='screen'
        ),
    ])

# import os
# from launch import LaunchDescription
# from launch.actions import DeclareLaunchArgument
# from launch_ros.actions import Node
# from launch.substitutions import LaunchConfiguration
# from ament_index_python.packages import get_package_share_directory

# def generate_launch_description():
#     # LiDAR parameters
#     serial_port = LaunchConfiguration('serial_port', default='/dev/ttyUSB0')
#     serial_baudrate = LaunchConfiguration('serial_baudrate', default='115200')
#     frame_id = LaunchConfiguration('frame_id', default='laser')
#     scan_mode = LaunchConfiguration('scan_mode', default='Standard')

#     # Get path to laser filter config
#     filter_config_file = os.path.join(
#         get_package_share_directory('sllidar_ros2'),
#         'config',
#         'lidar_filter.yaml'
#     )

#     return LaunchDescription([
#         # Launch arguments
#         DeclareLaunchArgument('serial_port', default_value=serial_port, description='LiDAR USB port'),
#         DeclareLaunchArgument('serial_baudrate', default_value=serial_baudrate, description='Baudrate'),
#         DeclareLaunchArgument('frame_id', default_value=frame_id, description='Frame ID'),
#         DeclareLaunchArgument('scan_mode', default_value=scan_mode, description='Scan Mode'),

#         # LiDAR node (publishes raw /scan_raw)
#         Node(
#             package='sllidar_ros2',
#             executable='sllidar_node',
#             name='sllidar_node',
#             parameters=[{
#                 'serial_port': serial_port,
#                 'serial_baudrate': serial_baudrate,
#                 'frame_id': frame_id,
#                 'scan_mode': scan_mode
#             }],
#             remappings=[('scan', 'scan_raw')],
#             output='screen'
#         ),

#         # Laser Filters Node (limits scan to 180°)
#         Node(
#             package='laser_filters',
#             executable='scan_to_scan_filter_chain',
#             name='scan_to_scan_filter_chain',
#             parameters=[filter_config_file],
#             remappings=[('scan', 'scan_raw'), ('scan_filtered', 'scan')],
#             output='screen'
#         )
#     ])
    