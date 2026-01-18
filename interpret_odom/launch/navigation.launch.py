import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    my_pkg_dir = get_package_share_directory('interpret_odom')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    sll_dir = get_package_share_directory('sllidar_ros2')

    # Paths
    default_map = os.path.join(my_pkg_dir, 'maps', 'my_map.yaml')
    default_params = os.path.join(my_pkg_dir, 'config', 'nav2_params.yaml')
    rviz_config = os.path.join(sll_dir, 'rviz', 'sllidar_ros2_final.rviz')

    # Launch Configs
    map_file = LaunchConfiguration('map')
    params_file = LaunchConfiguration('params_file')

    # Args
    declare_map = DeclareLaunchArgument('map', default_value=default_map)
    declare_params = DeclareLaunchArgument('params_file', default_value=default_params)

    # Include Hardware
    hardware_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(my_pkg_dir, 'launch', 'hardware.launch.py'))
    )

    # Nav2 Bringup (AMCL + Planner + Controller + BT)
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(nav2_bringup_dir, 'launch', 'bringup_launch.py')),
        launch_arguments={
            'slam': 'False',
            'map': map_file,
            'use_sim_time': 'False',
            'params_file': params_file,
            'autostart': 'True',
            'use_composition': 'True'
        }.items()
    )

    rviz_node = Node(
        package='rviz2', executable='rviz2',
        arguments=['-d', rviz_config]
    )

    return LaunchDescription([
        SetEnvironmentVariable('RCUTILS_LOGGING_BUFFERED_STREAM', '1'),
        declare_map,
        declare_params,
        hardware_launch,
        nav2_launch,
        rviz_node
    ])