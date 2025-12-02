import os
from launch import LaunchDescription
from launch.actions import TimerAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

#!/usr/bin/env python3



def generate_launch_description():
    # Node to run: ros2 run interpretOdom tracking_dead_wheel_node
    tracking_node = Node(
        package='interpretOdom',
        executable='tracking_dead_wheel_node',
        name='tracking_dead_wheel_node',
        output='screen'
    )

    # Include the sllidar launch after a 3 second delay
    sllidar_pkg = 'sllidar_ros2'
    sllidar_launch_path = os.path.join(
        get_package_share_directory(sllidar_pkg),
        'launch',
        'view_sllidar_a2m8_launch.py'
    )
    sllidar_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(sllidar_launch_path)
    )
    delayed_sllidar = TimerAction(period=3.0, actions=[sllidar_include])

    return LaunchDescription([
        tracking_node,
        delayed_sllidar,
    ])