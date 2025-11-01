from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_node',
            output='screen',
            parameters=['/home/souri/LIDAR_WS/src/sllidar_ros2/config/ekf.yaml'],
        ),
    ])
