from launch import LaunchDescription
from launch_ros.actions import Node
import os
config_path = os.path.join(os.path.expanduser("~"), "LIDAR_WS", "src", "sllidar_ros2", "config", "ekf_final.yaml")

ekf_node = Node(
    package="robot_localization",
    executable="ekf_node",
    name="ekf_node",
    output="screen",
    parameters=[config_path],
)

def generate_launch_description():
    return LaunchDescription([ekf_node])

