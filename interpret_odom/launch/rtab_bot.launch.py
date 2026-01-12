import os

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Node(
        #     package='serial_motor_demo',
        #     executable='receive_IMU',
        #     name='receive_imu',
        #     output='screen'
        # ),
        Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            output='screen'
        ),
        Node(
            package='serial_motor_demo',
            executable='OpenLoop_controller',
            name='OpenLoop_controller',
            output='screen'
        ),
    ])
