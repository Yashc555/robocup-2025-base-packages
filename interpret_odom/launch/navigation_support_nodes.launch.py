from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

#!/usr/bin/env python3

def generate_launch_description():
    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        output='screen',
    )

    twist_mux_params_path = os.path.join(
        get_package_share_directory('interpret_odom'),
        'config',
        'twist_mux.yaml'
    )

    twist_mux_node = Node(
        package='twist_mux',
        executable='twist_mux',
        name='twist_mux',
        output='screen',
        arguments=[
            '--ros-args',
            '--params-file',
            twist_mux_params_path
        ],
    )

    cmd_vel_to_pwm_node = Node(
        package='serial_motor_demo',
        executable='cmd_vel_to_pwm',
        name='cmd_vel_to_pwm',
        output='screen',
    )

    serial_arbiter_node = Node(
        package='interpret_odom',
        executable='serial_arbiter',
        name='serial_arbiter',
        output='screen'
    )

    return LaunchDescription([
        joy_node,
        twist_mux_node,
        cmd_vel_to_pwm_node,
        serial_arbiter_node
    ])