import os

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='serial_motor_demo',
            executable='receive_IMU',
            name='receive_imu',
            output='screen'
        ),
        Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            output='screen'
        ),
        Node(
            package='serial_motor_demo',
            executable='PID_Controller',
            name='pid_controller',
            output='screen'
        ),
        # Node(
        #     package='serial_motor_demo',
        #     executable='raw_odom_publisher',
        #     name='raw_odom_publisher',
        #     output='screen'
        # ),
        # Node(
        #     package='serial_motor_demo',
        #     executable='EKF',
        #     name='ekf_odometry',
        #     output='screen'
        # ),
        # Dynamic TF Broadcaster
        # Node(
        #     package='serial_motor_demo',
        #     executable='dynamic_tf_broadcaster',
        #     name='dynamic_tf_broadcaster',
        #     output='screen'
        # ),
    ])
