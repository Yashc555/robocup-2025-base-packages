#!/usr/bin/env python3
import os
from launch import LaunchDescription
from launch.actions import TimerAction, IncludeLaunchDescription, DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    """Launch multiple nodes that correspond to the requested ros2 run/launch commands.

    Starts:
    - serial_motor_demo/OpenLoop_controller
    - interpretOdom/tracking_dead_wheel_node
    - twist_mux (with params file from interpretOdom/config/twist_mux.yaml)
    - interpretOdom/serial_arbiter
    - serial_motor_demo/cmd_vel_to_pwm
    - includes sllidar_ros2's view_sllidar_a2m8_launch.py after a short delay
    """

    # Launch arguments to control delays (seconds) for each startup action
    declare_openloop_delay = DeclareLaunchArgument(
        'openloop_delay', default_value='0.0', description='Delay (sec) before starting OpenLoop_controller'
    )
    declare_tracking_delay = DeclareLaunchArgument(
        'tracking_delay', default_value='1.0', description='Delay (sec) before starting tracking_dead_wheel_node'
    )
    declare_twist_mux_delay = DeclareLaunchArgument(
        'twist_mux_delay', default_value='1.5', description='Delay (sec) before starting twist_mux'
    )
    declare_serial_arbiter_delay = DeclareLaunchArgument(
        'serial_arbiter_delay', default_value='0.0', description='Delay (sec) before starting serial_arbiter'
    )
    declare_cmd_vel_to_pwm_delay = DeclareLaunchArgument(
        'cmd_vel_to_pwm_delay', default_value='2.0', description='Delay (sec) before starting cmd_vel_to_pwm'
    )
    declare_sllidar_delay = DeclareLaunchArgument(
        'sllidar_delay', default_value='5.0', description='Delay (sec) before including sllidar launch'
    )

    openloop_delay = LaunchConfiguration('openloop_delay')
    tracking_delay = LaunchConfiguration('tracking_delay')
    twist_mux_delay = LaunchConfiguration('twist_mux_delay')
    serial_arbiter_delay = LaunchConfiguration('serial_arbiter_delay')
    cmd_vel_to_pwm_delay = LaunchConfiguration('cmd_vel_to_pwm_delay')
    sllidar_delay = LaunchConfiguration('sllidar_delay')

    # Locate twist_mux params file inside interpretOdom package so the launch is portable
    interpret_pkg = 'interpretOdom'
    try:
        twist_mux_params = os.path.join(
            get_package_share_directory(interpret_pkg),
            'config',
            'twist_mux.yaml',
        )
    except Exception:
        # Fall back to an absolute path if lookup fails (user originally specified an absolute path)
        twist_mux_params = '/home/sanjay/ros2_workspaces/base_ws/src/interpretOdom/config/twist_mux.yaml'

    # Nodes corresponding to the requested `ros2 run` commands
    openloop_node = Node(
        package='serial_motor_demo',
        executable='OpenLoop_controller',
        name='OpenLoop_controller',
        output='screen',
    )

    tracking_node = Node(
        package='interpretOdom',
        executable='tracking_dead_wheel_node',
        name='tracking_dead_wheel_node',
        output='screen',
    )

    twist_mux_node = Node(
        package='twist_mux',
        executable='twist_mux',
        name='twist_mux',
        output='screen',
        parameters=[twist_mux_params],
    )

    serial_arbiter_node = Node(
        package='interpretOdom',
        executable='serial_arbiter',
        name='serial_arbiter',
        output='screen',
    )

    cmd_vel_to_pwm_node = Node(
        package='serial_motor_demo',
        executable='cmd_vel_to_pwm',
        name='cmd_vel_to_pwm',
        output='screen',
    )

    # Include the sllidar launch file after a configurable delay so drivers come up cleanly
    sllidar_pkg = 'sllidar_ros2'
    delayed_actions = []
    try:
        sllidar_launch_path = os.path.join(
            get_package_share_directory(sllidar_pkg),
            'launch',
            'view_sllidar_a2m8_launch.py',
        )
        sllidar_include = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(sllidar_launch_path)
        )
        # Use the launch-configured delay (supports substitution)
        delayed_actions.append(TimerAction(period=sllidar_delay, actions=[sllidar_include]))
    except Exception:
        # If sllidar_ros2 isn't found in the environment, emit an informational log and continue.
        delayed_actions.append(LogInfo(msg=('sllidar_ros2 package not found; skipping sllidar include')))

    ld = LaunchDescription()

    # register launch arguments
    for arg in (
        declare_openloop_delay,
        declare_tracking_delay,
        declare_twist_mux_delay,
        declare_serial_arbiter_delay,
        declare_cmd_vel_to_pwm_delay,
        declare_sllidar_delay,
    ):
        ld.add_action(arg)

    # add nodes wrapped in TimerAction so they start after configured delays
    ld.add_action(TimerAction(period=openloop_delay, actions=[openloop_node]))
    ld.add_action(TimerAction(period=tracking_delay, actions=[tracking_node]))
    ld.add_action(TimerAction(period=twist_mux_delay, actions=[twist_mux_node]))
    ld.add_action(TimerAction(period=serial_arbiter_delay, actions=[serial_arbiter_node]))
    ld.add_action(TimerAction(period=cmd_vel_to_pwm_delay, actions=[cmd_vel_to_pwm_node]))

    # add delayed includes (if any)
    for a in delayed_actions:
        ld.add_action(a)

    return ld