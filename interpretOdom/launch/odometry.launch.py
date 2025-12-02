from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Launch odome node
        Node(
            package='interpretOdom',
            executable='tracking_dead_wheel_node',
            name='tracking_dead_wheel_node'
        ),
        # Launch RViz with custom config
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', '/home/sanjay/ros2_workspaces/base_ws/src/interpretOdom/config/rvizwodom.rviz']
        ),
        # Launch robot_localization EKF node with custom config
        # Node(
        #     package='robot_localization',
        #     executable='ekf_node',
        #     name='ekf_filter_node',
        #     output='screen',
        #     parameters=['/home/sanjay/ros2_workspaces/base_ws/src/interpretOdom/config/dead_wheel_w_imu.yaml']
        # ),
    ])