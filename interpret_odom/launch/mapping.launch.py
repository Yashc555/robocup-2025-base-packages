import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    sll_dir = get_package_share_directory('sllidar_ros2')

    slam_params_file = os.path.join(sll_dir, 'config', 'mapper_params_online_async.yaml')
    
    slam_toolbox_launch_file = os.path.join(sll_dir, 'launch', 'online_async_launch.py')
    
    slam_toolbox_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(slam_toolbox_launch_file),
        launch_arguments={
            'use_sim_time': 'false',
            'params_file': slam_params_file
        }.items(),
    )


    return LaunchDescription([
        SetEnvironmentVariable('RCUTILS_LOGGING_BUFFERED_STREAM', '1'),

        slam_toolbox_node, 

    ])