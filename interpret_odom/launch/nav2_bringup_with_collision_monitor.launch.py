import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Paths to your specific files
    interpret_odom_share = get_package_share_directory('interpret_odom')
    nav2_bringup_share = get_package_share_directory('nav2_bringup')
    
    # Configuration variables
    # We use your existing paths from your command
    params_file = LaunchConfiguration('params_file')
    map_yaml_file = LaunchConfiguration('map')
    use_sim_time = LaunchConfiguration('use_sim_time')

    # 1. Declare Arguments (matching your manual command)
    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(interpret_odom_share, 'config', 'nav2_params.yaml'),
        description='Full path to the ROS2 parameters file to use')

    declare_map_yaml_cmd = DeclareLaunchArgument(
        'map',
        default_value='/home/sanjay/rcup_maps/mymap.yaml',
        description='Full path to map yaml file to load')

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='False',
        description='Use simulation (Gazebo) clock if true')

    # 2. Include the standard Nav2 Navigation Launch
    # This replaces: ros2 launch nav2_bringup navigation_launch.py ...
    navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(nav2_bringup_share, 'launch', 'navigation_launch.py')),
        launch_arguments={
            'map': map_yaml_file,
            'params_file': params_file,
            'use_sim_time': use_sim_time,
            'autostart': 'True',
            'use_composition': 'False',
            'use_respawn': 'True'
        }.items()
    )

    # 3. Add the Collision Monitor Node
    # This node reads the 'collision_monitor' block you added to your params file
    collision_monitor_node = Node(
        package='nav2_collision_monitor',
        executable='collision_monitor',
        name='collision_monitor',
        output='screen',
        emulate_tty=True,
        parameters=['/home/sanjay/ros2_workspaces/base_ws/src/interpret_odom/config/nav2_params.yaml'],
    )

    ld = LaunchDescription()
    ld.add_action(declare_params_file_cmd)
    ld.add_action(declare_map_yaml_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(navigation_launch)
    ld.add_action(collision_monitor_node)

    return ld