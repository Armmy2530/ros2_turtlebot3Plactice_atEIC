from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Get the directory of the nav2_bringup package
    bringup_dir = get_package_share_directory('nav2_bringup')
    default_nav2_config = os.path.join(
        get_package_share_directory('armmy_turtlebot3'),
        'config', 'nav2' ,
        'nav2_RPP_tb3_gazebo_params.yaml'
    )

    # Create launch configuration variables
    use_sim_time = LaunchConfiguration('use_sim_time')
    map_yaml_file = LaunchConfiguration('map')
    nav2_config = LaunchConfiguration('nav2_config')

    default_map = get_package_share_directory('armmy_turtlebot3') + '/map/1_280924_tb3_world_carthographer/map_1727461600.yaml'

    # Declare launch arguments
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation clock if true'
    )

    declare_map_yaml_cmd = DeclareLaunchArgument(
        'map',
        default_value=default_map,
        description='Full path to the map YAML file'
    )

    declare_nav2_config_cmd = DeclareLaunchArgument(
        'nav2_config',
        default_value=default_nav2_config,
        description='Full path to the nav2 config YAML file'
    )

    # Include localization launch file
    localization_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(bringup_dir, 'launch', 'localization_launch.py')),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'map': map_yaml_file,
            'params_file' : nav2_config,
        }.items(),
    )

    # Include navigation launch file
    navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(bringup_dir, 'launch', 'navigation_launch.py')),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'params_file' : nav2_config,
        }.items()
    )

    # Create launch description
    ld = LaunchDescription()

    # Add declared launch arguments
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_map_yaml_cmd)
    ld.add_action(declare_nav2_config_cmd)

    # Add both localization and navigation launches
    ld.add_action(localization_launch)
    ld.add_action(navigation_launch)

    return ld
