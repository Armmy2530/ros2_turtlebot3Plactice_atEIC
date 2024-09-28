from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration
from launch.actions import IncludeLaunchDescription
import os

def generate_launch_description():
    default_map = get_package_share_directory('armmy_turtlebot3') + '/maps/1_280924_tb3_world_carthographer/map_1727461600.yaml'
    map_file = LaunchConfiguration('use_sim_time', default=default_map)
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    return LaunchDescription([
        IncludeLaunchDescription(
            os.path.join(get_package_share_directory('nav2_bringup'), 'launch', 'bringup_launch.py'),
            launch_arguments={
                'use_sim_time': use_sim_time,
                'autostart': 'true',
                'map' : map_file,
            }.items()
        )
    ])
