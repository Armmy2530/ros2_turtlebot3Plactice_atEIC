from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument

def generate_launch_description():
    useSimTime = LaunchConfiguration('use_sim_time', default='false')

    config_dir = os.path.join(
        get_package_share_directory('armmy_turtlebot3'),
        'config'
    )
    config_file = 'turtlebot3_cartographer.lua'

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation time if true'
        ),

        Node(
            package='cartographer_ros',
            executable='cartographer_node',
            name='cartographer_node',
            output='screen',
            parameters=[{'use_sim_time': useSimTime}],
            arguments=[
                '-configuration_directory', config_dir,
                '-configuration_basename', config_file
            ]
        ),

        # Occupancy grid node
        Node(
            package='cartographer_ros',
            executable='cartographer_occupancy_grid_node',
            name='occupancy_grid_node',
            output='screen',
            parameters=[{'use_sim_time': useSimTime}],
            remappings=[('/map', '/map')],
        ),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'use_sim_time': useSimTime}],
        ),
    ])
