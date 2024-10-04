from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    config_file_path = os.path.join(
        get_package_share_directory('armmy_turtlebot3'),
        'config','slam','slam_toolbox_params.yaml'
    )

    return LaunchDescription([
        Node(
            package='slam_toolbox',
            executable='sync_slam_toolbox_node',
            name='slam_toolbox',
            output='screen',
            parameters=[config_file_path],  # Load parameters from YAML
        ),
    ])
