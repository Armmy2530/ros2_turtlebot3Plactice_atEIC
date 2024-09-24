from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'config_filepath',
            default_value='config/joy_config.yaml',  # Default config file path
            description='Path to the configuration file for teleop_twist_joy'
        ),
        Node(
            package='teleop_twist_joy',
            executable='teleop_node',
            name='teleop_twist_joy',
            output='screen',
            parameters=[LaunchConfiguration('config_filepath')],
        ),
    ])