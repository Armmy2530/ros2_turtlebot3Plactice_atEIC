from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument

def generate_launch_description():
    joy_dev = LaunchConfiguration('joy_dev')

    return LaunchDescription([
        DeclareLaunchArgument('joy_config', default_value='xbox'),
        DeclareLaunchArgument('joy_dev', default_value='/dev/input/js0'),

        DeclareLaunchArgument(
            'config_filepath',
            default_value='config/xbox_config.yaml',  # Default config file path
            description='Path to the configuration file for teleop_twist_joy'
        ),

        DeclareLaunchArgument(
            'config_filepath',
            default_value='config/xbox_config.yaml',  # Default config file path
            description='Path to the configuration file for teleop_twist_joy'
        ),

        Node(
            package='joy', executable='joy_node', name='joy_node',
            parameters=[{
                'dev': joy_dev,
                'deadzone': 0.1,
                'autorepeat_rate': 20.0,
            }]),

        Node(
            package='teleop_twist_joy',
            executable='teleop_node',
            name='teleop_twist_joy',
            output='screen',
            parameters=[LaunchConfiguration('config_filepath')],
        ),
    ])