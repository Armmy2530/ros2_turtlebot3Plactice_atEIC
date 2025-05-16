#!/usr/bin/env python3
#
# Copyright 2019 ROBOTIS CO., LTD.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# Authors: Joep Tool

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.conditions import IfCondition
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import SetEnvironmentVariable
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit

def generate_launch_description():
    package_name = 'armmy_turtlebot3'
    omnibot_package_name = 'armmy_omnibot'

    armmy_turtlebot3_launch_file_dir = os.path.join(get_package_share_directory(package_name), 'launch')
    package_dir = os.path.join(get_package_share_directory(package_name))

    default_world = os.path.join(
        get_package_share_directory(package_name),
        'worlds',
        'turtlebot3_world_gz.world'
    )

    # Launch configuration variables
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    x_pose = LaunchConfiguration('x_pose', default='-2.0')
    y_pose = LaunchConfiguration('y_pose', default='1.0')
    gz_verbose = LaunchConfiguration('verbose', default='false')
    world_file = LaunchConfiguration('world', default=default_world)
    robot_model = LaunchConfiguration('robot_model', default=os.path.join(get_package_share_directory(omnibot_package_name), 'robots','newgz_omni_robot.urdf.xacro'))
    use_ros2_control = LaunchConfiguration('use_ros2_control', default='true')
    rviz_config_file = LaunchConfiguration('rviz_config_file')
    use_rviz = LaunchConfiguration('use_rviz')

    # Omni Controller parameters
    debug_cmd_vel = LaunchConfiguration('debug_cmd_vel', default='true')
    debug_odom = LaunchConfiguration('debug_odom', default='false')
    wheel_radius = LaunchConfiguration('wheel_radius', default='0.0247')
    wheel_offset = LaunchConfiguration('wheel_offset', default='0.067')
    init_theta = LaunchConfiguration('init_theta', default='0.0')

    declare_rviz_config_file_cmd = DeclareLaunchArgument(
        'rviz_config_file',
        default_value=os.path.join(
            package_dir, 'rviz', 'turtlebot3_rviz.rviz'),
        description='Full path to the RVIZ config file to use')
    
    declare_use_rviz_cmd = DeclareLaunchArgument(
        'use_rviz',
        default_value='True',
        description='Whether to start RVIZ')
    
    gzmodel_cmd  = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=os.path.join(get_package_share_directory(package_name),'models') + ':' + 
              os.path.join(get_package_share_directory(omnibot_package_name),'..')
    )

    gz_system_plugin_cmd  = SetEnvironmentVariable(
        name='GZ_SIM_SYSTEM_PLUGIN_PATH',
        value='/opt/ros/humble/lib/'
    )

    # Include the Gazebo launch file, provided by the ros_gz_sim package
    gazebo = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')]),
            launch_arguments={'gz_args': ['-r -v4 ', world_file ], 'on_exit_shutdown': 'true'}.items()
        )

    foxgloveBridge_cmd = Node(
            package='foxglove_bridge',
            executable='foxglove_bridge',
            name='foxglove_bridge',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
    )

    twist_mux_params = os.path.join(get_package_share_directory(package_name),'config','twist_mux','twist_mux.yaml')
    twist_mux = Node(
            package="twist_mux",
            executable="twist_mux",
            parameters=[twist_mux_params, {'use_sim_time': True}],
            remappings=[('/cmd_vel_out','/cmd_vel')]
        )
    
    robot_state_publisher_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(armmy_turtlebot3_launch_file_dir, 'robot_state_publisher.launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'robot_model': robot_model,
            'use_ros2_control': use_ros2_control,
        }.items()
    )
    
    # Run the spawner node from the ros_gz_sim package. The entity name doesn't really matter if you only have a single robot.
    spawn_entity = Node(package='ros_gz_sim', executable='create',
                        arguments=['-topic', 'robot_description',
                                    '-name', 'my_bot',
                                    '-x', x_pose,
                                    '-y', y_pose,
                                    '-z', '0.1'],
                        output='screen')

    forward_velocity_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["forward_velocity_controller"],
    )

    joint_broad_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_broad"],
    )

    bridge_params = os.path.join(get_package_share_directory(package_name),'config','new_gazebo','gz_bridge.yaml')
    ros_gz_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            '--ros-args',
            '-p',
            f'config_file:={bridge_params}',
        ]
    )

    armmy_omni_controller_cmd = Node(
            package='armmy_omni_controller',
            executable='omni_controller',
            parameters=[{'use_sim_time': use_sim_time,
                'debug_cmd_vel': debug_cmd_vel,
                'debug_odom'   : debug_odom,
                'wheel_radius' : wheel_radius,
                'wheel_offset' : wheel_offset,
                'init_x'       : x_pose,
                'init_y'       : y_pose,
                'init_theta'   : init_theta,
            }],
            condition=IfCondition(LaunchConfiguration('use_omni_controller', default='true'))
         )
    
    rviz_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(package_dir, 'launch', 'rviz.launch.py')),
        condition=IfCondition(use_rviz),
        launch_arguments={'rviz_config': rviz_config_file}.items())

    ld = LaunchDescription()

    # Add the commands to the launch description
    ld.add_action(gzmodel_cmd)
    ld.add_action(gz_system_plugin_cmd)
    ld.add_action(robot_state_publisher_cmd)
    ld.add_action(gazebo)
    ld.add_action(spawn_entity)
    ld.add_action(ros_gz_bridge)
    
    ld.add_action(twist_mux)
    ld.add_action(armmy_omni_controller_cmd)
    ld.add_action(forward_velocity_spawner)
    ld.add_action(joint_broad_spawner)
    
    ld.add_action(foxgloveBridge_cmd)
    ld.add_action(declare_use_rviz_cmd)
    ld.add_action(declare_rviz_config_file_cmd)
    ld.add_action(rviz_cmd)

    return ld
