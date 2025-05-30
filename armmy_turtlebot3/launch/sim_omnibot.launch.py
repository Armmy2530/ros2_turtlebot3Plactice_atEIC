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
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import SetEnvironmentVariable


def generate_launch_description():
    package_name = 'armmy_turtlebot3'
    omnibot_package_name = 'armmy_omnibot'

    package_dir = os.path.join(get_package_share_directory(package_name))
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')

    default_world = os.path.join(
        get_package_share_directory(package_name),
        'worlds',
        'turtlebot3_world.world'
    )

    defalut_gz_params = os.path.join(get_package_share_directory(
        package_name), 'config', 'gazebo', 'gazebo_params.yaml')

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    x_pose = LaunchConfiguration('x_pose', default='-2.0')
    y_pose = LaunchConfiguration('y_pose', default='1.0')
    gz_verbose = LaunchConfiguration('verbose', default='false')
    world_file = LaunchConfiguration('world', default=default_world)
    robot_model = LaunchConfiguration('robot_model', default=os.path.join(
        get_package_share_directory(omnibot_package_name), 'robots', 'omni_robot.urdf.xacro'))
    use_ros2_control = LaunchConfiguration('use_ros2_control', default='true')
    rviz_config_file = LaunchConfiguration('rviz_config_file')
    use_rviz = LaunchConfiguration('use_rviz')

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

    gzmodel_cmd = SetEnvironmentVariable(
        name='GAZEBO_MODEL_PATH',
        # Back one step for loading model in URDF File
        value=os.path.join(get_package_share_directory(
            omnibot_package_name), '..')
    )

    gzserver_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')
        ),
        launch_arguments={'world': world_file, 'verbose': gz_verbose,
                          'extra_gazebo_args': '--ros-args --params-file '+defalut_gz_params}.items()
    )

    gzclient_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')
        )
    )

    foxgloveBridge_cmd = Node(
        package='foxglove_bridge',
        executable='foxglove_bridge',
        name='foxglove_bridge',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
    )

    twist_mux_params = os.path.join(get_package_share_directory(
        package_name), 'config', 'twist_mux', 'twist_mux.yaml')
    twist_mux = Node(
        package="twist_mux",
        executable="twist_mux",
        parameters=[twist_mux_params, {'use_sim_time': True}],
        remappings=[('/cmd_vel_out', '/diff_cont/cmd_vel_unstamped')]
    )

    twist_stamper = Node(
        package='twist_stamper',
        executable='twist_stamper',
        parameters=[{'use_sim_time': use_sim_time}],
        remappings=[('/cmd_vel_in', '/diff_cont/cmd_vel_unstamped'),
                    ('/cmd_vel_out', '/diff_cont/cmd_vel')]
    )

    robot_state_publisher_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(package_dir, 'launch',
                         'robot_state_publisher.launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'robot_model': robot_model,
            'use_ros2_control': use_ros2_control,
        }.items()
    )

    spawn_robot_cmd = Node(package='gazebo_ros', executable='spawn_entity.py',
                           arguments=['-topic', 'robot_description',
                                      '-entity', 'my_bot',
                                      '-x', x_pose,
                                      '-y', y_pose,
                                      '-z', '0.01',
                                      ],
                           output='screen')

    delay_spawn_robot_cmd = TimerAction(
        period=5.0,
        actions=[spawn_robot_cmd]
    )

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

    armmy_omni_controller_cmd = Node(
        package='armmy_omni_controller',
        executable='omni_controller',
        parameters=[{'use_sim_time': use_sim_time,
                     'debug_cmd_vel': debug_cmd_vel,
                     'debug_odom': debug_odom,
                     'wheel_radius': wheel_radius,
                     'wheel_offset': wheel_offset,
                     'init_x': x_pose,
                     'init_y': y_pose,
                     'init_theta': init_theta,
                     }],
        condition=IfCondition(LaunchConfiguration(
            'use_omni_controller', default='true'))
    )

    rviz_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(package_dir, 'launch', 'rviz.launch.py')),
        condition=IfCondition(use_rviz),
        launch_arguments={'rviz_config': rviz_config_file}.items())

    ld = LaunchDescription()

    # Add the commands to the launch description
    ld.add_action(robot_state_publisher_cmd)
    ld.add_action(gzmodel_cmd)
    ld.add_action(gzserver_cmd)
    ld.add_action(gzclient_cmd)
    # ld.add_action(spawn_robot_cmd)
    ld.add_action(delay_spawn_robot_cmd)
    ld.add_action(forward_velocity_spawner)
    ld.add_action(joint_broad_spawner)
    ld.add_action(foxgloveBridge_cmd)
    ld.add_action(twist_mux)
    ld.add_action(twist_stamper)
    ld.add_action(armmy_omni_controller_cmd)
    ld.add_action(declare_rviz_config_file_cmd)
    ld.add_action(declare_use_rviz_cmd)
    ld.add_action(rviz_cmd)

    return ld
