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
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import SetEnvironmentVariable

def generate_launch_description():
    package_name = 'armmy_turtlebot3'

    armmy_turtlebot3_launch_file_dir = os.path.join(get_package_share_directory(package_name), 'launch')
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')

    default_world = os.path.join(
        get_package_share_directory(package_name),
        'worlds',
        'turtlebot3_world.world'
    )

    defalut_gz_params = os.path.join(get_package_share_directory(package_name),'config','gazebo','gazebo_params.yaml')

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    x_pose = LaunchConfiguration('x_pose', default='-2.0')
    y_pose = LaunchConfiguration('y_pose', default='1.0')
    gz_verbose = LaunchConfiguration('verbose', default='false')
    world_file = LaunchConfiguration('world', default=default_world)
    robot_model = LaunchConfiguration('robot_model', default=os.path.join(get_package_share_directory(package_name), 'urdf','tb3_custom','robot.urdf.xacro'))
    # robot_model = LaunchConfiguration('robot_model', default=os.path.join(get_package_share_directory(package_name), 'urdf','articubot','robot.urdf.xacro'))
    use_ros2_control = LaunchConfiguration('use_ros2_control', default='true')

    gzmodel_cmd  = SetEnvironmentVariable(
        name='GAZEBO_MODEL_PATH',
        value='/opt/ros/humble/share/turtlebot3_gazebo/models'
    )

    gzserver_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')
        ),
        launch_arguments={'world': world_file,'verbose': gz_verbose,'extra_gazebo_args':'--ros-args --params-file '+defalut_gz_params}.items()
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

    twist_mux_params = os.path.join(get_package_share_directory(package_name),'config','twist_mux','twist_mux.yaml')
    twist_mux = Node(
            package="twist_mux",
            executable="twist_mux",
            parameters=[twist_mux_params, {'use_sim_time': True}],
            remappings=[('/cmd_vel_out','/diff_cont/cmd_vel_unstamped')]
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

    # Run the spawner node from the gazebo_ros package. The entity name doesn't really matter if you only have a single robot.
    spawn_turtlebot_cmd = Node(package='gazebo_ros', executable='spawn_entity.py',
                        arguments=['-topic', 'robot_description',
                                    '-entity', 'my_bot',
                                    '-x', x_pose,
                                    '-y', y_pose,
                                    '-z','0.01'],
                        output='screen')
    
    diff_drive_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["diff_cont"],
    )
    
    arm_joint_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["forward_position_controller"],
    )

    joint_broad_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_broad"],
    )

    ld = LaunchDescription()

    # Add the commands to the launch description
    ld.add_action(robot_state_publisher_cmd)
    ld.add_action(gzmodel_cmd)
    ld.add_action(gzserver_cmd)
    ld.add_action(gzclient_cmd)
    ld.add_action(spawn_turtlebot_cmd)
    ld.add_action(diff_drive_spawner)
    ld.add_action(arm_joint_spawner)
    ld.add_action(joint_broad_spawner)
    ld.add_action(foxgloveBridge_cmd)
    ld.add_action(twist_mux)

    return ld
