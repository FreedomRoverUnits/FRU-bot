# Copyright (c) 2021 Juan Miguel Jimeno
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http:#www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os
from ament_index_python import get_package_prefix
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_sim_time = True
    
    robot_name = "FRU_bot"
    robot_idx = '0'
    robot_namespace = robot_name + robot_idx
    
    description_share_path = os.pathsep + os.path.join(get_package_prefix('fru_bot_description'), 'share')

    ekf_config_path = PathJoinSubstitution(
        [FindPackageShare("fru_bot_base"), "config", "ekf_default.yaml"]
    )

    world_path = PathJoinSubstitution(
        [FindPackageShare("fru_bot_gazebo"), "worlds", "playground.world"]
    )

    description_launch_path = PathJoinSubstitution(
        [FindPackageShare('fru_bot_description'), 'launch', 'description.launch.py']
    )
    
    os.environ['GAZEBO_MODEL_PATH'] = os.environ['GAZEBO_MODEL_PATH'] + description_share_path if \
        'GAZEBO_MODEL_PATH' in os.environ else description_share_path

    return LaunchDescription([
        DeclareLaunchArgument(
            name='world', 
            default_value=world_path,
            description='Gazebo world'
        ),
        
        DeclareLaunchArgument(
            name='namespace',
            default_value=robot_namespace,
            description='Robot namespace'
        ),
        
        DeclareLaunchArgument(
            name='idx',
            default_value=robot_idx,
            description='Robot index'
        ),
        
        ExecuteProcess(
            cmd=['gazebo', '-s', 'libgazebo_ros_factory.so',  '-s', 'libgazebo_ros_init.so', LaunchConfiguration('world')],
            output='screen'
        ),

        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            name='urdf_spawner',
            output='screen',
            namespace=LaunchConfiguration('namespace'),
            arguments=[
                "-topic", "robot_description", 
                "-entity", LaunchConfiguration('namespace'),
                "-robot_namespace", LaunchConfiguration('namespace')]
        ),

        Node(
            package='fru_bot_gazebo',
            executable='command_timeout.py',
            name='command_timeout',
            namespace=LaunchConfiguration('namespace')
        ),

        Node(
            package='robot_localization',
            executable='ekf_node',
            namespace=LaunchConfiguration('namespace'),
            output='screen',
            parameters=[
                {'use_sim_time': use_sim_time,
                 'imu0' : [LaunchConfiguration('namespace'), '/imu/data'],
                 'odom0' : [LaunchConfiguration('namespace'), '/odom/unfiltered']
                 }, 
                ekf_config_path
            ],
            remappings=[("odometry/filtered", "odom"),
                        ('/tf', 'tf'), 
                        ('/tf_static', 'tf_static')],
            arguments=["-robot_namespace", LaunchConfiguration('namespace')]
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(description_launch_path),
            launch_arguments={
                'use_sim_time': str(use_sim_time),
                'publish_joints': 'false',
                'namespace' : LaunchConfiguration('namespace'),
                'idx' : LaunchConfiguration('idx')
            }.items()
        ),
    ])

#sources: 
#https://navigation.ros.org/setup_guides/index.html#
#https://answers.ros.org/question/374976/ros2-launch-gazebolaunchpy-from-my-own-launch-file/
#https://github.com/ros2/rclcpp/issues/940