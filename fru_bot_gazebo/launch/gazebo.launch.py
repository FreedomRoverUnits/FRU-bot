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
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution, PythonExpression
from launch.actions import SetLaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition, UnlessCondition

def generate_launch_description():
    use_sim_time = True
    
    robot_name = "FRU_bot"
    robot_idx = ''
    use_ns=str(False)
    
    remappings = [("odometry/filtered", "odom")]
    
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
    
    # Launch arg defs
    os.environ['GAZEBO_MODEL_PATH'] = os.environ['GAZEBO_MODEL_PATH'] + description_share_path if \
        'GAZEBO_MODEL_PATH' in os.environ else description_share_path
    
    world_launch_arg = DeclareLaunchArgument(
            name='world', default_value=world_path, description='Gazebo world'
        )
    use_ns_launch_arg = DeclareLaunchArgument(
            name='use_ns', default_value=use_ns, description='Use a namespace'
        )
    idx_launch_arg = DeclareLaunchArgument(
            name='idx', default_value=robot_idx, description='Robot index'
        )
    ns_launch_arg = DeclareLaunchArgument(
            name='ns', default_value=[robot_name, LaunchConfiguration('idx')], 
            description='Robot namespace'
        )
    use_rviz_launch_arg = DeclareLaunchArgument(
            name='use_rviz', default_value='false', description='Start rviz'
        )
    
    # Launch config defs
    use_ns_lc = LaunchConfiguration('use_ns'); idx_lc = LaunchConfiguration('idx')  
    use_rviz_lc = LaunchConfiguration('use_rviz')
    
    namespace_lc = PythonExpression(['"', LaunchConfiguration('ns'), '"', ' if ', use_ns_lc, ' else ""'])
    return LaunchDescription([
        world_launch_arg, use_ns_launch_arg, idx_launch_arg,
        ns_launch_arg, use_rviz_launch_arg,
        
        ExecuteProcess(
            cmd=['gazebo', '-s', 'libgazebo_ros_factory.so',  '-s', 'libgazebo_ros_init.so', LaunchConfiguration('world')],
            output='screen'
        ),
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            name='urdf_spawner',
            output='screen',
            # namespace=LaunchConfiguration('namespace'),
            arguments=[
                "-topic", [namespace_lc, "/robot_description"], 
                "-entity", namespace_lc,
                "-robot_namespace", namespace_lc],
        ),
        Node(
            package='fru_bot_gazebo',
            executable='command_timeout.py',
            name='command_timeout',
            namespace=namespace_lc
        ),
        
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(description_launch_path),
            launch_arguments={
                'use_sim_time': str(use_sim_time),
                'publish_joints': 'false',
                'namespace' : namespace_lc,
                'idx' : idx_lc,
                'use_ns' : use_ns_lc,
                'use_rviz' : use_rviz_lc
            }.items()
        ),
        
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            namespace=namespace_lc,
            output='screen',
            parameters=[
                ekf_config_path,
                {
                'use_sim_time': use_sim_time,
                'odom0' : 'odom/unfiltered',
                'odom0_config' : [False, False, False,
                                    False, False, False,
                                    True, True, False, 
                                    False, False, True,
                                    False, False, False],
                'imu0' : 'imu/data',
                'imu0_config' : [False, False, False,
                                    False, False, False,
                                    False, False, False, 
                                    False, False, True,
                                    False, False, False],
                }
            ],
            remappings=remappings,
            arguments=["-robot_namespace ", namespace_lc]
        ),
    ])

#sources: 
#https://navigation.ros.org/setup_guides/index.html#
#https://answers.ros.org/question/374976/ros2-launch-gazebolaunchpy-from-my-own-launch-file/
#https://github.com/ros2/rclcpp/issues/940