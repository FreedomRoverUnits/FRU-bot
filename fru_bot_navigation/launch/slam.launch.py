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
from launch import LaunchDescription
from launch import LaunchContext
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import EnvironmentVariable
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from nav2_common.launch import ReplaceString

def generate_launch_description():
    package_name = 'fru_bot_navigation'
    
    remappings_tf = [('/tf', 'tf'), ('/tf_static', 'tf_static'),
                     ('/goal_pose', 'goal_pose'),
                    ('/clicked_point', 'clicked_point'),
                    ('/initialpose', 'initialpose')]
    
    slam_launch_path = PathJoinSubstitution(
        [FindPackageShare('slam_toolbox'), 'launch', 'online_async_launch.py']
    )

    slam_config_path = PathJoinSubstitution(
        [FindPackageShare(package_name), 'config', 'slam.yaml']
    )

    navigation_launch_path = PathJoinSubstitution(
        [FindPackageShare('nav2_bringup'), 'launch', 'navigation_launch.py']
    )

    nav2_config_path = PathJoinSubstitution(
        [FindPackageShare(package_name), 'config', 'navigation.yaml']    
    )

    rviz_config_path = PathJoinSubstitution(
        [FindPackageShare(package_name), 'rviz', 'fru_bot_slam.rviz']
    )
    
    rviz_namespaced_config_path = PathJoinSubstitution(
        [FindPackageShare(package_name), 'rviz', 'fru_bot_nav2_namespaced.rviz']
    )
    
    lc = LaunchContext()
    ros_distro = EnvironmentVariable('ROS_DISTRO')
    slam_param_name = 'slam_params_file'
    if ros_distro.perform(lc) == 'foxy': 
        slam_param_name = 'params_file'
        
    # Declare Launch Arguments
    sim_la = DeclareLaunchArgument(
            name='sim', default_value='false', description='Enable use_sime_time to true'
        )
    rviz_la = DeclareLaunchArgument(
            name='use_rviz', default_value='false', description='Run rviz'
        )
    ns_la = DeclareLaunchArgument(
        name='ns', default_value='', description='Robot namespace'
        )
    use_ns_la = DeclareLaunchArgument(
        name='use_ns', default_value=str(False), description='Use a namespace'
        )
    remap_tf_la = DeclareLaunchArgument(
        name='remap_tf', default_value=str(False), 
        description='Remap tf topics to ns'
        )
    
    # Def Launch Contexts
    sim_lc = LaunchConfiguration('sim'); use_rviz_lc = LaunchConfiguration('use_rviz')
    use_ns_lc = LaunchConfiguration('use_ns'); remap_tf_lc = LaunchConfiguration('remap_tf')
    ns_lc = LaunchConfiguration('ns')
    
    namespaced_rviz_config_file = ReplaceString(
            source_file=rviz_namespaced_config_path,
            replacements={'<robot_namespace>': ('/', ns_lc)})
    
    return LaunchDescription([
        sim_la, rviz_la, ns_la, use_ns_la, remap_tf_la,
        
        # Nav2 servers w/o remappings
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(navigation_launch_path),
            launch_arguments={
                'use_sim_time': sim_lc,
                'params_file': nav2_config_path
            }.items(),
            condition=UnlessCondition(remap_tf_lc)
        ),
        #Nav2 servers w/ remappings
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(slam_launch_path),
            launch_arguments={
                'use_sim_time': sim_lc,
                slam_param_name: slam_config_path
            }.items(),
            condition=IfCondition(remap_tf_lc),
            # remappings=[('/tf', 'tf'), ('/tf_static', 'tf_static')]
        ),

        # SLAM w/o remappings
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(slam_launch_path),
            launch_arguments={
                'use_sim_time': sim_lc,
                slam_param_name: slam_config_path
            }.items(),
            condition=UnlessCondition(remap_tf_lc)
        ),
        # SLAM w/ remappings
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(slam_launch_path),
            launch_arguments={
                'use_sim_time': sim_lc,
                slam_param_name: slam_config_path
            }.items(),
            condition=IfCondition(remap_tf_lc),
            # remappings=[('/tf', 'tf'), ('/tf_static', 'tf_static')]
        ),
        
        # Rviz w/o remapping and namespace
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config_path],
            condition=IfCondition(PythonExpression(['"', use_rviz_lc, '"',
                                                    ' == "true" and not ', remap_tf_lc,
                                                    ' and not ', use_ns_lc])),
            parameters=[{'use_sim_time': sim_lc}]
        ),
        # Rviz w/o remapping but w/ namespace
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', namespaced_rviz_config_file],
            condition=IfCondition(PythonExpression(['"', use_rviz_lc, '"',
                                                    ' == "true" and not ', remap_tf_lc,
                                                    ' and ', use_ns_lc])),
            parameters=[{'use_sim_time': sim_lc}]
        ),
        # Rviz w/ remapping and namespace
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', namespaced_rviz_config_file],
            condition=IfCondition(PythonExpression(['"', use_rviz_lc, '"',
                                                    ' == "true" and ', remap_tf_lc,
                                                    ' and ', use_ns_lc])),
            parameters=[{'use_sim_time': sim_lc}],
            remappings=remappings_tf
        ),
    ])
