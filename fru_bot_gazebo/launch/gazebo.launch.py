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
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution, PythonExpression, TextSubstitution
from launch.actions import SetLaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from nav2_common.launch import RewrittenYaml
from launch.conditions import IfCondition, UnlessCondition

def generate_launch_description():
    use_sim_time = True
    
    robot_name = "FRU_bot"
    
    remappings = [('odometry/filtered', 'odom')]
    remappings_tf = [('odometry/filtered', 'odom'), ('/tf', 'tf'), ('/tf_static', 'tf_static')]
    
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
            name='use_ns', default_value=str(False), description='Use a namespace'
        )
    frame_idx_launch_arg = DeclareLaunchArgument(
            name='frame_idx', default_value="", description='Robot index'
        )
    ns_launch_arg = DeclareLaunchArgument(
            name='ns', default_value="", description='Robot namespace'
        )
    use_rviz_launch_arg = DeclareLaunchArgument(
            name='use_rviz', default_value='false', description='Start rviz'
        )
    remap_tf_launch_arg = DeclareLaunchArgument(
        name='remap_tf', default_value=str(False), description='Remap tf topics to ns'
        )
    
    # Launch config defs
    use_ns_lc = LaunchConfiguration('use_ns'); frame_idx_lc = LaunchConfiguration('frame_idx')  
    use_rviz_lc = LaunchConfiguration('use_rviz'); remap_tf_lc = LaunchConfiguration('remap_tf')
    
    namespace_lc = PythonExpression(['"', LaunchConfiguration('ns'), '"', ' if ', use_ns_lc, ' else ""'])
    
    ekf_substitutions = {
                'use_sim_time': str(use_sim_time),
                'base_link_frame' : ['base_footprint', frame_idx_lc],
                'odom_frame' : ['odom', frame_idx_lc],
                'world_frame' : ['odom', frame_idx_lc],
                'odom0' : ['/', namespace_lc, '/', TextSubstitution(text='odom/unfiltered')],
                'imu0' : ['/', namespace_lc, '/', TextSubstitution(text='imu/data')]
                }
    configured_ekf_params = RewrittenYaml(source_file=ekf_config_path, param_rewrites=ekf_substitutions, 
                                          root_key=namespace_lc, convert_types=True)
    return LaunchDescription([
        world_launch_arg, use_ns_launch_arg, frame_idx_launch_arg,
        ns_launch_arg, use_rviz_launch_arg, ns_launch_arg, 
        remap_tf_launch_arg,
        
        ExecuteProcess(
            cmd=['gazebo', '-s', 'libgazebo_ros_factory.so',  '-s', 'libgazebo_ros_init.so', LaunchConfiguration('world')],
            output='screen'
        ),
        
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            name='urdf_spawner',
            output='screen',
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
                'ns' : namespace_lc,
                'prefix' : frame_idx_lc,
                'use_ns' : use_ns_lc,
                'use_rviz' : use_rviz_lc,
                'remap_tf' : remap_tf_lc
            }.items()
        ),
        
        # EKF Node w/o tf transforms
        Node(
            package='robot_localization', executable='ekf_node', name='ekf_filter_node',
            namespace=namespace_lc, output='screen',
            condition=UnlessCondition(remap_tf_lc),
            parameters=[
                ekf_config_path
            ],
            remappings=remappings,
            arguments=["-robot_namespace ", namespace_lc]
        ),
        # EKF Node w/ tf transforms
        Node(
            package='robot_localization', executable='ekf_node', name='ekf_filter_node',
            namespace=namespace_lc, output='screen',
            condition=IfCondition(remap_tf_lc),
            parameters=[
                configured_ekf_params
            ],
            remappings=remappings_tf,
            arguments=["-robot_namespace ", namespace_lc]
        )
    ])

#sources: 
#https://navigation.ros.org/setup_guides/index.html#
#https://answers.ros.org/question/374976/ros2-launch-gazebolaunchpy-from-my-own-launch-file/
#https://github.com/ros2/rclcpp/issues/940