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
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution, PythonExpression
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from nav2_common.launch import ReplaceString


def generate_launch_description():
    robot_name = "FRU_bot"
    package_name = 'fru_bot_description'
    use_ns=str(False); idx=''
 
    remappings= [('/tf', 'tf'), ('/tf_static', 'tf_static')]

    
    urdf_path = PathJoinSubstitution(
        [FindPackageShare(package_name), "urdf/robots", f"{robot_name}.urdf.xacro"]
    )

    rviz_config_path = PathJoinSubstitution(
        [FindPackageShare(package_name), 'rviz', 'description.rviz']
    )
    
    # Launch arg defs
    urdf_launch_arg = DeclareLaunchArgument(
            name='urdf', default_value=urdf_path, description='URDF path'
        )
    use_rviz_launch_arg = DeclareLaunchArgument(
            name='use_rviz', default_value='false', description='Start rviz'
        )
    pub_jnts_launch_arg = DeclareLaunchArgument(
            name='publish_joints', default_value='true', description='Launch joint_states_publisher'
        )
    use_sim_time_launch_arg = DeclareLaunchArgument(
            name='use_sim_time', default_value='false', description='Using sim time'
        )
    use_ns_launch_arg = DeclareLaunchArgument(
            name='use_ns', default_value=use_ns, description='Use a namespace'
        )
    ns_launch_arg = DeclareLaunchArgument(
            name='ns', default_value = '', description='Robot namespace'
        )
    use_prefix_launch_arg = DeclareLaunchArgument(
        name='prefix', default_value='', description='Frame prefix'
    )
    remap_tf_launch_arg = DeclareLaunchArgument(
        name='remap_tf', default_value='True', 
        description='Remap tf topics to ns'
    )
    # Launch Configuration Variables
    urdf_lc = LaunchConfiguration('urdf'); pub_jnts_lc=LaunchConfiguration('publish_joints')
    use_rviz_lc = LaunchConfiguration('use_rviz'); use_sim_time_lc = LaunchConfiguration('use_sim_time')
    use_ns_lc = LaunchConfiguration('use_ns'); prefix_lc = LaunchConfiguration('prefix')
    remap_tf_lc = LaunchConfiguration('remap_tf')
    
    namespace_lc = PythonExpression(['"', LaunchConfiguration('ns'), '"', ' if ', use_ns_lc, ' else ""'])
    return LaunchDescription([
        urdf_launch_arg, use_rviz_launch_arg, pub_jnts_launch_arg, use_sim_time_launch_arg,
        use_ns_launch_arg, ns_launch_arg, ns_launch_arg, use_prefix_launch_arg, 
        remap_tf_launch_arg,
        
        ## Jnt State Pub Node w/o and w/ tf transforms
        Node(
            package='joint_state_publisher', executable='joint_state_publisher',
            name='joint_state_publisher', namespace=namespace_lc,
            condition=IfCondition(
                PythonExpression(['"', pub_jnts_lc, '"', ' == "true" and not ', remap_tf_lc])),
        ),
        Node(
            package='joint_state_publisher', executable='joint_state_publisher',
            name='joint_state_publisher', namespace=namespace_lc,
            condition=IfCondition(
                PythonExpression(['"', pub_jnts_lc, '"', ' == "true" and ', remap_tf_lc])),
            remappings=remappings
        ),
        
        ## Robot State Pub Node w/o and w/ tf transforms
        Node(
            package='robot_state_publisher', executable='robot_state_publisher',
            name='robot_state_publisher', namespace=namespace_lc,
            condition=UnlessCondition(remap_tf_lc),
            output='screen', parameters=[{
                'use_sim_time': use_sim_time_lc,
                'robot_description': Command(
                ['xacro ', urdf_lc, ' ns:=', namespace_lc, ' ns_idx:=', prefix_lc])
                }]
        ),
        Node(
            package='robot_state_publisher', executable='robot_state_publisher',
            name='robot_state_publisher', namespace=namespace_lc,
            condition=IfCondition(remap_tf_lc),
            output='screen', parameters=[{
                'use_sim_time': use_sim_time_lc,
                'robot_description': Command(
                ['xacro ', urdf_lc, ' ns:=', namespace_lc, ' ns_idx:=', prefix_lc, ' remap:=True'])
                }],
            remappings=remappings
        ),
        
        ## Rviz w/o and w/ tf transforms
        Node(
            package='rviz2', executable='rviz2', name='rviz2',
            output='screen', arguments=['-d', rviz_config_path],
            condition=IfCondition(
                PythonExpression(['"', use_rviz_lc, '"', ' == "true" and not ', remap_tf_lc])),
            parameters=[{'use_sim_time': use_sim_time_lc}],
        ),
        Node(
            package='rviz2', executable='rviz2', name='rviz2',
            output='screen', arguments=['-d', rviz_config_path],
            namespace=namespace_lc,
            condition=IfCondition(
                PythonExpression(['"', use_rviz_lc, '"', ' == "true" and ', remap_tf_lc])),
            parameters=[{'use_sim_time': use_sim_time_lc}],
            remappings=remappings
        )
    ])

#sources: 
#https://navigation.ros.org/setup_guides/index.html#
#https://answers.ros.org/question/374976/ros2-launch-gazebolaunchpy-from-my-own-launch-file/
#https://github.com/ros2/rclcpp/issues/940