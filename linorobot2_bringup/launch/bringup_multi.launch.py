from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition, UnlessCondition


def generate_launch_description():
    
    bringup_a0_launch_path = PathJoinSubstitution(
        [FindPackageShare('linorobot2_bringup'), 'launch', 'bringup_agent0.launch.py']
    )
    bringup_a1_launch_path = PathJoinSubstitution(
        [FindPackageShare('linorobot2_bringup'), 'launch', 'bringup_agent1.launch.py']
    )
    bringup_a2_launch_path = PathJoinSubstitution(
        [FindPackageShare('linorobot2_bringup'), 'launch', 'bringup_agent2.launch.py']
    )
    
    return LaunchDescription([
        DeclareLaunchArgument(
            name='rviz',
            default_value="false",
            description='Start rviz'
        ),
        DeclareLaunchArgument(
            name='sim',
            default_value="false",
            description='Using sim'
        ),
        DeclareLaunchArgument(
            name='loc',
            default_value="true",
            description='Start ekf_node'
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(bringup_a0_launch_path),
            launch_arguments={'rviz' : LaunchConfiguration('rviz'),
                              'sim' : LaunchConfiguration('sim'),
                              'loc' : LaunchConfiguration('loc')}.items()
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(bringup_a1_launch_path),
            launch_arguments={'rviz' : 'false',
                              'sim' : LaunchConfiguration('sim'),
                              'loc' : LaunchConfiguration('loc')}.items()
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(bringup_a2_launch_path),
            launch_arguments={'rviz' : 'false',
                              'sim' : LaunchConfiguration('sim'),
                              'loc' : LaunchConfiguration('loc')}.items()
        )
    ])