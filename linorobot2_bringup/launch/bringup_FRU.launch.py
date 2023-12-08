from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition, UnlessCondition


def generate_launch_description():
    ekf_config_path = PathJoinSubstitution(
        [FindPackageShare("linorobot2_base"), "config", "ekf.yaml"]
    )
    description_launch_path = PathJoinSubstitution(
        [FindPackageShare('linorobot2_description'), 'launch', 'description.launch.py']
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
        Node(
                package='robot_localization',
                executable='ekf_node',
                name='ekf_filter_node',
                output='screen',
                parameters=[
                    ekf_config_path
                ],
                remappings=[("odometry/filtered", "odom")]
            ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(description_launch_path),
            launch_arguments={'rviz' : LaunchConfiguration('rviz'),
                              'sim' : LaunchConfiguration('sim')}.items()
        )
    ])