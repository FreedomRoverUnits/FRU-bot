from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition, UnlessCondition


def generate_launch_description():
    ID = 1
    urdf_path = PathJoinSubstitution(
        [FindPackageShare("linorobot2_description"), "urdf/robots", f"agent{ID}.urdf.xacro"]
    )
    ekf_config_path = PathJoinSubstitution(
        [FindPackageShare("linorobot2_base"), "config", f"ekf_a{ID}.yaml"]
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
        DeclareLaunchArgument(
            name='loc',
            default_value="true",
            description='Start ekf_node'
        ),
        DeclareLaunchArgument(
            name='urdf', 
            default_value=urdf_path,
            description='URDF path'
        ),
        Node(
            package='robot_localization',
            executable='ekf_node',
            condition=IfCondition(LaunchConfiguration('loc')),
            name=f'ekf_filter_node{ID}',
            output='screen',
            parameters=[
                ekf_config_path
            ],
            remappings=[("odometry/filtered", f"agent{ID}/odom")]
            ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(description_launch_path),
            launch_arguments={'rviz' : LaunchConfiguration('rviz'),
                              'sim' : LaunchConfiguration('sim'),
                              'urdf' : LaunchConfiguration('urdf')}.items()
        )
    ])