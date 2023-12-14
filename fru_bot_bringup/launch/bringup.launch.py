from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetLaunchConfiguration
from launch.substitutions import LaunchConfiguration, TextSubstitution, PathJoinSubstitution, PythonExpression
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node, PushRosNamespace
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition, UnlessCondition
from nav2_common.launch import RewrittenYaml

def generate_launch_description():
    robot_name = 'FRU_bot'
    robot_idx = ''
    
    use_ns=str(False)
    
    remappings = [("odometry/filtered", "odom"),] #  ('/tf', 'tf'), ('/tf_static', 'tf_static')
    
    ekf_config_path = PathJoinSubstitution(
        [FindPackageShare("fru_bot_base"), "config", "ekf_default.yaml"]
    )
    description_launch_path = PathJoinSubstitution(
        [FindPackageShare('fru_bot_description'), 'launch', 'description.launch.py']
    )
    sim_launch_path = PathJoinSubstitution(
        [FindPackageShare('fru_bot_gazebo'), 'launch', 'gazebo.launch.py']
    )
    
    # Launch arg defs
    use_rviz_launch_arg = DeclareLaunchArgument(
            name='use_rviz', default_value='false', description='Start rviz'
        )
    use_loc_launch_arg = DeclareLaunchArgument(
            name='use_loc', default_value=str(False), description='Start localization ekf node'
        )
    sim_launch_arg = DeclareLaunchArgument(
            name='sim', default_value=str(False), description='Using sim'
        )
    use_sim_time_launch_arg = DeclareLaunchArgument(
            name='use_sim_time', default_value='false', description='Using sim time'
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
    
    # Launch config defs
    use_rviz_lc = LaunchConfiguration('use_rviz'); sim_lc = LaunchConfiguration('sim')
    use_ns_lc = LaunchConfiguration('use_ns'); idx_lc = LaunchConfiguration('idx')
    use_loc_lc = LaunchConfiguration('use_loc')
    use_sim_time_lc = LaunchConfiguration('use_sim_time')
    
    namespace_lc = PythonExpression(['"', LaunchConfiguration('ns'), '"', ' if ', use_ns_lc, ' else ""'])
    
    ekf_substitutions = {
                'odom0' : ['/', namespace_lc, '/', TextSubstitution(text='odom/unfiltered')],
                'imu0' : ['/', namespace_lc, '/', TextSubstitution(text='imu/data')]
                }
    configured_ekf_params = RewrittenYaml(source_file=ekf_config_path, param_rewrites=ekf_substitutions, 
                                          root_key=namespace_lc, convert_types=True)
    return LaunchDescription([
        use_rviz_launch_arg, use_loc_launch_arg, sim_launch_arg, use_sim_time_launch_arg, 
        use_ns_launch_arg, idx_launch_arg, ns_launch_arg,
        
        # EKF Node w/ namespace
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            namespace=namespace_lc,
            output='screen',
            condition=IfCondition(
                PythonExpression(['not ', sim_lc, ' and ', use_loc_lc])
                ),
            parameters=[
                configured_ekf_params
            ],
            remappings=remappings,
            arguments=["-robot_namespace ", namespace_lc]
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(description_launch_path),
            launch_arguments={'use_rviz' : use_rviz_lc,
                              'use_sim_time' : use_sim_time_lc,
                              'namespace' : namespace_lc,
                              'idx' : idx_lc,
                              'use_ns' : use_ns_lc
                              }.items(),
            condition=IfCondition(PythonExpression(['not ', sim_lc]))
        ),
        
        # Launch Gazebo
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(sim_launch_path),
            launch_arguments={'namespace' : namespace_lc,
                              'idx' : idx_lc,
                              'use_ns' : use_ns_lc,
                              'use_rviz' : use_rviz_lc
                              }.items(),
            condition=IfCondition(PythonExpression([sim_lc]))
        )
    ])