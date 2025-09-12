
import os
from launch import LaunchDescription


from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument, GroupAction, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription



def generate_launch_description():

    bringup_dir = get_package_share_directory('localization_kf_bringup')
    log_level = LaunchConfiguration('log_level')
    use_sim_time = LaunchConfiguration('use_sim_time')
    rviz_config_file = LaunchConfiguration('rviz_config')
    
    launch_ekf = LaunchConfiguration('launch_ekf')

    lifecycle_nodes = [
        'trajectory_navigator'
    ]
    
    stdout_linebuf_envvar = SetEnvironmentVariable(
        'RCUTILS_LOGGING_BUFFERED_STREAM', '1')


    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time', default_value='true',
        description='Use simulation time')
    declare_log_level_cmd = DeclareLaunchArgument(
        'log_level', default_value='info',
        description='log level')
    declare_rviz_config_file_cmd = DeclareLaunchArgument(
        'rviz_config',
        default_value=os.path.join(bringup_dir, 'rviz', 'navigation_panel.rviz'),
        description='Full path to the RVIZ config file to use')
    
    declare_launch_ekf_cmd = DeclareLaunchArgument(
        'launch_ekf', default_value='true',
        description='Whether to launch the ekf node')
    
    load_nodes = GroupAction(
        actions=[
            Node(
                package='localization_kf_navigator',
                executable='trajectory_navigator',
                output='screen',
                arguments=['--ros-args', '--log-level', log_level],
                parameters=[
                    {'use_sim_time': use_sim_time}
                ],
            ),
            Node(
                package='nav2_lifecycle_manager',
                executable='lifecycle_manager',
                name='lifecycle_manager_navigation',
                output='screen',
                arguments=['--ros-args', '--log-level', log_level],
                parameters=[{'use_sim_time': use_sim_time},
                            {'autostart': True},
                            {'node_names': lifecycle_nodes}]
            ),
            Node(
                package='rviz2',
                executable='rviz2',
                name='rviz2',
                output='screen',
                arguments=['-d', rviz_config_file],
                parameters=[{'use_sim_time': use_sim_time}]
            ),
            Node(
                package='localization_kf_core',
                executable='localization_core',
                name='localization_core',
                output='screen',
                arguments=['--ros-args', '--log-level', log_level],
                parameters=[
                    {'use_sim_time': use_sim_time},
                    os.path.join(bringup_dir, 'params', 'ekf.yaml')
                ],
            ),
        ]
        
    )
    
    # include the EKF launch file ekf.launch.py
    ekf_launch_file = os.path.join(bringup_dir, 'launch', 'ekf.launch.py')
    load_ekf = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(ekf_launch_file),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'log_level': log_level
        }.items(),
        condition=IfCondition(launch_ekf)
    )
   

    # create the launch description and populate
    ld = LaunchDescription()

    # define the launch actions
    ld.add_action(stdout_linebuf_envvar)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_log_level_cmd)
    ld.add_action(declare_rviz_config_file_cmd)
    ld.add_action(declare_launch_ekf_cmd)
    ld.add_action(load_nodes)
    ld.add_action(load_ekf)

    return ld