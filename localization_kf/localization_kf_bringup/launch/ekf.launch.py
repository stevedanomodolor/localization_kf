
import os
from launch import LaunchDescription


from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument, GroupAction, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node



def generate_launch_description():

    bringup_dir = get_package_share_directory('localization_kf_bringup')
    log_level = LaunchConfiguration('log_level')
    use_sim_time = LaunchConfiguration('use_sim_time')
    localization_core_params = os.path.join(bringup_dir, 'params', 'ekf.yaml')

    lifecycle_nodes = [
        'localization_core'
    ]
    
    stdout_linebuf_envvar = SetEnvironmentVariable(
        'RCUTILS_LOGGING_BUFFERED_STREAM', '1')


    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time', default_value='true',
        description='Use simulation time')
    declare_log_level_cmd = DeclareLaunchArgument(
        'log_level', default_value='info',
        description='log level')
    
    load_nodes = GroupAction(
        actions=[
            Node(
                package='localization_kf_core',
                executable='localization_core',
                name='localization_core',
                output='screen',
                arguments=['--ros-args', '--log-level', log_level],
                # prefix=['xterm -e gdb -ex run --args'],
                parameters=[
                    {'use_sim_time': use_sim_time},
                    localization_core_params
                ],
            ),
            # Node(
            #     package='plotjuggler',
            #     executable='plotjuggler',
            #     name='plotjuggler',
            #     output='screen',
            #     arguments=[
            #         '--layout', os.path.join(bringup_dir, 'params', 'plotjoggler.xml'),
            #         '--ros-args', '--log-level', log_level
            #     ],
            #     parameters=[{'use_sim_time': use_sim_time}]
            # ),
            Node(
                package='nav2_lifecycle_manager',
                executable='lifecycle_manager',
                name='lifecycle_manager_navigation',
                output='screen',
                arguments=['--ros-args', '--log-level', log_level],
                parameters=[{'use_sim_time': use_sim_time},
                            {'autostart': True},
                            {'node_names': lifecycle_nodes}]
            )
        ]
        
    )

    # create the launch description and populate
    ld = LaunchDescription()

    # define the launch actions
    ld.add_action(stdout_linebuf_envvar)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_log_level_cmd)
    ld.add_action(load_nodes)

    return ld