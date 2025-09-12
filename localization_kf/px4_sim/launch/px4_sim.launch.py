from distutils.dir_util import copy_tree
import shutil
import os
import tempfile
from typing import List, Literal
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.actions import RegisterEventHandler, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource

import os
from launch.substitutions import LaunchConfiguration, PythonExpression
from subprocess import Popen, PIPE
from shlex import split

def seed_rootfs(px4_dir: str, rootfs: str) -> None:
    """
    Seed the PX4 root filesystem by copying necessary files.

    Args:
        px4_dir (str): Path to the PX4 directory.
        rootfs (str): Path to the root filesystem directory.
    """
    print(f"Seeding PX4 rootfs at '{rootfs}' from '{px4_dir}'")
    copy_tree(px4_dir, rootfs)


def get_px4_process(px4_dir: str, drone_id: str, additional_env: dict) -> ExecuteProcess:
    """
    Create an ExecuteProcess action to launch PX4 with the specified configuration.

    Args:
        px4_dir (str): Path to the PX4 directory.
        drone_id (str): Drone ID as a string.
        additional_env (dict): Additional environment variables for the process.

    Returns:
        ExecuteProcess: Launch action for the PX4 process.
    """
    rootfs_tmp = tempfile.TemporaryDirectory()
    rootfs_path = rootfs_tmp.name
    rc_script = os.path.join(px4_dir, 'etc/init.d-posix/rcS')
    seed_rootfs(px4_dir, rootfs_path)

    cmd = [
        'px4',
        f'{rootfs_path}/ROMFS/px4fmu_common',
        '-s', rc_script,
        '-i', drone_id,
        '-d'
    ]
    print(f"Launching PX4 with command: {cmd}")

    return ExecuteProcess(
        cmd=cmd,
        cwd=px4_dir,
        additional_env=additional_env,
        output='screen'
        # PX4 does not use ROS parameters, so use_sim_time is not passed here
    )


def set_px4_params(px4_dir: str, param_file: str) -> None:
    """
    Copy PX4 parameter file to the appropriate airframes directory.

    Args:
        px4_dir (str): Path to the PX4 directory.
        param_file (str): Parameter file name.
    """
    if param_file:
        src = os.path.join(px4_dir, 'params', param_file)
        dest = os.path.join(px4_dir, 'etc', 'init.d-posix', 'airframes', param_file)
        print(f"Copying PX4 param file from '{src}' to '{dest}'")
        shutil.copyfile(src, dest)

def generate_launch_description():
    # === Configuration ===
    px4_dir = get_package_share_directory('px4_sim')
    drone_id = '0'
    drone_type = 'x500'
    world_name = 'empty_px4_world'
    autostart_var = '4001'
    dds_domain_id = ''
    param_file_name = f"{autostart_var}_gz_{drone_type}"
    model_name = f"{drone_type}_{drone_id}"
    model_sdf_filename = os.path.join(px4_dir, 'models', drone_type, 'model.sdf')

    # === Launch Arguments ===
    use_sim_time = LaunchConfiguration('use_sim_time', default=True)
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value=use_sim_time,
        description='If true, use simulated clock'
    )

    # === Environment Variables ===
    os.environ['GZ_SIM_RESOURCE_PATH'] = (
        ':' + os.path.join(px4_dir, 'models') +
        ':' + os.path.join(px4_dir, 'worlds')
    )
    os.environ['PX4_GZ_WORLD'] = ""
    os.environ['PX4_SYS_AUTOSTART'] = autostart_var
    os.environ['PX4_GZ_MODEL_NAME'] = model_name

    # === PX4 Process and Params ===
    run_px4 = get_px4_process(
        px4_dir,
        drone_id,
        {'ROS_DOMAIN_ID': dds_domain_id}
    )
    set_px4_params(px4_dir, param_file_name)

    # === Wait for PX4 Spawn ===
    wait_spawn = ExecuteProcess(cmd=["sleep", "5"])

    # === micro-ROS Agent Node ===
    micro_ros_agent = Node(
        package='micro_ros_agent',
        executable='micro_ros_agent',
        arguments=['udp4', '-p', '8888'],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )

    # === Spawn Entity Node ===
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=[
            '-file', model_sdf_filename,
            '-name', model_name,
            '-allow_renaming', 'true',
            '-x', LaunchConfiguration('x_pose', default='0.0'),
            '-y', LaunchConfiguration('y_pose', default='0.0'),
            '-z', LaunchConfiguration('z_pose', default='0.24'),
            '-R', LaunchConfiguration('roll_pose', default='0.0'),
            '-P', LaunchConfiguration('pitch_pose', default='0.0'),
            '-Y', LaunchConfiguration('yaw_pose', default='0.0')
        ],
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # === Gazebo Instance Check ===
    p1 = Popen(split("gz topic -l"), stdout=PIPE)
    p2 = Popen(split("grep -m 1 -e '/world/.*/clock'"), stdin=p1.stdout, stdout=PIPE)
    p3 = Popen(split(r"sed 's/\/world\///g; s/\/clock//g'"), stdin=p2.stdout, stdout=PIPE)
    command_output = p3.stdout.read().decode('utf-8')

    # === Bridge Node ===
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        # arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
        parameters=[
            {
                'use_sim_time': use_sim_time,
                'config_file': os.path.join(px4_dir, 'configs', 'ros_gz_bridge.yaml'),
                'expand_gz_topic_names': True
            }
        ],
        output='screen'
    )

    # === Launch Description ===
    ld = LaunchDescription([
        use_sim_time_arg,
        spawn_entity,
        bridge,
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=spawn_entity,
                on_exit=[wait_spawn],
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=wait_spawn,
                on_exit=[run_px4],
            )
        ),
    ])

    # === Conditional Actions ===
    if not command_output:
        ld.add_action(micro_ros_agent)
        ld.add_action(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    [os.path.join(
                        get_package_share_directory('ros_gz_sim'),
                        'launch', 'gz_sim.launch.py'
                    )]
                ),
                launch_arguments=[('gz_args', [' -r -v 4 ', world_name, '.sdf'])]
            )
        )
    else:
        print('Another gz instance is running, it will only try to spawn the model in gz.')

    return ld
