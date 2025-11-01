import os
import subprocess
from datetime import datetime

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch import logging as launch_logging
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, RegisterEventHandler
from launch.event_handlers import OnShutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


LOGGER = launch_logging.get_logger('mapping_launch')


def _save_slam_toolbox_map(map_stem: str):
    if not map_stem:
        LOGGER.warning('Map output stem is empty; skipping map save.')
        return []

    map_dir = os.path.dirname(map_stem) or '.'
    os.makedirs(map_dir, exist_ok=True)

    LOGGER.info('Saving slam_toolbox map to %s.[yaml|pgm]', map_stem)
    try:
        subprocess.run(
            [
                'ros2', 'run', 'slam_toolbox', 'save_map',
                '--', '-f', map_stem,
            ],
            check=True,
        )
    except FileNotFoundError as exc:
        LOGGER.error('Map save failed (command missing: %s).', exc)
    except subprocess.CalledProcessError as exc:
        LOGGER.error('Map save failed with exit code %s.', exc.returncode)

    return []


def generate_launch_description():
    slam_config = os.path.join(
        get_package_share_directory('mapping'), 'config', 'slam_toolbox.yaml')
    rviz_config = os.path.join(
        get_package_share_directory('mapping'), 'rviz', 'map.rviz')
    localization_launch = os.path.join(
        get_package_share_directory('localization'), 'launch', 'ekf_launch.py')

    default_maps_dir = os.path.join(os.path.expanduser('~'), 'f1tenth', 'maps')
    os.makedirs(default_maps_dir, exist_ok=True)
    default_map_stem = os.path.join(
        default_maps_dir, datetime.now().strftime('map_%Y%m%d_%H%M%S'))

    map_output_argument = DeclareLaunchArgument(
        'map_output_stem',
        default_value=default_map_stem,
        description='Absolute path (without extension) used when saving the map on shutdown.',
    )
    use_sim_time_argument = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time',
    )

    def launch_setup(context, *args, **kwargs):
        use_sim_time = LaunchConfiguration('use_sim_time')
        map_stem = LaunchConfiguration('map_output_stem').perform(context)

        if not map_stem:
            LOGGER.warning('Map output stem is empty; map files will not be saved.')
        else:
            map_dir = os.path.dirname(map_stem) or '.'
            os.makedirs(map_dir, exist_ok=True)

        slam_node = Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            output='screen',
            parameters=[
                slam_config,
                {'use_sim_time': use_sim_time},
            ],
        )

        rviz_node = Node(
            package='rviz2',
            executable='rviz2',
            name='slam_toolbox_rviz',
            arguments=['-d', rviz_config],
            output='screen',
            parameters=[
                {'use_sim_time': use_sim_time},
            ],
        )

        ekf_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(localization_launch),
            launch_arguments={'use_sim_time': use_sim_time}.items(),
        )

        save_map_handler = None
        if map_stem:
            save_map_handler = RegisterEventHandler(
                OnShutdown(
                    on_shutdown=[
                        OpaqueFunction(function=lambda ctx: _save_slam_toolbox_map(map_stem))
                    ],
                )
            )

        actions = [
            slam_node,
            rviz_node,
            ekf_launch,
        ]
        if save_map_handler is not None:
            actions.append(save_map_handler)

        return actions

    return LaunchDescription([
        map_output_argument,
        use_sim_time_argument,
        OpaqueFunction(function=launch_setup),
    ])
