import os
import subprocess
import time
from datetime import datetime

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch import logging as launch_logging
from launch.actions import DeclareLaunchArgument, OpaqueFunction, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


LOGGER = launch_logging.get_logger('mapping_launch')


def _convert_pbstream_to_map(map_stem: str, wait_timeout_sec: float = 30.0):
    if not map_stem:
        LOGGER.warning('Map output stem is empty; skipping map conversion.')
        return []

    map_dir = os.path.dirname(map_stem) or '.'
    os.makedirs(map_dir, exist_ok=True)

    pbstream_path = f'{map_stem}.pbstream'
    deadline = time.time() + wait_timeout_sec
    while time.time() < deadline:
        if os.path.exists(pbstream_path):
            break
        time.sleep(0.5)
    else:
        LOGGER.error('Timed out waiting for pbstream file at %s', pbstream_path)
        return []

    LOGGER.info('Converting %s to occupancy grid...', pbstream_path)
    try:
        subprocess.run(
            [
                'ros2', 'run', 'cartographer_ros', 'cartographer_pbstream_to_ros_map',
                '-map_filestem', map_stem,
                '-pbstream_filename', pbstream_path,
            ],
            check=True,
        )
    except FileNotFoundError as exc:
        LOGGER.error('Map conversion failed (command missing: %s).', exc)
        return []
    except subprocess.CalledProcessError as exc:
        LOGGER.error('Map conversion failed with exit code %s.', exc.returncode)
        return []

    LOGGER.info('Map saved to %s.[pbstream|pgm|yaml]', map_stem)
    return []


def generate_launch_description():
    config_directory = os.path.join(
        get_package_share_directory('mapping'), 'config')
    config_basename = 'cartographer.lua'
    rviz_config_path = os.path.join(
        get_package_share_directory('mapping'), 'rviz', 'map.rviz')
    default_maps_dir = os.path.join(os.path.expanduser('~'), 'f1tenth_maps')
    os.makedirs(default_maps_dir, exist_ok=True)
    default_map_stem = os.path.join(
        default_maps_dir, datetime.now().strftime('map_%Y%m%d_%H%M%S'))

    map_output_argument = DeclareLaunchArgument(
        'map_output_stem',
        default_value=default_map_stem,
        description='Absolute path (without extension) used when saving the map on shutdown.',
    )

    def launch_setup(context, *args, **kwargs):
        map_stem = LaunchConfiguration('map_output_stem').perform(context)
        if not map_stem:
            LOGGER.warning('Map output stem is empty; map files will not be saved.')
        else:
            map_dir = os.path.dirname(map_stem) or '.'
            os.makedirs(map_dir, exist_ok=True)

        cartographer_kwargs = {
            'package': 'cartographer_ros',
            'executable': 'cartographer_node',
            'name': 'cartographer_node',
            'output': 'screen',
            'arguments': [
                '-configuration_directory', config_directory,
                '-configuration_basename', config_basename,
            ],
            'remappings': [
                ('imu', '/camera/camera/imu'),
            ],
        }
        if map_stem:
            cartographer_kwargs['arguments'].extend([
                '--save_state_filename', f'{map_stem}.pbstream',
            ])

        cartographer_node = Node(**cartographer_kwargs)

        occupancy_grid_node = Node(
            package='cartographer_ros',
            executable='cartographer_occupancy_grid_node',
            name='cartographer_occupancy_grid_node',
            output='screen',
            parameters=[{'resolution': 0.05}],
        )

        rviz_node = Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=[
                '-d', rviz_config_path,
            ],
        )

        convert_handler = None
        if map_stem:
            convert_handler = RegisterEventHandler(
                OnProcessExit(
                    target_action=cartographer_node,
                    on_exit=[
                        OpaqueFunction(function=lambda ctx: _convert_pbstream_to_map(map_stem))
                    ],
                )
            )

        return [
            cartographer_node,
            occupancy_grid_node,
            rviz_node,
            *( [convert_handler] if convert_handler is not None else [] ),
        ]

    return LaunchDescription([
        map_output_argument,
        OpaqueFunction(function=launch_setup),
    ])
