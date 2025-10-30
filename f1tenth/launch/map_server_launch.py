#!/usr/bin/env python3
"""Standalone Nav2 map_server launch file."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Launch Nav2 map_server with lifecycle manager."""
    f1tenth_share = get_package_share_directory('f1tenth')
    default_map = os.path.join(f1tenth_share, 'maps', 'track.yaml')

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time (clock from /clock topic).'
    )
    map_yaml_arg = DeclareLaunchArgument(
        'map',
        default_value=default_map,
        description='Absolute path to the map YAML file.'
    )
    map_topic_arg = DeclareLaunchArgument(
        'map_topic',
        default_value='map',
        description='Topic name for the occupancy map.'
    )

    map_server_node = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{
            'yaml_filename': LaunchConfiguration('map'),
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'topic': LaunchConfiguration('map_topic'),
            'frame_id': 'map',
        }]
    )

    lifecycle_manager_node = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_localization',
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'autostart': True,
            'node_names': ['map_server'],
        }]
    )

    return LaunchDescription([
        use_sim_time_arg,
        map_yaml_arg,
        map_topic_arg,
        map_server_node,
        lifecycle_manager_node,
    ])
