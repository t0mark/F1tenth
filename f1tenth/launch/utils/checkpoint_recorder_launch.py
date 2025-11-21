#!/usr/bin/env python3
"""
Checkpoint Recorder Launch
==========================
Launch file for checkpoint_recorder_node that saves clicked points to CSV and publishes path
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Create launch description for checkpoint recorder node."""
    # f1tenth 패키지의 소스 디렉토리 경로 찾기
    pkg_share = get_package_share_directory('f1tenth')

    # 현재 파일 경로에서 src/f1tenth 찾기
    parts = pkg_share.split(os.sep)

    # 'install' 이전의 경로를 찾아서 src 디렉토리로 변환
    if 'install' in parts:
        install_index = parts.index('install')
        workspace_root = os.sep.join(parts[:install_index])
        src_base = os.path.join(workspace_root, 'src', 'f1tenth')
    else:
        # fallback
        src_base = pkg_share

    default_csv_path = os.path.join(src_base, 'data', 'checkpoints.csv')
    simulator_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('simulation'),
                'launch',
                'gym_bridge_launch.py'
            )
        )
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'map_frame',
            default_value='map',
            description='Recorded checkpoints frame id.'
        ),
        DeclareLaunchArgument(
            'output_csv_path',
            default_value=default_csv_path,
            description='CSV file path where checkpoints will be stored.'
        ),
        DeclareLaunchArgument(
            'auto_save_on_add',
            default_value='true',
            description='Automatically save CSV each time a checkpoint is added.'
        ),
        DeclareLaunchArgument(
            'publish_topic',
            default_value='/checkpoint_path',
            description='Topic for published checkpoint path.'
        ),
        DeclareLaunchArgument(
            'clicked_point_topic',
            default_value='/clicked_point',
            description='Topic to subscribe for clicked points.'
        ),
        simulator_launch,
        Node(
            package='f1tenth',
            executable='checkpoint_recorder_node',
            name='checkpoint_recorder_node',
            emulate_tty=True,
            output='screen',
            parameters=[{
                'map_frame': LaunchConfiguration('map_frame'),
                'output_csv_path': LaunchConfiguration('output_csv_path'),
                'auto_save_on_add': LaunchConfiguration('auto_save_on_add'),
                'publish_topic': LaunchConfiguration('publish_topic'),
                'clicked_point_topic': LaunchConfiguration('clicked_point_topic'),
            }]
        ),
    ])
