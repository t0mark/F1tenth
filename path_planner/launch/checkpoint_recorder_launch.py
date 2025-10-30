#!/usr/bin/env python3
"""
Checkpoint Recorder Launch
==========================
클릭한 포인트를 CSV로 저장하고 경로를 발행하는 checkpoint_recorder_node 런치 파일

사용법:
  ros2 launch path_planner checkpoint_recorder_launch.py \
    output_csv_path:=/absolute/path/to/checkpoints.csv
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
    pkg_share = get_package_share_directory('path_planner')
    workspace_root = os.path.abspath(os.path.join(pkg_share, '..', '..', '..', '..'))
    src_base = os.path.join(workspace_root, 'src', 'path_planner', 'path_planner')
    if os.path.isdir(os.path.join(workspace_root, 'src')):
        default_csv_path = os.path.join(src_base, 'data', 'checkpoints.csv')
    else:
        default_csv_path = os.path.join(pkg_share, 'data', 'checkpoints.csv')
    simulator_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('simulator'),
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
            package='path_planner',
            executable='checkpoint_recorder_node',
            name='checkpoint_recorder_node',
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
