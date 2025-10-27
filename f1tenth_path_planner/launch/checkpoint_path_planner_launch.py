#!/usr/bin/env python3
import os
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    # Get workspace root from COLCON_PREFIX_PATH or assume standard structure
    ws_root = os.environ.get('COLCON_PREFIX_PATH', '/home/tomark/sim_ws/install').split(':')[0]
    ws_root = os.path.dirname(ws_root)  # Remove 'install' to get workspace root
    default_csv_path = os.path.join(ws_root, 'src', 'f1tenth_path_planner', 'data', 'checkpoints.csv')

    global_node = Node(
        package='f1tenth_path_planner',
        executable='global_checkpoint_node',
        name='global_checkpoint_node',
        output='screen',
        parameters=[{
            'checkpoint_csv_path': default_csv_path,
            'publish_topic': '/global_path',
            'initial_pose_topic': '/initialpose',
        }]
    )

    local_node = Node(
        package='f1tenth_path_planner',
        executable='local_avoidance_node',
        name='local_avoidance_node',
        output='screen',
        parameters=[{
            'global_path_topic': '/global_path',
            'scan_topic': '/scan',  # or '/scan_fixed' if using SLAM fix node
            'local_horizon': 8.0,
            'path_resolution': 0.2,
            'lateral_offsets': [0.0, 0.4, -0.4],
            'safety_radius': 0.4,
            'lookahead_distance': 2.0,
            'base_frame': 'ego_racecar/base_link',
            'map_frame': 'map',
        }]
    )

    ld = LaunchDescription()
    ld.add_action(global_node)
    ld.add_action(local_node)
    return ld
