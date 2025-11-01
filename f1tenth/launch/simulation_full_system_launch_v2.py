#!/usr/bin/env python3
"""
F1TENTH Full System Launch
===========================
전체 자율주행 시스템 통합 런치 파일 (레이싱 로직 적용)

실행 순서:
1. Simulator (gym_bridge_launch.py) - 시뮬레이션 환경
2. Racing Logic (opponent_detection, spliner, state_machine, controller) - 2초 지연
"""

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os

def generate_launch_description():
    """F1TENTH 통합 자율주행 시스템 런치 (레이싱 로직 적용)"""

    # 1. Simulator Launch (시뮬레이션 환경)
    simulator_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('simulator'),
                'launch',
                'gym_bridge_launch.py'
            ])
        )
    )

    # 최종 Waypoint 파일 경로 설정
    pkg_path_planner = FindPackageShare('path_planner')
    waypoint_file_path = PathJoinSubstitution([
        pkg_path_planner,
        'data',
        'final_waypoints.csv'
    ])

    # 2. Racing Logic Nodes - 2초 지연
    racing_logic_launch = TimerAction(
        period=2.0,
        actions=[
            # Obstacle Detection Node
            Node(
                package='path_planner',
                executable='opponent_detection_node',
                name='opponent_detection',
                output='screen'
            ),
            # Spline Planner Node
            Node(
                package='path_planner',
                executable='spliner_node',
                name='spline_node',
                output='screen',
                parameters=[{'waypoint_file': waypoint_file_path}]
            ),
            # State Machine Node
            Node(
                package='path_planner',
                executable='state_machine_node',
                name='state_machine',
                output='screen',
                parameters=[{'waypoint_file': waypoint_file_path}]
            ),
            # Controller Node
            Node(
                package='path_planner',
                executable='controller_node',
                name='controller',
                output='screen',
                parameters=[{'waypoint_file': waypoint_file_path}]
            ),
        ]
    )

    return LaunchDescription([
        # Launch Sequence
        simulator_launch,
        racing_logic_launch,
    ])
