#!/usr/bin/env python3
"""
F1TENTH Full System Launch
===========================
시뮬레이션과 실제 환경 모두를 지원하는 통합 런치 파일

모드:
- is_simulation=true: Simulation (gym_bridge) + Planning + Control
- is_simulation=false: Map Server + Localization + Planning + Control

사용 예:
  # 시뮬레이션 모드
  ros2 launch f1tenth full_system_launch.py is_simulation:=true

  # 실제 환경 모드
  ros2 launch f1tenth full_system_launch.py is_simulation:=false map:=/path/to/map.yaml

  # 커스텀 설정 파일 사용
  ros2 launch f1tenth full_system_launch.py config:=my_config.yaml
"""

import os
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory


def launch_setup(context, *args, **kwargs):
    """런치 인자 해석 후 호출되는 설정 함수"""

    is_simulation = LaunchConfiguration('is_simulation').perform(context).lower() == 'true'
    config_file = LaunchConfiguration('config').perform(context)

    # 설정 파일 절대 경로 생성
    pkg_share = get_package_share_directory('f1tenth')
    if not os.path.isabs(config_file):
        config_file = os.path.join(pkg_share, 'config', config_file)

    nodes = []

    # ============================================================
    # 시뮬레이션 모드: gym_bridge 실행
    # ============================================================
    if is_simulation:
        simulation_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    FindPackageShare('simulation'),
                    'launch',
                    'gym_bridge_launch.py'
                ])
            ),
            launch_arguments={
                'map_path': LaunchConfiguration('map_path'),
            }.items()
        )
        nodes.append(simulation_launch)

    # ============================================================
    # 실제 환경 모드: Map Server + Localization 실행
    # ============================================================
    else:
        # Map Server
        map_server_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    FindPackageShare('f1tenth'),
                    'launch',
                    'tools',
                    'map_server_launch.py'
                ])
            ),
            launch_arguments={
                'map': LaunchConfiguration('map'),
                'use_sim_time': 'false',
                'map_topic': 'map',
            }.items()
        )
        nodes.append(map_server_launch)

        # Localization (EKF + AMCL)
        localization_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    FindPackageShare('f1tenth'),
                    'launch',
                    'localization_launch.py'
                ])
            ),
            launch_arguments={
                'use_sim_time': 'false',
            }.items()
        )
        nodes.append(localization_launch)

    # ============================================================
    # Planning (항상 실행)
    # ============================================================
    planning_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('f1tenth'),
                'launch',
                'planning_launch.py'
            ])
        ),
        launch_arguments={
            'global_config': config_file,
            'local_config': config_file,
            'is_integrated': 'true',  # simulation은 별도로 이미 실행됨
        }.items()
    )
    nodes.append(planning_launch)

    # ============================================================
    # Control (항상 실행)
    # ============================================================
    control_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('f1tenth'),
                'launch',
                'control_launch.py'
            ])
        ),
        launch_arguments={
            'controller_config': config_file,
            'is_integrated': 'true',
        }.items()
    )
    nodes.append(control_launch)

    return nodes


def generate_launch_description():
    """통합 시스템 런치 디스크립션 생성"""

    pkg_share = get_package_share_directory('f1tenth')

    return LaunchDescription([
        # ============================================================
        # Launch Arguments
        # ============================================================
        DeclareLaunchArgument(
            'is_simulation',
            default_value='true',
            description='시뮬레이션 모드 (true: gym_bridge, false: map_server + localization)'
        ),
        DeclareLaunchArgument(
            'map_path',
            default_value='track',
            description='시뮬레이션용 맵 경로 (확장자 제외)'
        ),
        DeclareLaunchArgument(
            'map',
            default_value=PathJoinSubstitution([
                FindPackageShare('f1tenth'),
                'maps',
                'track.yaml'
            ]),
            description='실제 환경용 맵 YAML 절대 경로'
        ),
        DeclareLaunchArgument(
            'config',
            default_value='full_system.yaml',
            description='통합 시스템 설정 파일 (global, local, control 모두 포함)'
        ),

        # 동적 설정 함수 실행
        OpaqueFunction(function=launch_setup)
    ])
