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

import math
import os
import yaml
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

    config_file = LaunchConfiguration('config').perform(context)

    # 설정 파일 절대 경로 생성
    pkg_share = get_package_share_directory('f1tenth')
    if not os.path.isabs(config_file):
        config_file = os.path.join(pkg_share, 'config', config_file)

    # YAML 설정 파일 로드
    with open(config_file, 'r') as f:
        config = yaml.safe_load(f)

    # 시뮬레이션 모드 결정: 런치 인자 우선, 없으면 설정 파일에서 읽기
    is_simulation_arg = LaunchConfiguration('is_simulation').perform(context)
    if is_simulation_arg == 'auto':
        # 설정 파일에서 읽기
        is_simulation = config.get('system', {}).get('is_simulation', True)
    else:
        # 런치 인자 사용
        is_simulation = is_simulation_arg.lower() == 'true'

    # 각 서브시스템의 설정 파일 경로 추출
    localization_global = config.get('localization', {}).get('global_config', 'localization/global_amcl.yaml')
    localization_local = config.get('localization', {}).get('local_config', 'localization/local_ekf.yaml')
    planning_global = config.get('planning', {}).get('global_config', 'planning/global_checkpoint.yaml')
    planning_local = config.get('planning', {}).get('local_config', 'planning/local_sampler.yaml')
    control_config = config.get('control', {}).get('config', 'control/pure_pursuit.yaml')

    # 상대 경로를 절대 경로로 변환
    if not os.path.isabs(localization_global):
        localization_global = os.path.join(pkg_share, 'config', localization_global)
    if not os.path.isabs(localization_local):
        localization_local = os.path.join(pkg_share, 'config', localization_local)
    if not os.path.isabs(planning_global):
        planning_global = os.path.join(pkg_share, 'config', planning_global)
    if not os.path.isabs(planning_local):
        planning_local = os.path.join(pkg_share, 'config', planning_local)
    if not os.path.isabs(control_config):
        control_config = os.path.join(pkg_share, 'config', control_config)

    map_yaml_path = LaunchConfiguration('map').perform(context) if not is_simulation else \
                    os.path.join(pkg_share, 'maps', LaunchConfiguration('map_path').perform(context) + '.yaml')

    map_graph_settings = config.get('map_graph', {})
    map_graph_auto_generate = str(map_graph_settings.get('auto_generate', True)).lower()
    map_graph_dir = map_graph_settings.get('graph_dir', 'data')
    map_graph_xy_res = str(map_graph_settings.get('xy_resolution', 0.1))
    map_graph_theta_res = str(map_graph_settings.get('theta_resolution', math.pi / 12.0))
    map_graph_vehicle_length = str(map_graph_settings.get('vehicle_length', 0.50))
    map_graph_vehicle_width = str(map_graph_settings.get('vehicle_width', 0.25))
    map_graph_safety_margin = str(map_graph_settings.get('safety_margin', 0.1))

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
                    'utils',
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
                'global_config': localization_global,
                'local_config': localization_local,
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
            'global_config': planning_global,
            'local_config': planning_local,
            'is_integrated': 'true',
            'map_yaml': map_yaml_path,
            'map_graph_auto_generate': map_graph_auto_generate,
            'map_graph_dir': map_graph_dir,
            'map_graph_xy_resolution': map_graph_xy_res,
            'map_graph_theta_resolution': map_graph_theta_res,
            'map_graph_vehicle_length': map_graph_vehicle_length,
            'map_graph_vehicle_width': map_graph_vehicle_width,
            'map_graph_safety_margin': map_graph_safety_margin,
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
            'controller_config': control_config,
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
            default_value='auto',
            description='시뮬레이션 모드 (true/false/auto). auto인 경우 설정 파일에서 읽음'
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
