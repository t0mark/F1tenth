#!/usr/bin/env python3
"""
F1TENTH Full System Launch
===========================
전체 자율주행 시스템 통합 런치 파일

실행 순서:
1. Simulator (gym_bridge_launch.py) - 시뮬레이션 환경
2. Path Planner (path_planner_launch.py) - 경로 계획 (2초 지연)
3. Control (pure_pursuit_launch.py) - 차량 제어 (4초 지연)

사용법:
  # 기본 실행
  ros2 launch f1tenth full_system_launch.py

  # 커스텀 설정으로 실행
  ros2 launch f1tenth full_system_launch.py \
    global_config:=global_centerline.yaml \
    local_config:=local_avoidance.yaml
"""

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """F1TENTH 통합 자율주행 시스템 런치"""

    global_config_arg = DeclareLaunchArgument(
        'global_config',
        default_value='global_checkpoint.yaml',
        description='글로벌 플래너 설정 파일.'
    )
    local_config_arg = DeclareLaunchArgument(
        'local_config',
        default_value='local_sampler.yaml',
        description='로컬 플래너 설정 파일.'
    )

    # 1. Simulator Launch (시뮬레이션 환경)
    simulator_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('simulation'),
                'launch',
                'gym_bridge_launch.py'
            ])
        )
    )

    # 2. Path Sampler (CSV-based) - 2초 지연
    path_sampler_launch = TimerAction(
        period=2.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution([
                        FindPackageShare('f1tenth'),
                        'launch',
                        'path_planner_launch.py'
                    ])
                ),
                launch_arguments={
                    'global_config': LaunchConfiguration('global_config'),
                    'local_config': LaunchConfiguration('local_config'),
                    'is_integrated': 'true'
                }.items()
            )
        ]
    )

    # 3. Control Launch (차량 제어) - 4초 지연
    control_launch = TimerAction(
        period=4.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution([
                        FindPackageShare('f1tenth'),
                        'launch',
                        'control_launch.py'
                    ])
                )
            )
        ]
    )

    return LaunchDescription([
        global_config_arg,
        local_config_arg,
        # Launch Sequence
        simulator_launch,
        path_sampler_launch,
        control_launch,
    ])
