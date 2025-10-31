#!/usr/bin/env python3
"""
F1TENTH Real Vehicle Full System Launch
=======================================
실차 운용을 위한 통합 런치 파일

시퀀스:
1. Map Server (정적 맵 제공)
2. Localization (EKF + AMCL)
3. Path Planner
4. Control (Pure Pursuit)

사용 예:
  ros2 launch f1tenth real_full_system_launch.py \
    map:=/abs/path/to/map.yaml \
    global_config:=global_checkpoint.yaml \
    local_config:=local_avoidance.yaml
"""

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """통합 실차 시스템 런치 디스크립션 생성."""

    # Launch arguments
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Simulation time 사용 여부 (실차는 일반적으로 false).'
    )
    map_arg = DeclareLaunchArgument(
        'map',
        default_value=PathJoinSubstitution([
            FindPackageShare('f1tenth'),
            'maps',
            'track.yaml'
        ]),
        description='맵 YAML 절대 경로.'
    )
    map_topic_arg = DeclareLaunchArgument(
        'map_topic',
        default_value='map',
        description='맵을 퍼블리시할 토픽 이름.'
    )
    global_config_arg = DeclareLaunchArgument(
        'global_config',
        default_value='global_checkpoint.yaml',
        description='글로벌 플래너 설정 파일.'
    )
    local_config_arg = DeclareLaunchArgument(
        'local_config',
        default_value='local_avoidance.yaml',
        description='로컬 플래너 설정 파일.'
    )
    rviz_config_arg = DeclareLaunchArgument(
        'rviz_config',
        default_value=PathJoinSubstitution([
            FindPackageShare('f1tenth'),
            'rviz',
            'f1tenth.rviz'
        ]),
        description='RViz 설정 파일 경로.'
    )

    # 1. Map server (즉시 실행)
    map_server_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('f1tenth'),
                'launch',
                'map_server_launch.py'
            ])
        ),
        launch_arguments={
            'map': LaunchConfiguration('map'),
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'map_topic': LaunchConfiguration('map_topic'),
        }.items()
    )

    # 2. Localization (EKF + AMCL) - map server 기동 후 2초 대기
    localization_launch = TimerAction(
        period=2.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution([
                        FindPackageShare('localization'),
                        'launch',
                        'amcl_launch.py'
                    ])
                ),
                launch_arguments={
                    'use_sim_time': LaunchConfiguration('use_sim_time'),
                }.items()
            )
        ]
    )

    lifecycle_manager_node = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_localization',
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'autostart': True,
            'node_names': ['map_server', 'amcl'],
        }]
    )

    # 3. Path planner - localization 이후 2초 지연
    path_planner_launch = TimerAction(
        period=2.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution([
                        FindPackageShare('path_planner'),
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

    # 4. Control - 경로 플래너 이후 6초 지연
    control_launch = TimerAction(
        period=2.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution([
                        FindPackageShare('control'),
                        'launch',
                        'pure_pursuit_launch.py'
                    ])
                )
            )
        ]
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='f1tenth_rviz',
        output='screen',
        arguments=['-d', LaunchConfiguration('rviz_config')],
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }]
    )

    return LaunchDescription([
        use_sim_time_arg,
        map_arg,
        map_topic_arg,
        global_config_arg,
        local_config_arg,
        rviz_config_arg,
        map_server_launch,
        lifecycle_manager_node,
        localization_launch,
        path_planner_launch,
        control_launch,
        rviz_node,
    ])
