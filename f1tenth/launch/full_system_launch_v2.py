#!/usr/bin/env python3
"""
F1TENTH Real Vehicle Full System Launch
=======================================
실차 운용을 위한 통합 런치 파일 (레이싱 로직 적용)

실행 순서:
1. Map Server (정적 맵 제공)
2. Localization (EKF + AMCL) - 2초 지연
3. Racing Logic (opponent_detection, spliner, state_machine, controller) - 추가 2초 지연
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
    """F1TENTH 통합 실차 시스템 런치 (레이싱 로직 적용)"""

    # Launch arguments
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Simulation time 사용 여부 (실차는 false)'
    )
    map_arg = DeclareLaunchArgument(
        'map',
        default_value=PathJoinSubstitution([
            FindPackageShare('f1tenth'),
            'maps',
            'track.yaml'
        ]),
        description='맵 YAML 절대 경로'
    )
    map_topic_arg = DeclareLaunchArgument(
        'map_topic',
        default_value='map',
        description='맵 토픽 이름'
    )
    rviz_config_arg = DeclareLaunchArgument(
        'rviz_config',
        default_value=PathJoinSubstitution([
            FindPackageShare('f1tenth'),
            'rviz',
            'f1tenth.rviz'
        ]),
        description='RViz 설정 파일 경로'
    )

    # 최종 Waypoint 파일 경로 설정
    pkg_path_planner = FindPackageShare('path_planner')
    waypoint_file_path = PathJoinSubstitution([
        pkg_path_planner,
        'data',
        'final_waypoints.csv'
    ])

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

    # 3. Racing Logic Nodes - localization 이후 2초 지연
    racing_logic_launch = TimerAction(
        period=4.0,  # map server(0s) + localization(2s) + 추가 대기(2s)
        actions=[
            # Obstacle Detection Node
            Node(
                package='path_planner',
                executable='opponent_detection_node',
                name='opponent_detection',
                output='screen',
                parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]
            ),
            # Spline Planner Node
            Node(
                package='path_planner',
                executable='spliner_node',
                name='spline_node',
                output='screen',
                parameters=[
                    {'waypoint_file': waypoint_file_path},
                    {'use_sim_time': LaunchConfiguration('use_sim_time')}
                ]
            ),
            # State Machine Node
            Node(
                package='path_planner',
                executable='state_machine_node',
                name='state_machine',
                output='screen',
                parameters=[
                    {'waypoint_file': waypoint_file_path},
                    {'use_sim_time': LaunchConfiguration('use_sim_time')}
                ]
            ),
            # Controller Node
            Node(
                package='path_planner',
                executable='controller_node',
                name='controller',
                output='screen',
                parameters=[
                    {'waypoint_file': waypoint_file_path},
                    {'use_sim_time': LaunchConfiguration('use_sim_time')},
                ]
            ),
        ]
    )

    # RViz
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
        # Launch Arguments
        use_sim_time_arg,
        map_arg,
        map_topic_arg,
        rviz_config_arg,

        # Launch Sequence
        map_server_launch,
        lifecycle_manager_node,
        localization_launch,
        racing_logic_launch,
        rviz_node,
    ])
