#!/usr/bin/env python3
"""
Graph Generator Launch File
============================
그래프 생성 노드를 실행하는 런치 파일
- global_planner 노드 실행 (planning_launch.py에서 복사)
- map_server 실행
- graph_generator 노드 실행
"""

import os
import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def load_yaml_params(yaml_path, workspace_root):
    """YAML 파일 로드 및 ${WORKSPACE_ROOT} 플레이스홀더 치환"""
    with open(yaml_path, 'r') as f:
        params = yaml.safe_load(f)

    # 모든 문자열 값에서 ${WORKSPACE_ROOT}를 재귀적으로 치환
    def substitute_workspace_root(obj):
        if isinstance(obj, dict):
            return {k: substitute_workspace_root(v) for k, v in obj.items()}
        elif isinstance(obj, list):
            return [substitute_workspace_root(v) for v in obj]
        elif isinstance(obj, str):
            return obj.replace('${WORKSPACE_ROOT}', workspace_root)
        return obj

    return substitute_workspace_root(params)


def get_workspace_root():
    """COLCON_PREFIX_PATH에서 워크스페이스 루트 경로 가져오기"""
    prefix_path = os.environ.get('COLCON_PREFIX_PATH', '/home/tomark/f1_ws/install')
    install_path = prefix_path.split(':')[0]
    return os.path.dirname(install_path)  # 'install' 제거하여 워크스페이스 루트 획득


def get_src_path():
    """워크스페이스의 src 경로 가져오기"""
    ws_root = get_workspace_root()
    return os.path.join(ws_root, 'src')


def launch_setup(context, *args, **kwargs):
    """런치 인자 해석 후 호출되는 설정 함수"""
    pkg_share = get_package_share_directory('f1tenth')
    ws_root = get_workspace_root()
    src_path = get_src_path()

    # 런치 인자 해석
    global_config = LaunchConfiguration('global_config').perform(context)
    map_yaml = LaunchConfiguration('map_yaml').perform(context)
    output_filename = LaunchConfiguration('output_filename').perform(context)
    long_interval = float(LaunchConfiguration('longitudinal_sampling_interval').perform(context))
    lat_interval = float(LaunchConfiguration('lateral_sampling_interval').perform(context))
    max_lat_dist = float(LaunchConfiguration('max_lateral_distance').perform(context))
    min_wall_clear = float(LaunchConfiguration('min_wall_clearance').perform(context))

    # 파일명만 제공된 경우 전체 경로로 변환
    if not os.path.isabs(global_config):
        global_config = os.path.join(pkg_share, 'config', 'planning', global_config)

    if not os.path.isabs(map_yaml):
        map_yaml = os.path.join(pkg_share, 'maps', map_yaml)

    # 출력 경로 설정 (src 디렉토리 내에 저장)
    output_graph_path = os.path.join(src_path, 'f1tenth', 'data', output_filename)

    # 글로벌 플래너 설정 파일 로드
    global_params = load_yaml_params(global_config, ws_root)

    # YAML에서 노드 정보 추출
    global_node_info = global_params.get('global_planner', {}).get('ros__parameters', {})

    # 노드 메타데이터 추출
    global_executable = global_node_info.pop('node_executable', 'global_centerline_node')
    global_name = global_node_info.pop('node_name', 'global_planner_node')

    # 1. Global Planner 노드 생성 (planning_launch.py에서 복사)
    # 그래프 생성용이므로 repeat_count=1로 오버라이드 (한 바퀴만 필요)
    global_node_info['repeat_count'] = 1

    global_node = Node(
        package='f1tenth',
        executable=global_executable,
        name=global_name,
        output='screen',
        parameters=[global_node_info]
    )

    # 2. Map Server 런치 (map_server_launch.py 사용)
    map_server_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, 'launch', 'utils', 'map_server_launch.py')
        ),
        launch_arguments={
            'map': map_yaml,
            'use_sim_time': 'false',
            'map_topic': 'map',
        }.items()
    )

    # 3. Map Server Lifecycle Manager
    lifecycle_manager_node = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_graph_gen',
        output='screen',
        parameters=[{
            'use_sim_time': False,
            'autostart': True,
            'node_names': ['map_server'],
        }]
    )

    # 4. Graph Generator 노드
    graph_generator_node = Node(
        package='f1tenth',
        executable='graph_generator',
        name='graph_generator',
        output='screen',
        parameters=[{
            'global_path_topic': global_node_info.get('publish_topic', '/global_path'),
            'map_topic': 'map',
            'output_graph_path': output_graph_path,
            'longitudinal_sampling_interval': long_interval,
            'lateral_sampling_interval': lat_interval,
            'max_lateral_distance': max_lat_dist,
            'min_wall_clearance': min_wall_clear,
        }]
    )

    return [global_node, map_server_launch, lifecycle_manager_node, graph_generator_node]


def generate_launch_description():
    """그래프 생성을 위한 런치 디스크립션 생성"""

    return LaunchDescription([
        # 런치 인자
        DeclareLaunchArgument(
            'global_config',
            default_value='global_checkpoint.yaml',
            description='글로벌 플래너 설정 파일 (파일명 또는 절대 경로)'
        ),
        DeclareLaunchArgument(
            'map_yaml',
            default_value='track.yaml',
            description='맵 YAML 파일 (파일명 또는 절대 경로)'
        ),
        DeclareLaunchArgument(
            'output_filename',
            default_value='graph.npz',
            description='출력 그래프 파일명 (src/f1tenth/data/ 디렉토리에 저장됨)'
        ),
        DeclareLaunchArgument(
            'longitudinal_sampling_interval',
            default_value='0.5',
            description='글로벌 경로 종방향 샘플링 간격 (m)'
        ),
        DeclareLaunchArgument(
            'lateral_sampling_interval',
            default_value='0.5',
            description='법선 방향 횡방향 샘플링 간격 (m)'
        ),
        DeclareLaunchArgument(
            'max_lateral_distance',
            default_value='2.5',
            description='법선 방향 최대 샘플링 거리 (m)'
        ),
        DeclareLaunchArgument(
            'min_wall_clearance',
            default_value='0.2',
            description='노드 생성 시 벽까지의 최소 거리 (m)'
        ),
        # 노드 실행
        OpaqueFunction(function=launch_setup)
    ])
