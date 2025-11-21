#!/usr/bin/env python3
"""
Unified Path Planner Launch File
=================================
글로벌 플래너와 로컬 플래너를 YAML 설정 파일로 선택 가능한 통합 런치 파일
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
    # YAML 파일 로드 및 ${WORKSPACE_ROOT} 플레이스홀더 치환
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
    # COLCON_PREFIX_PATH에서 워크스페이스 루트 경로 가져오기
    prefix_path = os.environ.get('COLCON_PREFIX_PATH', '/home/tomark/f1_ws/install')
    install_path = prefix_path.split(':')[0]
    return os.path.dirname(install_path)  # 'install' 제거하여 워크스페이스 루트 획득


def launch_setup(context, *args, **kwargs):
    """런치 인자 해석 후 호출되는 설정 함수"""
    pkg_share = get_package_share_directory('f1tenth')
    ws_root = get_workspace_root()

    # 런치 인자 해석
    global_config = LaunchConfiguration('global_config').perform(context)
    local_config = LaunchConfiguration('local_config').perform(context)
    is_integrated = LaunchConfiguration('is_integrated').perform(context).lower() == 'true'

    # 파일명만 제공된 경우 전체 경로로 변환
    if not os.path.isabs(global_config):
        global_config = os.path.join(pkg_share, 'config', 'planning', global_config)
    if not os.path.isabs(local_config):
        local_config = os.path.join(pkg_share, 'config', 'planning', local_config)

    # 설정 파일 로드
    global_params = load_yaml_params(global_config, ws_root)
    local_params = load_yaml_params(local_config, ws_root)

    # YAML에서 노드 정보 추출
    global_node_info = global_params.get('global_planner', {}).get('ros__parameters', {})
    local_node_info = local_params.get('local_planner', {}).get('ros__parameters', {})

    # 노드 메타데이터 추출
    global_executable = global_node_info.pop('node_executable', 'global_centerline_node')
    global_name = global_node_info.pop('node_name', 'global_planner_node')
    local_executable = local_node_info.pop('node_executable', 'local_avoidance_node')
    local_name = local_node_info.pop('node_name', 'local_avoidance_node')

    # 노드 생성
    global_node = Node(
        package='f1tenth',
        executable=global_executable,
        name=global_name,
        output='screen',
        parameters=[global_node_info]
    )

    local_node = Node(
        package='f1tenth',
        executable=local_executable,
        name=local_name,
        output='screen',
        parameters=[local_node_info]
    )

    # is_integrated가 false인 경우에만 simulation 포함
    nodes = [global_node, local_node]
    if not is_integrated:
        simulation_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    get_package_share_directory('simulation'),
                    'launch',
                    'gym_bridge_launch.py'
                )
            )
        )
        nodes.insert(0, simulation_launch)  # 맨 앞에 simulation 추가

    return nodes


def generate_launch_description():
    # 설정 가능한 글로벌/로컬 플래너로 런치 디스크립션 생성

    return LaunchDescription([
        # 런치 인자
        DeclareLaunchArgument(
            'global_config',
            default_value='global_checkpoint.yaml',
            description='글로벌 플래너 설정 파일 (파일명 또는 절대 경로)'
        ),
        DeclareLaunchArgument(
            'local_config',
            default_value='local_sampler.yaml',
            description='로컬 플래너 설정 파일 (파일명 또는 절대 경로)'
        ),
        DeclareLaunchArgument(
            'is_integrated',
            default_value='false',
            description='통합 런치 모드 (true: simulation 별도 실행, false: simulation 포함 실행)'
        ),
        # simulation 및 path_planner 노드들 실행
        OpaqueFunction(function=launch_setup)
    ])
