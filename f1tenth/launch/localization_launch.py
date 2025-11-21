#!/usr/bin/env python3
"""
Unified Localization Launch File
=================================
글로벌 로컬라이제이션과 로컬 로컬라이제이션을 YAML 설정 파일로 선택 가능한 통합 런치 파일
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


def launch_setup(context, *args, **kwargs):
    """런치 인자 해석 후 호출되는 설정 함수"""
    pkg_share = get_package_share_directory('f1tenth')
    ws_root = get_workspace_root()

    # 런치 인자 해석
    global_config = LaunchConfiguration('global_config').perform(context)
    local_config = LaunchConfiguration('local_config').perform(context)
    use_sim_time = LaunchConfiguration('use_sim_time').perform(context).lower() == 'true'

    # 파일명만 제공된 경우 전체 경로로 변환
    if not os.path.isabs(global_config):
        global_config = os.path.join(pkg_share, 'config', 'localization', global_config)
    if not os.path.isabs(local_config):
        local_config = os.path.join(pkg_share, 'config', 'localization', local_config)

    # 설정 파일 로드
    global_params = load_yaml_params(global_config, ws_root)
    local_params = load_yaml_params(local_config, ws_root)

    # YAML에서 노드 정보 추출 (첫 번째 키를 자동으로 가져옴)
    global_key = list(global_params.keys())[0]  # 'amcl' 등
    local_key = list(local_params.keys())[0]    # 'ekf_filter_node' 등

    global_node_info = global_params[global_key].get('ros__parameters', {})
    local_node_info = local_params[local_key].get('ros__parameters', {})

    # 노드 메타데이터 추출
    global_package = global_node_info.pop('node_package', 'nav2_amcl')
    global_executable = global_node_info.pop('node_executable', 'amcl')
    global_name = global_node_info.pop('node_name', 'amcl')

    local_package = local_node_info.pop('node_package', 'robot_localization')
    local_executable = local_node_info.pop('node_executable', 'ekf_node')
    local_name = local_node_info.pop('node_name', 'ekf_filter_node')

    # use_sim_time 파라미터 추가
    global_node_info['use_sim_time'] = use_sim_time
    local_node_info['use_sim_time'] = use_sim_time

    # 노드 생성
    global_node = Node(
        package=global_package,
        executable=global_executable,
        name=global_name,
        output='screen',
        parameters=[global_node_info]
    )

    local_node = Node(
        package=local_package,
        executable=local_executable,
        name=local_name,
        output='screen',
        parameters=[local_node_info]
    )

    # TF to Odometry 노드 - map->base_link를 odometry로 발행
    odom_publisher_node = Node(
        package='f1tenth',
        executable='odom_publisher_node',
        name='odom_publisher_node',
        output='screen',
        parameters=[
            {
                'use_sim_time': use_sim_time,
                'source_frame': 'map',
                'target_frame': 'base_link',
                'odom_topic': '/odom',
                'publish_rate': 30.0
            }
        ]
    )

    return [local_node, global_node, odom_publisher_node]


def generate_launch_description():
    """설정 가능한 글로벌/로컬 로컬라이제이션으로 런치 디스크립션 생성"""

    return LaunchDescription([
        # 런치 인자
        DeclareLaunchArgument(
            'global_config',
            default_value='global_amcl.yaml',
            description='글로벌 로컬라이제이션 설정 파일 (파일명 또는 절대 경로)'
        ),
        DeclareLaunchArgument(
            'local_config',
            default_value='local_ekf.yaml',
            description='로컬 로컬라이제이션 설정 파일 (파일명 또는 절대 경로)'
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation time'
        ),

        # localization 노드들 실행
        OpaqueFunction(function=launch_setup)
    ])
