#!/usr/bin/env python3
"""
Unified Controller Launch File
===============================
컨트롤러를 YAML 설정 파일로 선택 가능한 통합 런치 파일
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
    controller_config = LaunchConfiguration('controller_config').perform(context)

    # 파일명만 제공된 경우 전체 경로로 변환
    if not os.path.isabs(controller_config):
        controller_config = os.path.join(pkg_share, 'config', 'control', controller_config)

    # 설정 파일 로드
    controller_params = load_yaml_params(controller_config, ws_root)

    # YAML에서 노드 정보 추출 (첫 번째 키를 자동으로 가져옴)
    controller_key = list(controller_params.keys())[0]  # 'controller' 등
    controller_node_info = controller_params[controller_key].get('ros__parameters', {})

    # 노드 메타데이터 추출
    controller_package = controller_node_info.pop('node_package', 'f1tenth')
    controller_executable = controller_node_info.pop('node_executable', 'pure_pursuit_node')
    controller_name = controller_node_info.pop('node_name', 'pure_pursuit_controller')

    # 노드 생성
    controller_node = Node(
        package=controller_package,
        executable=controller_executable,
        name=controller_name,
        output='screen',
        parameters=[controller_node_info]
    )

    return [controller_node]


def generate_launch_description():
    """설정 가능한 컨트롤러로 런치 디스크립션 생성"""

    return LaunchDescription([
        # 런치 인자
        DeclareLaunchArgument(
            'controller_config',
            default_value='pure_pursuit.yaml',
            description='컨트롤러 설정 파일 (파일명 또는 절대 경로)'
        ),

        # controller 노드 실행
        OpaqueFunction(function=launch_setup)
    ])
