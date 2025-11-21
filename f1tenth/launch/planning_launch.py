#!/usr/bin/env python3
"""
Unified Path Planner Launch File
=================================
글로벌 플래너와 로컬 플래너를 YAML 설정 파일로 선택 가능한 통합 런치 파일
"""

import math
import os
from pathlib import Path
import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory

from f1tenth.planning.tools.map_graph import GraphConfig, MapGraphBuilder


DEFAULT_GRAPH_SETTINGS = {
    'xy_resolution': 0.1,
    'theta_resolution': math.pi / 12.0,
    'vehicle_length': 0.50,
    'vehicle_width': 0.25,
    'safety_margin': 0.1,
    'graph_dir': 'data',
    'auto_generate': True,
}


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


def _get_src_base(pkg_share: str) -> Path:
    parts = pkg_share.split(os.sep)
    if 'install' in parts:
        install_index = parts.index('install')
        workspace_root = os.sep.join(parts[:install_index]) or os.sep
        return Path(workspace_root) / 'src' / 'f1tenth'
    return Path(pkg_share)


def _resolve_graph_dir(pkg_share: str, graph_dir: str) -> Path:
    graph_dir_path = Path(graph_dir)
    if graph_dir_path.is_absolute():
        return graph_dir_path
    return (_get_src_base(pkg_share) / graph_dir_path).resolve()


def _ensure_map_graph(map_yaml_path: str, graph_dir: Path, graph_config: GraphConfig) -> str:
    map_yaml = Path(map_yaml_path)
    if not map_yaml.exists():
        raise FileNotFoundError(f"Map YAML not found: {map_yaml}")

    graph_dir.mkdir(parents=True, exist_ok=True)
    graph_path = graph_dir / f"{map_yaml.stem}_graph.npz"

    if graph_path.exists():
        return str(graph_path)

    builder = MapGraphBuilder(str(map_yaml), graph_config)
    builder.load_map()
    graph = builder.build_graph()
    graph.save(str(graph_path))
    return str(graph_path)


def launch_setup(context, *args, **kwargs):
    """런치 인자 해석 후 호출되는 설정 함수"""
    pkg_share = get_package_share_directory('f1tenth')
    ws_root = get_workspace_root()

    # 런치 인자 해석
    global_config = LaunchConfiguration('global_config').perform(context)
    local_config = LaunchConfiguration('local_config').perform(context)
    is_integrated = LaunchConfiguration('is_integrated').perform(context).lower() == 'true'
    map_yaml = LaunchConfiguration('map_yaml').perform(context)

    auto_generate = LaunchConfiguration('map_graph_auto_generate').perform(context).lower() == 'true'
    graph_dir_arg = LaunchConfiguration('map_graph_dir').perform(context)
    xy_res = float(LaunchConfiguration('map_graph_xy_resolution').perform(context))
    theta_res = float(LaunchConfiguration('map_graph_theta_resolution').perform(context))
    vehicle_len = float(LaunchConfiguration('map_graph_vehicle_length').perform(context))
    vehicle_wid = float(LaunchConfiguration('map_graph_vehicle_width').perform(context))
    safety_margin = float(LaunchConfiguration('map_graph_safety_margin').perform(context))

    # 파일명만 제공된 경우 전체 경로로 변환
    if not os.path.isabs(global_config):
        global_config = os.path.join(pkg_share, 'config', 'planning', global_config)
    if not os.path.isabs(local_config):
        local_config = os.path.join(pkg_share, 'config', 'planning', local_config)

    if auto_generate:
        graph_config = GraphConfig(
            xy_resolution=xy_res,
            theta_resolution=theta_res,
            vehicle_length=vehicle_len,
            vehicle_width=vehicle_wid,
            safety_margin=safety_margin,
        )
        graph_dir = _resolve_graph_dir(pkg_share, graph_dir_arg)
        try:
            graph_path = _ensure_map_graph(map_yaml, graph_dir, graph_config)
            print(f"[Planning Launch] Map graph ready: {graph_path}")
        except Exception as exc:
            print(f"[Planning Launch] WARNING: could not generate map graph: {exc}")
    else:
        print("[Planning Launch] Map graph auto-generation disabled.")

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
        DeclareLaunchArgument(
            'map_yaml',
            default_value=PathJoinSubstitution([
                FindPackageShare('f1tenth'),
                'maps',
                'track.yaml'
            ]),
            description='Hybrid A*에서 사용할 맵 YAML 절대 경로'
        ),
        DeclareLaunchArgument(
            'map_graph_auto_generate',
            default_value='true',
            description='맵 그래프 자동 생성 여부'
        ),
        DeclareLaunchArgument(
            'map_graph_dir',
            default_value='data',
            description='그래프 저장 폴더 (src/f1tenth 기준 상대 경로 또는 절대 경로)'
        ),
        DeclareLaunchArgument(
            'map_graph_xy_resolution',
            default_value=str(DEFAULT_GRAPH_SETTINGS['xy_resolution']),
            description='그래프 XY 해상도 (m)'
        ),
        DeclareLaunchArgument(
            'map_graph_theta_resolution',
            default_value=str(DEFAULT_GRAPH_SETTINGS['theta_resolution']),
            description='그래프 각도 해상도 (rad)'
        ),
        DeclareLaunchArgument(
            'map_graph_vehicle_length',
            default_value=str(DEFAULT_GRAPH_SETTINGS['vehicle_length']),
            description='차량 길이 (m)'
        ),
        DeclareLaunchArgument(
            'map_graph_vehicle_width',
            default_value=str(DEFAULT_GRAPH_SETTINGS['vehicle_width']),
            description='차량 너비 (m)'
        ),
        DeclareLaunchArgument(
            'map_graph_safety_margin',
            default_value=str(DEFAULT_GRAPH_SETTINGS['safety_margin']),
            description='안전 마진 (m)'
        ),

        # simulation 및 path_planner 노드들 실행
        OpaqueFunction(function=launch_setup)
    ])
