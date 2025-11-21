#!/usr/bin/env python3
"""
Graph Generator Utility Launch
===============================
맵 YAML을 전처리하여 Hybrid A* 등에서 사용할 그래프(.npz)를 생성하는 런치 파일.

설정 값은 런치 인자 또는 YAML 설정 파일을 통해 조정할 수 있으며,
기본값은 f1tenth 패키지의 maps/data 폴더를 기준으로 한다.
"""

import os
from pathlib import Path

import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory

from f1tenth.utils.graph_generator import GraphConfig, MapGraphBuilder


def _resolve_with_share(pkg_share: str, value: str) -> str:
    """상대 경로를 패키지 share 디렉터리 기준으로 변환"""
    if os.path.isabs(value):
        return value
    return os.path.join(pkg_share, value)


def _load_config(pkg_share: str, config_file: str):
    """그래프 생성 설정 YAML 로드"""
    if not config_file:
        return {}

    config_path = _resolve_with_share(pkg_share, config_file)
    if not os.path.exists(config_path):
        raise FileNotFoundError(f"Graph generator config not found: {config_path}")

    with open(config_path, 'r', encoding='utf-8') as f:
        config = yaml.safe_load(f) or {}

    if isinstance(config, dict) and 'graph_generator' in config:
        return config['graph_generator']
    return config if isinstance(config, dict) else {}


def _generator_setup(context, *args, **kwargs):
    pkg_share = get_package_share_directory('f1tenth')

    config_overrides = _load_config(pkg_share, LaunchConfiguration('config_file').perform(context))

    map_yaml = _resolve_with_share(
        pkg_share,
        config_overrides.get('map_yaml', LaunchConfiguration('map_yaml').perform(context))
    )
    output_dir = _resolve_with_share(
        pkg_share,
        config_overrides.get('output_dir', LaunchConfiguration('output_dir').perform(context))
    )
    graph_name = str(config_overrides.get('graph_name', LaunchConfiguration('graph_name').perform(context)))

    xy_resolution = float(config_overrides.get('xy_resolution', LaunchConfiguration('xy_resolution').perform(context)))
    theta_resolution = float(config_overrides.get('theta_resolution', LaunchConfiguration('theta_resolution').perform(context)))
    vehicle_length = float(config_overrides.get('vehicle_length', LaunchConfiguration('vehicle_length').perform(context)))
    vehicle_width = float(config_overrides.get('vehicle_width', LaunchConfiguration('vehicle_width').perform(context)))
    safety_margin = float(config_overrides.get('safety_margin', LaunchConfiguration('safety_margin').perform(context)))

    graph_config = GraphConfig(
        xy_resolution=xy_resolution,
        theta_resolution=theta_resolution,
        vehicle_length=vehicle_length,
        vehicle_width=vehicle_width,
        safety_margin=safety_margin,
    )

    os.makedirs(output_dir, exist_ok=True)
    output_path = Path(output_dir) / graph_name
    if output_path.suffix != '.npz':
        output_path = output_path.with_suffix('.npz')

    builder = MapGraphBuilder(map_yaml, graph_config)
    print(f"[Graph Generator] Loading map: {map_yaml}")
    builder.load_map()
    print("[Graph Generator] Building graph ...")
    graph = builder.build_graph()
    graph.save(str(output_path))
    print(f"[Graph Generator] Saved graph to: {output_path}")

    return []


def generate_launch_description():
    pkg_share = FindPackageShare('f1tenth')

    return LaunchDescription([
        DeclareLaunchArgument(
            'map_yaml',
            default_value=PathJoinSubstitution([pkg_share, 'maps', 'track.yaml']),
            description='그래프를 생성할 맵 YAML 절대 경로 또는 패키지 상대 경로'
        ),
        DeclareLaunchArgument(
            'config_file',
            default_value=PathJoinSubstitution([pkg_share, 'config', 'utils', 'graph_generator.yaml']),
            description='그래프 생성 설정 YAML (선택 사항)'
        ),
        DeclareLaunchArgument(
            'output_dir',
            default_value=PathJoinSubstitution([pkg_share, 'data']),
            description='생성된 그래프 파일을 저장할 폴더'
        ),
        DeclareLaunchArgument(
            'graph_name',
            default_value='track_graph.npz',
            description='저장될 그래프 파일 이름 (확장자 미포함 시 자동 추가)'
        ),
        DeclareLaunchArgument(
            'xy_resolution',
            default_value='0.1',
            description='그래프 XY 해상도 (meters)'
        ),
        DeclareLaunchArgument(
            'theta_resolution',
            default_value=str(3.1415926535 / 12.0),
            description='그래프 각도 해상도 (radians)'
        ),
        DeclareLaunchArgument(
            'vehicle_length',
            default_value='0.50',
            description='차량 길이 (meters)'
        ),
        DeclareLaunchArgument(
            'vehicle_width',
            default_value='0.25',
            description='차량 너비 (meters)'
        ),
        DeclareLaunchArgument(
            'safety_margin',
            default_value='0.1',
            description='안전 마진 (meters)'
        ),
        OpaqueFunction(function=_generator_setup)
    ])
