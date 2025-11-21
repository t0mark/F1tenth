#!/usr/bin/env python3
"""
Occupancy Map to Hash Graph Converter for Hybrid A*
====================================================
맵 전체를 해시 그래프로 변환하여 Hybrid A* 등의 플래너에서 사용

주요 기능:
- 맵 전체를 균일한 그리드로 변환
- 해시 기반 빠른 이웃 탐색
- 충돌 체크 및 유효성 검사
- 사전 계산된 그래프 저장/로드

사용 예:
    # 전처리 (오프라인)
    python3 -m f1tenth.utils.graph_generator \
        --map maps/track.yaml \
        --output data/track_graph.npz

    # 런타임 (노드에서)
    from f1tenth.utils.graph_generator import MapGraph
    graph = MapGraph.load('data/track_graph.npz')
    neighbors = graph.get_neighbors(x, y, theta)
"""

from __future__ import annotations

import argparse
import json
import math
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, List, Optional, Tuple

import numpy as np
import yaml
from PIL import Image
from scipy.ndimage import distance_transform_edt


@dataclass(frozen=True)
class GraphConfig:
    """그래프 생성 설정"""
    xy_resolution: float  # 그리드 해상도 (m)
    theta_resolution: float  # 각도 해상도 (rad)
    vehicle_length: float  # 차량 길이 (m)
    vehicle_width: float  # 차량 너비 (m)
    safety_margin: float  # 안전 마진 (m)


@dataclass
class MapData:
    """맵 데이터"""
    occupancy_grid: np.ndarray  # [H, W] 점유 그리드 (0: free, 1: occupied)
    distance_field: np.ndarray  # [H, W] 거리 필드 (각 셀에서 가장 가까운 장애물까지 거리)
    resolution: float  # 맵 해상도 (m/pixel)
    origin: Tuple[float, float]  # 맵 원점 (x, y in meters)
    width: int  # 맵 너비 (픽셀)
    height: int  # 맵 높이 (픽셀)


class MapGraphBuilder:
    """맵 전체를 그래프로 변환하는 빌더"""

    def __init__(self, map_yaml_path: str, config: GraphConfig):
        """
        Args:
            map_yaml_path: 맵 YAML 파일 경로
            config: 그래프 생성 설정
        """
        self.map_yaml_path = Path(map_yaml_path)
        self.config = config
        self.map_data: Optional[MapData] = None

    def load_map(self) -> MapData:
        """맵 YAML 파일 로드 및 파싱"""
        if not self.map_yaml_path.exists():
            raise FileNotFoundError(f"Map file not found: {self.map_yaml_path}")

        with self.map_yaml_path.open('r', encoding='utf-8') as f:
            map_config = yaml.safe_load(f)

        # 맵 메타데이터
        resolution = float(map_config.get('resolution', 0.05))
        origin = map_config.get('origin', [0.0, 0.0, 0.0])
        origin_x, origin_y = float(origin[0]), float(origin[1])
        occupied_thresh = float(map_config.get('occupied_thresh', 0.65))
        free_thresh = float(map_config.get('free_thresh', 0.25))
        mode = str(map_config.get('mode', 'continuous')).lower()
        negate = bool(map_config.get('negate', False))

        # 이미지 로드
        image_path = map_config.get('image', '')
        if not image_path:
            raise ValueError("Map YAML must contain 'image' field")

        image_full_path = (self.map_yaml_path.parent / image_path).resolve()
        if not image_full_path.exists():
            raise FileNotFoundError(f"Map image not found: {image_full_path}")

        # 이미지를 grayscale로 변환
        image = Image.open(str(image_full_path)).convert('L')
        img_raw = np.array(image, dtype=np.uint8)

        # Negate 처리
        if negate:
            img_raw = 255 - img_raw

        if mode == 'trinary':
            occupancy_grid = self._build_trinary_grid(img_raw)
        else:
            img_array = img_raw.astype(np.float32) / 255.0
            occupancy_grid = self._build_threshold_grid(img_array, free_thresh, occupied_thresh)

        # 거리 필드 계산 (각 free cell에서 가장 가까운 occupied cell까지 거리)
        free_mask = (occupancy_grid == 0)
        distance_field = distance_transform_edt(free_mask, sampling=resolution).astype(np.float32)

        height, width = occupancy_grid.shape

        self.map_data = MapData(
            occupancy_grid=occupancy_grid,
            distance_field=distance_field,
            resolution=resolution,
            origin=(origin_x, origin_y),
            width=width,
            height=height
        )

        return self.map_data

    @staticmethod
    def _build_trinary_grid(img_raw: np.ndarray) -> np.ndarray:
        """trinary 모드 맵을 처리하여 점유 그리드 생성 (unknown 영역 차단)"""
        occupancy_grid = np.ones_like(img_raw, dtype=np.uint8)

        unique_vals = np.unique(img_raw)
        if len(unique_vals) == 0:
            return occupancy_grid

        free_value = int(unique_vals[-1])
        tolerance = 5
        free_mask = img_raw >= (free_value - tolerance)
        occupancy_grid[free_mask] = 0
        return occupancy_grid

    @staticmethod
    def _build_threshold_grid(img_array: np.ndarray, free_thresh: float, occupied_thresh: float) -> np.ndarray:
        """threshold 기반 점유 그리드 생성 (unknown -> occupied)"""
        occupancy_grid = np.ones(img_array.shape, dtype=np.uint8)
        free_mask = img_array >= free_thresh
        occupancy_grid[free_mask] = 0
        occupied_mask = img_array <= occupied_thresh
        occupancy_grid[occupied_mask] = 1
        return occupancy_grid

    def world_to_grid(self, x: float, y: float) -> Tuple[int, int]:
        """월드 좌표를 그리드 좌표로 변환"""
        if self.map_data is None:
            raise RuntimeError("Map not loaded. Call load_map() first.")

        origin_x, origin_y = self.map_data.origin
        col = int(math.floor((x - origin_x) / self.map_data.resolution))
        row = int(math.floor((y - origin_y) / self.map_data.resolution))

        # Y축 반전 (이미지 좌표계)
        row = self.map_data.height - 1 - row

        return row, col

    def grid_to_world(self, row: int, col: int) -> Tuple[float, float]:
        """그리드 좌표를 월드 좌표로 변환"""
        if self.map_data is None:
            raise RuntimeError("Map not loaded. Call load_map() first.")

        # Y축 반전 (이미지 좌표계)
        row_world = self.map_data.height - 1 - row

        origin_x, origin_y = self.map_data.origin
        x = origin_x + (col + 0.5) * self.map_data.resolution
        y = origin_y + (row_world + 0.5) * self.map_data.resolution

        return x, y

    def is_valid_position(self, x: float, y: float) -> bool:
        """위치가 충돌 없이 유효한지 체크 (안전 마진 포함)"""
        if self.map_data is None:
            raise RuntimeError("Map not loaded. Call load_map() first.")

        row, col = self.world_to_grid(x, y)

        # 맵 경계 체크
        if row < 0 or row >= self.map_data.height or col < 0 or col >= self.map_data.width:
            return False

        # 장애물 체크 (거리 필드 사용)
        distance = self.map_data.distance_field[row, col]
        min_required_distance = self.config.safety_margin + max(
            self.config.vehicle_length, self.config.vehicle_width
        ) / 2.0

        return distance >= min_required_distance

    def is_valid_configuration(self, x: float, y: float, theta: float) -> bool:
        """
        차량 형상을 고려한 충돌 체크

        Args:
            x, y: 차량 중심 위치 (m)
            theta: 차량 방향 (rad)

        Returns:
            충돌이 없으면 True
        """
        if self.map_data is None:
            raise RuntimeError("Map not loaded. Call load_map() first.")

        # 차량의 네 모서리 좌표 계산
        half_length = self.config.vehicle_length / 2.0
        half_width = self.config.vehicle_width / 2.0

        cos_theta = math.cos(theta)
        sin_theta = math.sin(theta)

        # 차량의 네 모서리 (로컬 좌표)
        corners_local = [
            (half_length, half_width),
            (half_length, -half_width),
            (-half_length, half_width),
            (-half_length, -half_width),
        ]

        # 월드 좌표로 변환 후 체크
        for local_x, local_y in corners_local:
            world_x = x + local_x * cos_theta - local_y * sin_theta
            world_y = y + local_x * sin_theta + local_y * cos_theta

            if not self.is_valid_position(world_x, world_y):
                return False

        # 차량 중심도 체크
        return self.is_valid_position(x, y)

    def get_distance_to_obstacle(self, x: float, y: float) -> float:
        """주어진 위치에서 가장 가까운 장애물까지의 거리"""
        if self.map_data is None:
            raise RuntimeError("Map not loaded. Call load_map() first.")

        row, col = self.world_to_grid(x, y)

        # 맵 경계 체크
        if row < 0 or row >= self.map_data.height or col < 0 or col >= self.map_data.width:
            return 0.0

        return float(self.map_data.distance_field[row, col])

    def build_graph(self) -> MapGraph:
        """
        맵 전체를 그래프로 변환

        Returns:
            MapGraph 객체
        """
        if self.map_data is None:
            self.load_map()

        print(f"Building graph with resolution: xy={self.config.xy_resolution}m, theta={self.config.theta_resolution}rad")
        print(f"Map size: {self.map_data.width} x {self.map_data.height} pixels")
        print(f"Map resolution: {self.map_data.resolution} m/pixel")

        return MapGraph(self.map_data, self.config)


class MapGraph:
    """런타임에서 사용하는 해시 그래프"""

    def __init__(self, map_data: MapData, config: GraphConfig):
        """
        Args:
            map_data: 로드된 맵 데이터
            config: 그래프 설정
        """
        self.map_data = map_data
        self.config = config

        # 각도 해상도로 discretize
        self.num_theta_bins = int(math.ceil(2 * math.pi / config.theta_resolution))
        self.theta_resolution = 2 * math.pi / self.num_theta_bins

    def discretize_state(self, x: float, y: float, theta: float) -> Tuple[int, int, int]:
        """
        연속 state를 이산 그리드 인덱스로 변환

        Args:
            x, y: 위치 (m)
            theta: 방향 (rad)

        Returns:
            (grid_x, grid_y, theta_idx)
        """
        grid_x = int(math.floor(x / self.config.xy_resolution))
        grid_y = int(math.floor(y / self.config.xy_resolution))

        # Theta를 [0, 2π) 범위로 정규화
        theta_normalized = theta % (2 * math.pi)
        theta_idx = int(math.floor(theta_normalized / self.theta_resolution))
        theta_idx = theta_idx % self.num_theta_bins

        return grid_x, grid_y, theta_idx

    def continuous_from_discrete(self, grid_x: int, grid_y: int, theta_idx: int) -> Tuple[float, float, float]:
        """
        이산 그리드 인덱스를 연속 state로 변환 (그리드 중심점)

        Args:
            grid_x, grid_y: 그리드 인덱스
            theta_idx: 각도 인덱스

        Returns:
            (x, y, theta)
        """
        x = (grid_x + 0.5) * self.config.xy_resolution
        y = (grid_y + 0.5) * self.config.xy_resolution
        theta = (theta_idx + 0.5) * self.theta_resolution

        return x, y, theta

    def is_valid_position(self, x: float, y: float) -> bool:
        """위치가 충돌 없이 유효한지 체크"""
        # 맵 데이터의 grid 좌표로 변환
        origin_x, origin_y = self.map_data.origin
        col = int(math.floor((x - origin_x) / self.map_data.resolution))
        row = int(math.floor((y - origin_y) / self.map_data.resolution))

        # Y축 반전 (이미지 좌표계)
        row = self.map_data.height - 1 - row

        # 맵 경계 체크
        if row < 0 or row >= self.map_data.height or col < 0 or col >= self.map_data.width:
            return False

        # 장애물 체크
        distance = self.map_data.distance_field[row, col]
        min_required_distance = self.config.safety_margin + max(
            self.config.vehicle_length, self.config.vehicle_width
        ) / 2.0

        return distance >= min_required_distance

    def is_valid_configuration(self, x: float, y: float, theta: float) -> bool:
        """
        차량 형상을 고려한 충돌 체크

        Args:
            x, y: 차량 중심 위치 (m)
            theta: 차량 방향 (rad)

        Returns:
            충돌이 없으면 True
        """
        # 차량의 네 모서리 좌표 계산
        half_length = self.config.vehicle_length / 2.0
        half_width = self.config.vehicle_width / 2.0

        cos_theta = math.cos(theta)
        sin_theta = math.sin(theta)

        # 차량의 네 모서리 (로컬 좌표)
        corners_local = [
            (half_length, half_width),
            (half_length, -half_width),
            (-half_length, half_width),
            (-half_length, -half_width),
        ]

        # 월드 좌표로 변환 후 체크
        for local_x, local_y in corners_local:
            world_x = x + local_x * cos_theta - local_y * sin_theta
            world_y = y + local_x * sin_theta + local_y * cos_theta

            if not self.is_valid_position(world_x, world_y):
                return False

        # 차량 중심도 체크
        return self.is_valid_position(x, y)

    def get_distance_to_obstacle(self, x: float, y: float) -> float:
        """주어진 위치에서 가장 가까운 장애물까지의 거리"""
        origin_x, origin_y = self.map_data.origin
        col = int(math.floor((x - origin_x) / self.map_data.resolution))
        row = int(math.floor((y - origin_y) / self.map_data.resolution))

        # Y축 반전 (이미지 좌표계)
        row = self.map_data.height - 1 - row

        # 맵 경계 체크
        if row < 0 or row >= self.map_data.height or col < 0 or col >= self.map_data.width:
            return 0.0

        return float(self.map_data.distance_field[row, col])

    def get_neighbors(
        self,
        x: float,
        y: float,
        theta: float,
        motion_primitives: Optional[List[Tuple[float, float, float]]] = None
    ) -> List[Tuple[float, float, float, float]]:
        """
        주어진 state의 이웃 노드 반환

        Args:
            x, y: 현재 위치 (m)
            theta: 현재 방향 (rad)
            motion_primitives: 모션 프리미티브 리스트 [(dx, dy, dtheta), ...]
                               None이면 기본 프리미티브 사용

        Returns:
            List of (next_x, next_y, next_theta, cost)
        """
        if motion_primitives is None:
            motion_primitives = self._get_default_motion_primitives()

        neighbors = []

        for dx, dy, dtheta in motion_primitives:
            # 차량 좌표계에서 월드 좌표계로 변환
            cos_theta = math.cos(theta)
            sin_theta = math.sin(theta)

            next_x = x + dx * cos_theta - dy * sin_theta
            next_y = y + dx * sin_theta + dy * cos_theta
            next_theta = theta + dtheta

            # 각도 정규화
            next_theta = (next_theta + math.pi) % (2 * math.pi) - math.pi

            # 충돌 체크
            if self.is_valid_configuration(next_x, next_y, next_theta):
                # Cost 계산 (유클리드 거리)
                cost = math.sqrt(dx**2 + dy**2)
                neighbors.append((next_x, next_y, next_theta, cost))

        return neighbors

    def _get_default_motion_primitives(self) -> List[Tuple[float, float, float]]:
        """
        기본 모션 프리미티브 (Ackermann 스타일)

        Returns:
            List of (dx, dy, dtheta) in vehicle frame
        """
        step = self.config.xy_resolution

        # 조향각 옵션
        steering_angles = [-0.3, -0.15, 0.0, 0.15, 0.3]  # rad

        primitives = []
        wheelbase = 0.33  # F1TENTH wheelbase (m)

        for steer in steering_angles:
            # Ackermann 운동학
            # dx = step (전진)
            # dtheta = (step / wheelbase) * tan(steer)
            dx = step
            dtheta = (step / wheelbase) * math.tan(steer)
            dy = 0.0  # 차량 좌표계에서 횡방향 이동 없음

            primitives.append((dx, dy, dtheta))

        # 후진 추가 (옵션)
        # for steer in [-0.15, 0.0, 0.15]:
        #     dx = -step
        #     dtheta = -(step / wheelbase) * math.tan(steer)
        #     dy = 0.0
        #     primitives.append((dx, dy, dtheta))

        return primitives

    def save(self, output_path: str) -> None:
        """그래프를 파일로 저장"""
        output_path = Path(output_path)
        output_path.parent.mkdir(parents=True, exist_ok=True)

        # NPZ 형식으로 저장
        np.savez_compressed(
            output_path,
            occupancy_grid=self.map_data.occupancy_grid,
            distance_field=self.map_data.distance_field,
            map_resolution=self.map_data.resolution,
            map_origin=np.array(self.map_data.origin),
            map_width=self.map_data.width,
            map_height=self.map_data.height,
            xy_resolution=self.config.xy_resolution,
            theta_resolution=self.config.theta_resolution,
            vehicle_length=self.config.vehicle_length,
            vehicle_width=self.config.vehicle_width,
            safety_margin=self.config.safety_margin,
        )

        # 메타데이터 JSON 저장
        meta_path = output_path.with_suffix('.json')
        metadata = {
            'map_width': int(self.map_data.width),
            'map_height': int(self.map_data.height),
            'map_resolution': float(self.map_data.resolution),
            'map_origin': [float(x) for x in self.map_data.origin],
            'xy_resolution': float(self.config.xy_resolution),
            'theta_resolution': float(self.config.theta_resolution),
            'num_theta_bins': int(self.num_theta_bins),
            'vehicle_length': float(self.config.vehicle_length),
            'vehicle_width': float(self.config.vehicle_width),
            'safety_margin': float(self.config.safety_margin),
        }

        with meta_path.open('w', encoding='utf-8') as f:
            json.dump(metadata, f, indent=2, sort_keys=True)

        print(f"Graph saved to {output_path}")
        print(f"Metadata saved to {meta_path}")

    @classmethod
    def load(cls, graph_path: str) -> MapGraph:
        """파일에서 그래프 로드"""
        graph_path = Path(graph_path)

        if not graph_path.exists():
            raise FileNotFoundError(f"Graph file not found: {graph_path}")

        # NPZ 로드
        data = np.load(graph_path)

        # MapData 복원
        map_data = MapData(
            occupancy_grid=data['occupancy_grid'],
            distance_field=data['distance_field'],
            resolution=float(data['map_resolution']),
            origin=tuple(data['map_origin']),
            width=int(data['map_width']),
            height=int(data['map_height']),
        )

        # GraphConfig 복원
        config = GraphConfig(
            xy_resolution=float(data['xy_resolution']),
            theta_resolution=float(data['theta_resolution']),
            vehicle_length=float(data['vehicle_length']),
            vehicle_width=float(data['vehicle_width']),
            safety_margin=float(data['safety_margin']),
        )

        print(f"Graph loaded from {graph_path}")
        return cls(map_data, config)


def parse_args() -> argparse.Namespace:
    """커맨드라인 인자 파싱"""
    parser = argparse.ArgumentParser(
        description='Convert occupancy map to hash graph for Hybrid A*'
    )

    parser.add_argument(
        '--map',
        type=Path,
        required=True,
        help='Path to map YAML file'
    )

    parser.add_argument(
        '--output',
        type=Path,
        default=Path('data/map_graph.npz'),
        help='Output path for graph file (default: data/map_graph.npz)'
    )

    parser.add_argument(
        '--xy-resolution',
        type=float,
        default=0.1,
        help='Grid resolution in meters (default: 0.1)'
    )

    parser.add_argument(
        '--theta-resolution',
        type=float,
        default=math.pi / 12,
        help='Theta resolution in radians (default: π/12)'
    )

    parser.add_argument(
        '--vehicle-length',
        type=float,
        default=0.50,
        help='Vehicle length in meters (default: 0.50)'
    )

    parser.add_argument(
        '--vehicle-width',
        type=float,
        default=0.25,
        help='Vehicle width in meters (default: 0.25)'
    )

    parser.add_argument(
        '--safety-margin',
        type=float,
        default=0.1,
        help='Safety margin in meters (default: 0.1)'
    )

    return parser.parse_args()


def main() -> None:
    """메인 함수 - 그래프 전처리 실행"""
    args = parse_args()

    # 설정 생성
    config = GraphConfig(
        xy_resolution=args.xy_resolution,
        theta_resolution=args.theta_resolution,
        vehicle_length=args.vehicle_length,
        vehicle_width=args.vehicle_width,
        safety_margin=args.safety_margin,
    )

    # 그래프 빌드
    builder = MapGraphBuilder(str(args.map), config)
    builder.load_map()
    graph = builder.build_graph()

    # 저장
    graph.save(str(args.output))

    print(f"\nGraph statistics:")
    print(f"  Map size: {graph.map_data.width} x {graph.map_data.height} pixels")
    print(f"  Map resolution: {graph.map_data.resolution} m/pixel")
    print(f"  XY resolution: {config.xy_resolution} m")
    print(f"  Theta bins: {graph.num_theta_bins}")
    print(f"  Vehicle: {config.vehicle_length}m x {config.vehicle_width}m")
    print(f"  Safety margin: {config.safety_margin}m")


if __name__ == '__main__':
    main()
