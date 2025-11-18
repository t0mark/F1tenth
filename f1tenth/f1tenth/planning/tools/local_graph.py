#!/usr/bin/env python3
"""
Generate a pre-computed tube graph around the global checkpoints path.

The graph is stored as:
  - <prefix>.npz            : dense arrays (node poses, attributes, edges)
  - <prefix>_index.json     : mapping (s_idx, lat_idx, head_idx) -> node_id
  - <prefix>_meta.json      : configuration metadata
"""

from __future__ import annotations

import argparse
import csv
import json
import math
from dataclasses import dataclass
from pathlib import Path
from typing import Optional, Sequence, Tuple

import numpy as np
import yaml
from PIL import Image
from scipy.ndimage import distance_transform_edt


@dataclass(frozen=True)
class GraphData:
    node_positions: np.ndarray
    node_yaws: np.ndarray
    node_lateral_offsets: np.ndarray
    node_curvatures: np.ndarray
    node_wall_distances: np.ndarray
    node_indices: np.ndarray
    edges_from: np.ndarray
    edges_to: np.ndarray
    edges_cost: np.ndarray


def wrap_angle(angle: np.ndarray) -> np.ndarray:
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


def load_centerline_source(path: Path) -> dict:
    raw = np.genfromtxt(path, delimiter=',', names=True, dtype=None, encoding=None)
    if raw.size == 0:
        raise ValueError(f'파일 {path} 에서 데이터를 읽을 수 없습니다.')
    raw = np.atleast_1d(raw)
    names = raw.dtype.names
    if names is None or len(names) < 2:
        raise ValueError(f'파일 {path} 에 적어도 x, y 열이 필요합니다.')

    def _extract(name: str) -> Optional[np.ndarray]:
        if name in names:
            return np.asarray(raw[name], dtype=float)
        return None

    x = _extract('x_ref_m')
    if x is None:
        x = _extract('x')
    y = _extract('y_ref_m')
    if y is None:
        y = _extract('y')
    if x is None or y is None:
        raise ValueError(f'파일 {path} 에 x 좌표와 y 좌표 열이 필요합니다.')

    yaw = _extract('psi_racetraj_rad')
    s = _extract('s_racetraj_m')
    curvature = _extract('kappa_racetraj_radpm')

    return {
        'x': np.asarray(x, dtype=float),
        'y': np.asarray(y, dtype=float),
        'yaw': np.asarray(yaw, dtype=float) if yaw is not None else None,
        's': np.asarray(s, dtype=float) if s is not None else None,
        'curvature': np.asarray(curvature, dtype=float) if curvature is not None else None,
    }


def _compute_arc_length(x: np.ndarray, y: np.ndarray, closed_loop: bool) -> np.ndarray:
    if closed_loop:
        x_ext = np.append(x, x[0])
        y_ext = np.append(y, y[0])
        diff_x = np.diff(x_ext)
        diff_y = np.diff(y_ext)
        seg = np.hypot(diff_x, diff_y)
        cumulative = np.cumsum(seg)
        return np.concatenate(([0.0], cumulative[:-1]))
    diff_x = np.diff(x)
    diff_y = np.diff(y)
    seg = np.hypot(diff_x, diff_y)
    cumulative = np.concatenate(([0.0], np.cumsum(seg)))
    return cumulative


def _estimate_headings(x: np.ndarray, y: np.ndarray, s: np.ndarray, closed_loop: bool) -> np.ndarray:
    if len(x) < 2:
        return np.zeros_like(x)
    if closed_loop and len(x) >= 3:
        ds = np.gradient(s)
        dx = np.gradient(x, ds, edge_order=2)
        dy = np.gradient(y, ds, edge_order=2)
    else:
        dx = np.gradient(x, s, edge_order=2)
        dy = np.gradient(y, s, edge_order=2)
    return wrap_angle(np.arctan2(dy, dx))


def prepare_centerline(
    source: dict,
    s_step: float,
    closed_loop: bool,
) -> Tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray, float]:
    x = source['x']
    y = source['y']
    yaw = source['yaw']
    curvature = source['curvature']
    if source['s'] is not None:
        s_values = source['s']
    else:
        s_values = _compute_arc_length(x, y, closed_loop)

    s_values = np.asarray(s_values, dtype=float)
    if s_values.ndim != 1:
        s_values = s_values.reshape(-1)
    if yaw is None:
        yaw = _estimate_headings(x, y, s_values, closed_loop)
    else:
        yaw = wrap_angle(np.asarray(yaw, dtype=float))
    yaw_unwrapped = np.unwrap(yaw)
    if curvature is None:
        curvature = np.zeros_like(s_values, dtype=float)

    curvature = np.asarray(curvature, dtype=float)
    if curvature.ndim != 1:
        curvature = curvature.reshape(-1)

    total_length = float(s_values[-1])

    if s_step > 0.0:
        if closed_loop:
            target_s = np.arange(0.0, total_length, s_step, dtype=float)
        else:
            target_s = np.arange(0.0, total_length + 1e-9, s_step, dtype=float)
        if target_s.size < 2:
            target_s = np.linspace(0.0, total_length, 2, dtype=float)
        x_interp = np.interp(target_s, s_values, x)
        y_interp = np.interp(target_s, s_values, y)
        yaw_interp = np.interp(target_s, s_values, yaw_unwrapped)
        curvature_interp = np.interp(target_s, s_values, curvature)
        centerline = np.stack((x_interp, y_interp), axis=1)
        headings = wrap_angle(yaw_interp)
        return centerline, headings, target_s, curvature_interp, total_length

    centerline = np.stack((x, y), axis=1)
    headings = wrap_angle(yaw_unwrapped)
    return centerline, headings, s_values, curvature, total_length


def load_wall_distance_field(
    map_yaml: Optional[Path],
    distance_threshold: float,
) -> Tuple[Optional[np.ndarray], Tuple[float, float], float, dict]:
    if map_yaml is None:
        return None, (0.0, 0.0), 1.0, {}
    map_yaml = map_yaml.expanduser()
    if not map_yaml.exists():
        return None, (0.0, 0.0), 1.0, {}

    with map_yaml.open('r', encoding='utf-8') as f:
        map_config = yaml.safe_load(f)

    resolution = float(map_config.get('resolution', 0.05))
    origin = map_config.get('origin', [-0.5, -0.5, 0.0])
    origin_x = float(origin[0])
    origin_y = float(origin[1])
    occupied_thresh = float(map_config.get('occupied_thresh', 0.65))
    negate = bool(map_config.get('negate', False))
    image_path = map_config.get('image', '')
    if not image_path:
        return None, (origin_x, origin_y), resolution, {}
    image_full_path = (map_yaml.parent / image_path).resolve()
    if not image_full_path.exists():
        return None, (origin_x, origin_y), resolution, {}

    image = Image.open(str(image_full_path)).convert('L')
    img_array = np.array(image, dtype=np.float32) / 255.0
    if negate:
        img_array = 1.0 - img_array

    occupied_mask = img_array <= occupied_thresh
    free_mask = np.logical_not(occupied_mask)

    distance_field = distance_transform_edt(free_mask, sampling=resolution).astype(np.float32)
    distance_field = np.where(occupied_mask, 0.0, distance_field)

    metadata = {
        'map_yaml': str(map_yaml),
        'map_image': str(image_full_path),
        'map_resolution': resolution,
        'map_origin': [origin_x, origin_y],
        'occupied_thresh': occupied_thresh,
        'negate': negate,
        'distance_threshold': float(distance_threshold),
        'distance_field_shape': [int(distance_field.shape[0]), int(distance_field.shape[1])],
    }

    return distance_field, (origin_x, origin_y), resolution, metadata


def sample_wall_distances(
    node_positions: np.ndarray,
    distance_field: Optional[np.ndarray],
    origin: Tuple[float, float],
    resolution: float,
    threshold: float,
) -> np.ndarray:
    """
    각 노드 위치에서 벽까지의 유클리드 거리를 샘플링
    threshold 파라미터는 호환성을 위해 유지하지만, 거리 값을 그대로 반환
    """
    if distance_field is None:
        return np.zeros(node_positions.shape[0], dtype=np.float32)

    height, width = distance_field.shape
    origin_x, origin_y = origin
    distances = np.zeros(node_positions.shape[0], dtype=np.float32)

    for idx, (x, y) in enumerate(node_positions):
        map_x = (x - origin_x) / resolution
        map_y = (y - origin_y) / resolution
        col = int(math.floor(map_x))
        row_from_origin = int(math.floor(map_y))
        if col < 0 or col >= width or row_from_origin < 0 or row_from_origin >= height:
            distances[idx] = 0.0
            continue
        row = height - 1 - row_from_origin
        if row < 0 or row >= height:
            distances[idx] = 0.0
            continue
        # 유클리드 거리를 그대로 저장 (threshold 적용 안 함)
        dist = float(distance_field[row, col])
        distances[idx] = dist

    return distances


def build_graph(
    centerline: np.ndarray,
    headings: np.ndarray,
    s_values: np.ndarray,
    lateral_offsets: Sequence[float],
    heading_offsets: Sequence[float],
    curvature: np.ndarray,
    max_lateral_step: int,
    max_heading_step: int,
    lateral_weight: float,
    heading_weight: float,
    include_reverse: bool,
    lateral_bias_gain: float,
    lateral_bias_limit: float,
    distance_field: Optional[np.ndarray],
    map_origin: Tuple[float, float],
    map_resolution: float,
    wall_distance_threshold: float,
    curvature_cost_weight: float,
    closed_loop: bool,
) -> GraphData:
    num_s = len(centerline)
    lat_offsets = np.asarray(lateral_offsets, dtype=float)
    head_offsets = np.asarray(heading_offsets, dtype=float)

    num_lat = lat_offsets.size
    num_head = head_offsets.size
    if num_lat == 0 or num_head == 0:
        raise ValueError('At least one lateral offset and one heading offset are required.')

    normals = np.stack([-np.sin(headings), np.cos(headings)], axis=1)

    base_pos = centerline[:, None, None, :]
    base_normals = normals[:, None, None, :]

    curvature = np.asarray(curvature, dtype=float)
    if curvature.shape[0] != num_s:
        raise ValueError('곡률 배열의 길이는 centerline 샘플 개수와 동일해야 합니다.')

    bias_gain = float(lateral_bias_gain)
    bias_limit = float(max(0.0, lateral_bias_limit))
    base_abs_limit = float(np.max(np.abs(lat_offsets))) if lat_offsets.size else 0.0

    # Curvature sign convention: positive curvature turns left (bias grid towards inner radius).
    # Therefore subtract curvature*gain so positive curvature shifts offsets negative (left side).
    bias_values = -curvature * bias_gain
    if bias_limit > 0.0:
        bias_values = np.clip(bias_values, -bias_limit, bias_limit)

    biased_offsets = lat_offsets[None, :] + bias_values[:, None]
    if base_abs_limit > 0.0:
        biased_offsets = np.clip(biased_offsets, -base_abs_limit, base_abs_limit)

    pos_offsets = biased_offsets[:, :, None, None]
    node_positions = base_pos + base_normals * pos_offsets
    node_positions = np.broadcast_to(node_positions, (num_s, num_lat, num_head, 2))

    yaw_grid = headings[:, None, None] + head_offsets[None, None, :]
    node_yaws = wrap_angle(np.broadcast_to(yaw_grid, (num_s, num_lat, num_head)))

    node_positions_flat = node_positions.reshape(-1, 2).astype(np.float32)
    node_yaws_flat = node_yaws.reshape(-1).astype(np.float32)
    node_l_flat = np.broadcast_to(biased_offsets[:, :, None], (num_s, num_lat, num_head)).reshape(-1).astype(np.float32)
    node_curv_flat = np.broadcast_to(curvature[:, None, None], (num_s, num_lat, num_head)).reshape(-1).astype(np.float32)
    node_wall_dist_flat = sample_wall_distances(
        node_positions_flat,
        distance_field,
        map_origin,
        map_resolution,
        wall_distance_threshold,
    )
    s_indices = np.arange(num_s, dtype=np.int32)
    l_indices = np.arange(num_lat, dtype=np.int32)
    h_indices = np.arange(num_head, dtype=np.int32)
    mesh = np.stack(np.meshgrid(s_indices, l_indices, h_indices, indexing='ij'), axis=-1)
    node_indices = mesh.reshape(-1, 3).astype(np.int32)

    node_id_grid = np.arange(node_positions_flat.shape[0], dtype=np.int32).reshape(num_s, num_lat, num_head)

    lat_steps = [d for d in range(-max_lateral_step, max_lateral_step + 1)]
    head_steps = [d for d in range(-max_heading_step, max_heading_step + 1)]

    edges_from: list[int] = []
    edges_to: list[int] = []
    edges_cost: list[float] = []

    def add_edge(a: int, b: int) -> None:
        pos_a = node_positions_flat[a]
        pos_b = node_positions_flat[b]
        dist = float(np.linalg.norm(pos_b - pos_a))
        if dist <= 1e-6:
            return
        s_idx_a, lat_idx_a, head_idx_a = node_indices[a]
        s_idx_b, lat_idx_b, head_idx_b = node_indices[b]
        lateral_delta = abs(
            biased_offsets[s_idx_b, lat_idx_b] - biased_offsets[s_idx_a, lat_idx_a]
        )
        heading_delta = abs(head_offsets[head_idx_b] - head_offsets[head_idx_a])
        inner_a = max(0.0, curvature[s_idx_a] * biased_offsets[s_idx_a, lat_idx_a])
        inner_b = max(0.0, curvature[s_idx_b] * biased_offsets[s_idx_b, lat_idx_b])
        curvature_penalty = curvature_cost_weight * 0.5 * (inner_a + inner_b)
        weight = 1.0 + lateral_weight * lateral_delta + heading_weight * heading_delta + curvature_penalty
        edges_from.append(a)
        edges_to.append(b)
        edges_cost.append(dist * weight)

    s_iter = range(num_s) if closed_loop else range(num_s - 1)
    for s_idx in s_iter:
        next_s_idx = (s_idx + 1) % num_s
        if not closed_loop and next_s_idx == 0:
            continue
        for lat_idx in range(num_lat):
            for head_idx in range(num_head):
                src = node_id_grid[s_idx, lat_idx, head_idx]
                for d_lat in lat_steps:
                    new_lat = lat_idx + d_lat
                    if new_lat < 0 or new_lat >= num_lat:
                        continue
                    for d_head in head_steps:
                        new_head = head_idx + d_head
                        if new_head < 0 or new_head >= num_head:
                            continue
                        dst = node_id_grid[next_s_idx, new_lat, new_head]
                        add_edge(src, dst)
                        if include_reverse:
                            add_edge(dst, src)

    return GraphData(
        node_positions=node_positions_flat,
        node_yaws=node_yaws_flat,
        node_lateral_offsets=node_l_flat,
        node_curvatures=node_curv_flat,
        node_wall_distances=node_wall_dist_flat,
        node_indices=node_indices,
        edges_from=np.asarray(edges_from, dtype=np.int32),
        edges_to=np.asarray(edges_to, dtype=np.int32),
        edges_cost=np.asarray(edges_cost, dtype=np.float32),
    )


def save_graph(graph: GraphData, data_dir: Path, prefix: str, meta: dict) -> None:
    data_dir.mkdir(parents=True, exist_ok=True)

    graph_path = data_dir / f'{prefix}.npz'
    meta_path = data_dir / f'{prefix}_meta.json'

    np.savez_compressed(
        graph_path,
        node_positions=graph.node_positions,
        node_yaws=graph.node_yaws,
        node_lateral_offsets=graph.node_lateral_offsets,
        node_curvatures=graph.node_curvatures,
        node_wall_distances=graph.node_wall_distances,
        node_indices=graph.node_indices,
        edges_from=graph.edges_from,
        edges_to=graph.edges_to,
        edges_cost=graph.edges_cost,
    )

    meta = dict(meta)
    meta.update(
        {
            'graph_path': graph_path.name,
            'meta_path': meta_path.name,
            'num_nodes': int(graph.node_positions.shape[0]),
            'num_edges': int(graph.edges_from.shape[0]),
        }
    )
    with meta_path.open('w', encoding='utf-8') as f:
        json.dump(meta, f, indent=2, sort_keys=True)


def parse_args() -> argparse.Namespace:
    pkg_utils_dir = Path(__file__).resolve().parent
    pkg_dir = pkg_utils_dir.parent
    data_dir = pkg_dir.parent / 'data'

    parser = argparse.ArgumentParser(description='Precompute a local tube graph around a reference trajectory.')
    parser.add_argument(
        '--input',
        '--checkpoints',
        dest='input_path',
        type=Path,
        default=data_dir / 'fitted_waypoints.csv',
        help='Path to input CSV (supports fitted_waypoints with curvature) (default: %(default)s)',
    )
    parser.add_argument(
        '--output-dir',
        type=Path,
        default=data_dir,
        help='Destination directory for graph files (default: %(default)s)',
    )
    parser.add_argument(
        '--prefix',
        type=str,
        default='local_graph',
        help='Filename prefix for generated files (default: %(default)s)',
    )
    parser.add_argument(
        '--s-step',
        type=float,
        default=0.2,
        help='Longitudinal resampling step along the path in meters (default: %(default)s)',
    )
    parser.add_argument(
        '--tube-width',
        type=float,
        default=0.6,
        help='Half-width of the tube in meters (default: %(default)s)',
    )
    parser.add_argument(
        '--lateral-count',
        type=int,
        default=5,
        help='Number of lateral offsets across the tube (default: %(default)s)',
    )
    parser.add_argument(
        '--heading-range-deg',
        type=float,
        default=12.0,
        help='Symmetric range of heading offsets around centerline in degrees (default: %(default)s)',
    )
    parser.add_argument(
        '--heading-count',
        type=int,
        default=3,
        help='Number of heading offsets per state (default: %(default)s)',
    )

    default_map_yaml = (pkg_dir.parent.parent / 'f1tenth/maps/track.yaml')
    if not default_map_yaml.exists():
        default_map_yaml = None

    parser.add_argument(
        '--map-yaml',
        type=Path,
        default=default_map_yaml,
        help='Path to map YAML for wall distance computation (default: %(default)s)',
    )
    parser.add_argument(
        '--wall-distance-threshold',
        type=float,
        default=0.3,
        help='Distances below this value (m) are clamped to zero when stored (default: %(default)s)',
    )
    parser.add_argument(
        '--curvature-cost-weight',
        type=float,
        default=5.0,
        help='Weight applied to abs(curvature) when computing curvature-based cost (default: %(default)s)',
    )
    parser.add_argument(
        '--csv-output',
        type=Path,
        default=None,
        help='Optional path to export per-node heuristic CSV (default: <output-dir>/<prefix>_heuristics.csv)',
    )
    parser.add_argument(
        '--curvature-bias-gain',
        type=float,
        default=0.0,
        help='Gain to convert curvature (rad/m) into lateral offset bias (default: %(default)s)',
    )
    parser.add_argument(
        '--curvature-bias-limit',
        type=float,
        default=0.25,
        help='Maximum absolute lateral bias in meters applied per section (default: %(default)s)',
    )
    parser.add_argument(
        '--max-lateral-step',
        type=int,
        default=1,
        help='Maximum lateral index change permitted between longitudinal steps (default: %(default)s)',
    )
    parser.add_argument(
        '--max-heading-step',
        type=int,
        default=1,
        help='Maximum heading index change permitted between longitudinal steps (default: %(default)s)',
    )
    parser.add_argument(
        '--lateral-weight',
        type=float,
        default=1.0,
        help='Edge cost weight per meter of lateral change (default: %(default)s)',
    )
    parser.add_argument(
        '--heading-weight',
        type=float,
        default=0.5,
        help='Edge cost weight per radian of heading change (default: %(default)s)',
    )
    parser.add_argument(
        '--include-reverse',
        action='store_true',
        help='Include reverse-directed edges to enable backward traversal.',
    )
    parser.add_argument(
        '--open-path',
        dest='closed_loop',
        action='store_false',
        help='Treat the checkpoints as an open path (no wrap-around edges).',
    )
    parser.set_defaults(closed_loop=True)
    return parser.parse_args()


def compute_lateral_offsets(width: float, count: int) -> np.ndarray:
    if count <= 1:
        return np.zeros(1, dtype=float)
    return np.linspace(-width, width, count)


def compute_heading_offsets(range_deg: float, count: int) -> np.ndarray:
    if count <= 1 or range_deg <= 0.0:
        return np.zeros(1, dtype=float)
    range_rad = math.radians(range_deg)
    return np.linspace(-range_rad, range_rad, count)


def main() -> None:
    args = parse_args()

    source = load_centerline_source(args.input_path)
    centerline, headings, s_values, curvature, total_length = prepare_centerline(
        source=source,
        s_step=args.s_step,
        closed_loop=args.closed_loop,
    )

    lateral_offsets = compute_lateral_offsets(args.tube_width, args.lateral_count)
    heading_offsets = compute_heading_offsets(args.heading_range_deg, args.heading_count)

    distance_field, map_origin, map_resolution, distance_meta = load_wall_distance_field(
        args.map_yaml,
        args.wall_distance_threshold,
    )

    graph = build_graph(
        centerline=centerline,
        headings=headings,
        s_values=s_values,
        lateral_offsets=lateral_offsets,
        heading_offsets=heading_offsets,
        curvature=curvature,
        max_lateral_step=max(0, args.max_lateral_step),
        max_heading_step=max(0, args.max_heading_step),
        lateral_weight=max(0.0, args.lateral_weight),
        heading_weight=max(0.0, args.heading_weight),
        include_reverse=bool(args.include_reverse),
        lateral_bias_gain=float(args.curvature_bias_gain),
        lateral_bias_limit=float(args.curvature_bias_limit),
        distance_field=distance_field,
        map_origin=map_origin,
        map_resolution=map_resolution,
        wall_distance_threshold=float(args.wall_distance_threshold),
        curvature_cost_weight=float(args.curvature_cost_weight),
        closed_loop=bool(args.closed_loop),
    )

    meta = {
        'input': str(args.input_path),
        's_step': float(args.s_step),
        'tube_width': float(args.tube_width),
        'lateral_offsets': lateral_offsets.tolist(),
        'heading_offsets_rad': heading_offsets.tolist(),
        'max_lateral_step': int(args.max_lateral_step),
        'max_heading_step': int(args.max_heading_step),
        'lateral_weight': float(args.lateral_weight),
        'heading_weight': float(args.heading_weight),
        'include_reverse': bool(args.include_reverse),
        'curvature_bias_gain': float(args.curvature_bias_gain),
        'curvature_bias_limit': float(args.curvature_bias_limit),
        'wall_distance_threshold': float(args.wall_distance_threshold),
        'curvature_cost_weight': float(args.curvature_cost_weight),
        'closed_loop': bool(args.closed_loop),
        'centerline_length': float(total_length),
    }

    meta.update(distance_meta)

    save_graph(graph, args.output_dir, args.prefix, meta)

    csv_path = args.csv_output
    if csv_path is None:
        csv_path = args.output_dir / f'{args.prefix}_heuristics.csv'
    else:
        csv_path = csv_path.expanduser()
        if not csv_path.is_absolute():
            csv_path = (Path.cwd() / csv_path).resolve()

    csv_path.parent.mkdir(parents=True, exist_ok=True)

    with csv_path.open('w', newline='', encoding='utf-8') as csv_file:
        writer = csv.writer(csv_file)
        writer.writerow([
            's_idx',
            'lat_idx',
            'head_idx',
            'x',
            'y',
            'wall_distance',
            'curvature',
            'lateral_offset',
            'curvature_heuristic',
        ])
        for (s_idx, lat_idx, head_idx), pos, wall_dist, curv, lat_off in zip(
            graph.node_indices,
            graph.node_positions,
            graph.node_wall_distances,
            graph.node_curvatures,
            graph.node_lateral_offsets,
        ):
            # 새로운 곡률 기반 휴리스틱 계산
            # 안쪽(curvature * lateral_offset < 0): 높은 페널티
            # 바깥쪽(curvature * lateral_offset >= 0): 보너스
            if curv * lat_off < 0:
                curvature_heuristic = abs(curv * lat_off) * 3.0
            else:
                curvature_heuristic = -abs(curv * lat_off) * 0.5

            writer.writerow([
                int(s_idx),
                int(lat_idx),
                int(head_idx),
                f'{float(pos[0]):.6f}',
                f'{float(pos[1]):.6f}',
                f'{float(wall_dist):.6f}',
                f'{float(curv):.6f}',
                f'{float(lat_off):.6f}',
                f'{float(curvature_heuristic):.6f}',
            ])

    print(
        f'Graph saved to {args.output_dir} with prefix "{args.prefix}" '
        f'(nodes={graph.node_positions.shape[0]}, edges={graph.edges_from.shape[0]})\n'
        f'Heuristic CSV exported to {csv_path}'
    )


if __name__ == '__main__':
    main()
