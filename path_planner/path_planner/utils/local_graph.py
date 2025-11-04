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
import json
import math
from dataclasses import dataclass
from pathlib import Path
from typing import Sequence, Tuple

import numpy as np


@dataclass(frozen=True)
class GraphData:
    node_positions: np.ndarray
    node_yaws: np.ndarray
    node_s_values: np.ndarray
    node_lateral_offsets: np.ndarray
    node_heading_offsets: np.ndarray
    node_indices: np.ndarray
    edges_from: np.ndarray
    edges_to: np.ndarray
    edges_cost: np.ndarray
    index_map: dict


def wrap_angle(angle: np.ndarray) -> np.ndarray:
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


def load_checkpoints(path: Path) -> np.ndarray:
    data = np.genfromtxt(path, delimiter=',', skip_header=1)
    if data.ndim == 1:
        data = data.reshape(1, -1)
    if data.shape[1] < 2:
        raise ValueError(f'Checkpoint file {path} must contain at least two columns (x,y).')
    return data[:, :2]


def resample_centerline(points: np.ndarray, s_step: float, closed_loop: bool) -> Tuple[np.ndarray, np.ndarray, float]:
    if closed_loop:
        if np.linalg.norm(points[0] - points[-1]) > 1e-6:
            points_ext = np.vstack([points, points[:1]])
        else:
            points_ext = points
    else:
        points_ext = points

    deltas = np.diff(points_ext, axis=0)
    seg_lengths = np.hypot(deltas[:, 0], deltas[:, 1])
    cumulative = np.concatenate(([0.0], np.cumsum(seg_lengths)))
    total_length = cumulative[-1]
    if total_length <= 0:
        raise ValueError('Total length of checkpoints path must be positive.')

    num_samples = max(2, int(math.floor(total_length / s_step)) + 1)
    if closed_loop:
        s_values = np.linspace(0.0, total_length, num_samples, endpoint=False)
    else:
        s_values = np.linspace(0.0, total_length, num_samples, endpoint=True)

    x_samples = np.interp(s_values, cumulative, points_ext[:, 0])
    y_samples = np.interp(s_values, cumulative, points_ext[:, 1])
    resampled = np.stack((x_samples, y_samples), axis=1)
    return resampled, s_values, float(total_length)


def compute_headings(points: np.ndarray, s_values: np.ndarray, closed_loop: bool) -> np.ndarray:
    if closed_loop and len(points) >= 3:
        s_step = float(np.median(np.diff(s_values)))
        if s_step <= 0:
            s_step = 1.0
        dx = (np.roll(points[:, 0], -1) - np.roll(points[:, 0], 1)) / (2.0 * s_step)
        dy = (np.roll(points[:, 1], -1) - np.roll(points[:, 1], 1)) / (2.0 * s_step)
    else:
        dx = np.gradient(points[:, 0], s_values, edge_order=2)
        dy = np.gradient(points[:, 1], s_values, edge_order=2)
    headings = np.arctan2(dy, dx)
    return wrap_angle(headings)


def build_graph(
    centerline: np.ndarray,
    headings: np.ndarray,
    s_values: np.ndarray,
    lateral_offsets: Sequence[float],
    heading_offsets: Sequence[float],
    max_lateral_step: int,
    max_heading_step: int,
    lateral_weight: float,
    heading_weight: float,
    include_reverse: bool,
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
    pos_offsets = lat_offsets[None, :, None, None]
    node_positions = base_pos + base_normals * pos_offsets
    node_positions = np.broadcast_to(node_positions, (num_s, num_lat, num_head, 2))

    yaw_grid = headings[:, None, None] + head_offsets[None, None, :]
    node_yaws = wrap_angle(np.broadcast_to(yaw_grid, (num_s, num_lat, num_head)))

    node_positions_flat = node_positions.reshape(-1, 2).astype(np.float32)
    node_yaws_flat = node_yaws.reshape(-1).astype(np.float32)
    node_s_flat = np.broadcast_to(s_values[:, None, None], (num_s, num_lat, num_head)).reshape(-1).astype(np.float32)
    node_l_flat = np.broadcast_to(lat_offsets[None, :, None], (num_s, num_lat, num_head)).reshape(-1).astype(np.float32)
    node_h_flat = np.broadcast_to(head_offsets[None, None, :], (num_s, num_lat, num_head)).reshape(-1).astype(np.float32)

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
        lateral_delta = abs(lat_offsets[lat_idx_b] - lat_offsets[lat_idx_a])
        heading_delta = abs(head_offsets[head_idx_b] - head_offsets[head_idx_a])
        weight = 1.0 + lateral_weight * lateral_delta + heading_weight * heading_delta
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

    index_map = {
        f'{int(i)}_{int(j)}_{int(k)}': int(node_id_grid[i, j, k])
        for i in range(num_s)
        for j in range(num_lat)
        for k in range(num_head)
    }

    return GraphData(
        node_positions=node_positions_flat,
        node_yaws=node_yaws_flat,
        node_s_values=node_s_flat,
        node_lateral_offsets=node_l_flat,
        node_heading_offsets=node_h_flat,
        node_indices=node_indices,
        edges_from=np.asarray(edges_from, dtype=np.int32),
        edges_to=np.asarray(edges_to, dtype=np.int32),
        edges_cost=np.asarray(edges_cost, dtype=np.float32),
        index_map=index_map,
    )


def save_graph(graph: GraphData, data_dir: Path, prefix: str, meta: dict) -> None:
    data_dir.mkdir(parents=True, exist_ok=True)

    graph_path = data_dir / f'{prefix}.npz'
    index_path = data_dir / f'{prefix}_index.json'
    meta_path = data_dir / f'{prefix}_meta.json'

    np.savez_compressed(
        graph_path,
        node_positions=graph.node_positions,
        node_yaws=graph.node_yaws,
        node_s_values=graph.node_s_values,
        node_lateral_offsets=graph.node_lateral_offsets,
        node_heading_offsets=graph.node_heading_offsets,
        node_indices=graph.node_indices,
        edges_from=graph.edges_from,
        edges_to=graph.edges_to,
        edges_cost=graph.edges_cost,
    )

    with index_path.open('w', encoding='utf-8') as f:
        json.dump(graph.index_map, f, separators=(',', ':'), sort_keys=True)

    meta = dict(meta)
    meta.update(
        {
            'graph_path': graph_path.name,
            'index_path': index_path.name,
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

    parser = argparse.ArgumentParser(description='Precompute a local tube graph around checkpoints.')
    parser.add_argument(
        '--checkpoints',
        type=Path,
        default=data_dir / 'checkpoints.csv',
        help='Path to checkpoints CSV (default: %(default)s)',
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

    checkpoints = load_checkpoints(args.checkpoints)
    centerline, s_values, total_length = resample_centerline(checkpoints, args.s_step, args.closed_loop)
    headings = compute_headings(centerline, s_values, args.closed_loop)

    lateral_offsets = compute_lateral_offsets(args.tube_width, args.lateral_count)
    heading_offsets = compute_heading_offsets(args.heading_range_deg, args.heading_count)

    graph = build_graph(
        centerline=centerline,
        headings=headings,
        s_values=s_values,
        lateral_offsets=lateral_offsets,
        heading_offsets=heading_offsets,
        max_lateral_step=max(0, args.max_lateral_step),
        max_heading_step=max(0, args.max_heading_step),
        lateral_weight=max(0.0, args.lateral_weight),
        heading_weight=max(0.0, args.heading_weight),
        include_reverse=bool(args.include_reverse),
        closed_loop=bool(args.closed_loop),
    )

    meta = {
        'checkpoints': str(args.checkpoints),
        's_step': float(args.s_step),
        'tube_width': float(args.tube_width),
        'lateral_offsets': lateral_offsets.tolist(),
        'heading_offsets_rad': heading_offsets.tolist(),
        'max_lateral_step': int(args.max_lateral_step),
        'max_heading_step': int(args.max_heading_step),
        'lateral_weight': float(args.lateral_weight),
        'heading_weight': float(args.heading_weight),
        'include_reverse': bool(args.include_reverse),
        'closed_loop': bool(args.closed_loop),
        'centerline_length': float(total_length),
    }

    save_graph(graph, args.output_dir, args.prefix, meta)

    print(
        f'Graph saved to {args.output_dir} with prefix "{args.prefix}" '
        f'(nodes={graph.node_positions.shape[0]}, edges={graph.edges_from.shape[0]})'
    )


if __name__ == '__main__':
    main()
