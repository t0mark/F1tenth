#!/usr/bin/env python3
import json
import heapq
import math
from dataclasses import dataclass
from pathlib import Path as PathLib
from typing import Dict, List, Optional, Tuple

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped, Point
from nav_msgs.msg import Odometry, Path
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy, QoSHistoryPolicy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray
from scipy.spatial import cKDTree
from ament_index_python.packages import get_package_share_directory


def wrap_angle(angle: float) -> float:
    """Wrap angle to [-pi, pi)."""
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


@dataclass
class HybridState:
    x: float
    y: float
    yaw: float
    steering: float


class HybridAStarLocalPlanner(Node):
    def __init__(self) -> None:
        super().__init__('local_hybrid_astar_node')

        # Planner topics and frames
        self.declare_parameter('global_path_topic', '/global_path')
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('output_topic', '/local_path')
        self.declare_parameter('static_map_topic', '/map')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('base_frame', 'base_link')

        # Vehicle configuration
        self.declare_parameter('inflation_radius', 0.25)

        # Planner parameters
        self.declare_parameter('lookahead_distance', 5.0)
        self.declare_parameter('planner_hz', 50.0)
        self.declare_parameter('max_iterations', 1500)

        # Cost weights
        self.declare_parameter('steering_change_weight', 3.0)
        self.declare_parameter('obstacle_cost_weight', 25.0)
        self.declare_parameter('path_distance_weight', 3.0)
        self.declare_parameter('heuristic_weight', 1.0)
        self.declare_parameter('curvature_heuristic_weight', 15.0)
        self.declare_parameter('wall_distance_heuristic_weight', 10.0)
        self.declare_parameter('dynamic_obstacle_threshold', 0.7)

        # Graph parameters
        self.declare_parameter('local_graph_enabled', True)
        self.declare_parameter('local_graph_prefix', 'local_graph')
        self.declare_parameter('local_graph_dir', '')
        self.declare_parameter('publish_local_graph_markers', True)
        self.declare_parameter('local_graph_marker_topic', '/local_graph_markers')
        self.declare_parameter('local_graph_marker_scale', 0.06)
        self.declare_parameter('local_graph_edge_scale', 0.02)
        self.declare_parameter('use_local_graph_planner', True)
        self.declare_parameter('graph_match_max_distance', 1.0)
        self.declare_parameter('graph_yaw_weight', 1.5)
        self.declare_parameter('graph_goal_tolerance', 0.5)

        # Parameter retrieval
        self.global_path_topic = self.get_parameter('global_path_topic').get_parameter_value().string_value
        self.scan_topic = self.get_parameter('scan_topic').get_parameter_value().string_value
        self.odom_topic = self.get_parameter('odom_topic').get_parameter_value().string_value
        self.output_topic = self.get_parameter('output_topic').get_parameter_value().string_value
        self.map_frame = self.get_parameter('map_frame').get_parameter_value().string_value
        self.base_frame = self.get_parameter('base_frame').get_parameter_value().string_value

        self.inflation_radius = float(self.get_parameter('inflation_radius').value)
        self.lookahead_distance = float(self.get_parameter('lookahead_distance').value)
        self.planner_hz = float(self.get_parameter('planner_hz').value)
        self.max_iterations = int(self.get_parameter('max_iterations').value)

        self.steering_change_weight = float(self.get_parameter('steering_change_weight').value)
        self.obstacle_cost_weight = float(self.get_parameter('obstacle_cost_weight').value)
        self.path_distance_weight = float(self.get_parameter('path_distance_weight').value)
        self.heuristic_weight = float(self.get_parameter('heuristic_weight').value)
        self.curvature_heuristic_weight = float(self.get_parameter('curvature_heuristic_weight').value)
        self.wall_distance_heuristic_weight = float(self.get_parameter('wall_distance_heuristic_weight').value)
        self.dynamic_obstacle_threshold = float(self.get_parameter('dynamic_obstacle_threshold').value)

        self.local_graph_enabled = bool(self.get_parameter('local_graph_enabled').value)
        self.local_graph_prefix = self.get_parameter('local_graph_prefix').get_parameter_value().string_value
        self.local_graph_dir_param = self.get_parameter('local_graph_dir').get_parameter_value().string_value.strip()
        self.publish_graph_markers = bool(self.get_parameter('publish_local_graph_markers').value)
        self.local_graph_marker_topic = self.get_parameter('local_graph_marker_topic').get_parameter_value().string_value
        self.local_graph_marker_scale = max(1e-3, float(self.get_parameter('local_graph_marker_scale').value))
        self.local_graph_edge_scale = max(1e-3, float(self.get_parameter('local_graph_edge_scale').value))
        self.use_local_graph_planner = bool(self.get_parameter('use_local_graph_planner').value)
        self.graph_match_max_distance = max(0.05, float(self.get_parameter('graph_match_max_distance').value))
        self.graph_yaw_weight = max(1e-3, float(self.get_parameter('graph_yaw_weight').value))
        self.graph_goal_tolerance = max(0.05, float(self.get_parameter('graph_goal_tolerance').value))

        # Runtime state
        self.global_path_np: Optional[np.ndarray] = None  # shape (N, 3) for x, y, yaw
        self.global_path_tree: Optional[cKDTree] = None  # KDTree for fast path deviation query
        self.latest_scan: Optional[LaserScan] = None
        self.latest_pose: Optional[Tuple[float, float, float]] = None  # x, y, yaw
        self.last_path_msg: Optional[Path] = None

        self._last_warnings: Dict[str, float] = {}
        self.local_graph_positions: Optional[np.ndarray] = None
        self.local_graph_yaws: Optional[np.ndarray] = None
        self.local_graph_lateral_offsets: Optional[np.ndarray] = None
        self.local_graph_curvatures: Optional[np.ndarray] = None
        self.local_graph_wall_distances: Optional[np.ndarray] = None
        self.local_graph_dynamic_costs: Optional[np.ndarray] = None
        self.local_graph_tree = None
        self.local_graph_edges_from: Optional[np.ndarray] = None
        self.local_graph_edges_to: Optional[np.ndarray] = None
        self.local_graph_edges_cost: Optional[np.ndarray] = None
        self.local_graph_neighbors: List[np.ndarray] = []
        self.local_graph_neighbor_costs: List[np.ndarray] = []
        self.local_graph_indices: Optional[np.ndarray] = None  # (s_idx, lat_idx, head_idx) per node
        self.local_graph_meta: Dict[str, object] = {}
        self.graph_marker_pub = None

        # ROS interfaces
        self.global_path_sub = self.create_subscription(Path, self.global_path_topic, self._on_global_path, 10)
        self.scan_sub = self.create_subscription(LaserScan, self.scan_topic, self._on_scan, 10)
        self.odom_sub = self.create_subscription(Odometry, self.odom_topic, self._on_odom, 10)

        self.path_pub = self.create_publisher(Path, self.output_topic, 10)
        self._load_local_graph()
        if self.publish_graph_markers:
            qos = QoSProfile(history=QoSHistoryPolicy.KEEP_LAST, depth=1)
            qos.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL
            qos.reliability = QoSReliabilityPolicy.RELIABLE
            self.graph_marker_pub = self.create_publisher(MarkerArray, self.local_graph_marker_topic, qos)
            # 정적 그래프 마커는 한 번만 퍼블리시
            self._publish_static_graph_markers()

        timer_period = 1.0 / max(0.1, self.planner_hz)
        self.timer = self.create_timer(timer_period, self._on_timer)

        self.get_logger().info(
            'Hybrid A* local planner 초기화 완료 '
            f'(inflation_radius: {self.inflation_radius:.2f}m, '
            f'obstacle_cost_weight: {self.obstacle_cost_weight:.1f})'
        )

    # --------------------------------------------------------------------- #
    # ROS callbacks
    # --------------------------------------------------------------------- #
    def _on_global_path(self, msg: Path) -> None:
        if not msg.poses:
            self.global_path_np = None
            self.global_path_tree = None
            return

        points = []
        poses = msg.poses
        for i, pose in enumerate(poses):
            x = pose.pose.position.x
            y = pose.pose.position.y
            if i < len(poses) - 1:
                next_pose = poses[i + 1]
                dx = next_pose.pose.position.x - x
                dy = next_pose.pose.position.y - y
            elif i > 0:
                prev_pose = poses[i - 1]
                dx = x - prev_pose.pose.position.x
                dy = y - prev_pose.pose.position.y
            else:
                dx = 1.0
                dy = 0.0
            yaw = math.atan2(dy, dx)
            points.append((x, y, yaw))

        self.global_path_np = np.array(points, dtype=float)

        # KDTree 빌드 (경로 편차 계산 최적화용)
        if len(self.global_path_np) > 0:
            self.global_path_tree = cKDTree(self.global_path_np[:, :2])
        else:
            self.global_path_tree = None

    def _on_scan(self, msg: LaserScan) -> None:
        self.latest_scan = msg

    def _on_odom(self, msg: Odometry) -> None:
        pos = msg.pose.pose.position
        ori = msg.pose.pose.orientation
        siny_cosp = 2.0 * (ori.w * ori.z + ori.x * ori.y)
        cosy_cosp = 1.0 - 2.0 * (ori.y * ori.y + ori.z * ori.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        self.latest_pose = (pos.x, pos.y, yaw)

    # --------------------------------------------------------------------- #
    # Planner loop
    # --------------------------------------------------------------------- #
    def _on_timer(self) -> None:
        if not self._data_ready():
            return

        if not self._update_dynamic_costs():
            self._throttled_warn('dynamic_cost', '동적 장애물 코스트를 업데이트할 수 없습니다.')
            return

        goal = self._select_goal()
        if goal is None:
            self._throttled_warn('goal', '글로벌 경로 목표를 찾을 수 없습니다.')
            return

        path_states = self._plan_hybrid_astar(goal)
        if not path_states:
            self._throttled_warn('planner', 'Hybrid A* 탐색 실패 - 마지막 경로를 재사용합니다.')
            if self.last_path_msg is not None:
                self.path_pub.publish(self.last_path_msg)
            return

        path_msg = self._states_to_path(path_states)
        self.last_path_msg = path_msg
        self.path_pub.publish(path_msg)

    def _data_ready(self) -> bool:
        ready = True
        now_sec = self.get_clock().now().nanoseconds * 1e-9
        if self.latest_pose is None:
            ready = False
            self._throttled_warn('odom', '/odom 데이터를 기다리는 중입니다.', now_sec)
        if self.latest_scan is None:
            ready = False
            self._throttled_warn('scan', '/scan 데이터를 기다리는 중입니다.', now_sec)
        if self.global_path_np is None or len(self.global_path_np) == 0:
            ready = False
            self._throttled_warn('global_path', '글로벌 경로가 아직 준비되지 않았습니다.', now_sec)
        return ready

    def _throttled_warn(self, key: str, message: str, now_sec: Optional[float] = None, period: float = 1.0) -> None:
        if now_sec is None:
            now_sec = self.get_clock().now().nanoseconds * 1e-9
        last = self._last_warnings.get(key, 0.0)
        if now_sec - last >= period:
            self.get_logger().warn(message)
            self._last_warnings[key] = now_sec

    # --------------------------------------------------------------------- #
    # Local graph utilities
    # --------------------------------------------------------------------- #
    def _load_local_graph(self) -> None:
        if not self.local_graph_enabled:
            return

        self.local_graph_positions = None
        self.local_graph_yaws = None
        self.local_graph_lateral_offsets = None
        self.local_graph_curvatures = None
        self.local_graph_wall_distances = None
        self.local_graph_edges_from = None
        self.local_graph_edges_to = None
        self.local_graph_edges_cost = None
        self.local_graph_neighbors = []
        self.local_graph_neighbor_costs = []
        self.local_graph_indices = None
        self.local_graph_meta = {}

        try:
            pkg_share_dir = PathLib(get_package_share_directory('path_planner'))
        except Exception:
            pkg_share_dir = PathLib(__file__).resolve().parent.parent
        default_data_dir = (pkg_share_dir / 'data').resolve()

        if self.local_graph_dir_param:
            base_dir = PathLib(self.local_graph_dir_param).expanduser()
            if not base_dir.is_absolute():
                base_dir = (pkg_share_dir / base_dir).resolve()
        else:
            base_dir = default_data_dir
        base_dir = base_dir.resolve()
        prefix = self.local_graph_prefix
        npz_path = base_dir / f'{prefix}.npz'
        meta_path = base_dir / f'{prefix}_meta.json'

        if not npz_path.exists():
            self.get_logger().warn(f'로컬 그래프 파일을 찾을 수 없습니다: {npz_path}')
            return

        try:
            npz_file = np.load(npz_path)
        except Exception as exc:
            self.get_logger().error(f'로컬 그래프 데이터를 불러오는 중 오류 발생: {exc}')
            return

        try:
            required = [
                'node_positions',
                'node_yaws',
                'node_lateral_offsets',
                'edges_from',
                'edges_to',
                'edges_cost',
            ]
            missing = [key for key in required if key not in npz_file]
            if missing:
                self.get_logger().error(f'로컬 그래프 데이터에 누락된 키: {missing}')
                return

            self.local_graph_positions = np.array(npz_file['node_positions'], dtype=np.float32)
            self.local_graph_yaws = np.array(npz_file['node_yaws'], dtype=np.float32)
            node_count = int(self.local_graph_positions.shape[0])
            self.local_graph_lateral_offsets = np.array(npz_file['node_lateral_offsets'], dtype=np.float32)
            self.local_graph_edges_from = np.array(npz_file['edges_from'], dtype=np.int32)
            self.local_graph_edges_to = np.array(npz_file['edges_to'], dtype=np.int32)
            self.local_graph_edges_cost = np.array(npz_file['edges_cost'], dtype=np.float32)

            if 'node_curvatures' in npz_file:
                self.local_graph_curvatures = np.array(npz_file['node_curvatures'], dtype=np.float32)
            else:
                self.local_graph_curvatures = np.zeros(node_count, dtype=np.float32)

            if 'node_wall_distances' in npz_file:
                self.local_graph_wall_distances = np.array(npz_file['node_wall_distances'], dtype=np.float32)
            else:
                self.local_graph_wall_distances = np.zeros(node_count, dtype=np.float32)

            if 'node_indices' in npz_file:
                self.local_graph_indices = np.array(npz_file['node_indices'], dtype=np.int32)
            else:
                self.local_graph_indices = None
        finally:
            npz_file.close()

        node_count = int(self.local_graph_positions.shape[0])
        neighbor_lists: List[List[int]] = [[] for _ in range(node_count)]
        neighbor_cost_lists: List[List[float]] = [[] for _ in range(node_count)]
        if self.local_graph_edges_from is not None and self.local_graph_edges_to is not None and self.local_graph_edges_cost is not None:
            for src, dst, cost in zip(
                self.local_graph_edges_from.tolist(),
                self.local_graph_edges_to.tolist(),
                self.local_graph_edges_cost.tolist()
            ):
                neighbor_lists[src].append(int(dst))
                neighbor_cost_lists[src].append(float(cost))
        self.local_graph_neighbors = [
            np.array(lst, dtype=np.int32) if lst else np.empty(0, dtype=np.int32)
            for lst in neighbor_lists
        ]
        self.local_graph_neighbor_costs = [
            np.array(lst, dtype=np.float32) if lst else np.empty(0, dtype=np.float32)
            for lst in neighbor_cost_lists
        ]
        if node_count > 0:
            self.local_graph_tree = cKDTree(self.local_graph_positions)
            self.local_graph_dynamic_costs = np.zeros(node_count, dtype=np.float32)
        else:
            self.local_graph_tree = None
            self.local_graph_dynamic_costs = None

        if meta_path.exists():
            try:
                with meta_path.open('r', encoding='utf-8') as f:
                    meta = json.load(f)
                if isinstance(meta, dict):
                    self.local_graph_meta = meta
                    self.local_graph_closed_loop = bool(meta.get('closed_loop', False))
            except Exception as exc:
                self.get_logger().warn(f'로컬 그래프 메타데이터를 읽지 못했습니다: {exc}')
                self.local_graph_meta = {}
                self.local_graph_closed_loop = False
        else:
            self.local_graph_meta = {}
            self.local_graph_closed_loop = False

        node_count = int(self.local_graph_positions.shape[0])
        edge_count = int(self.local_graph_edges_from.shape[0])
        self.get_logger().info(
            f'로컬 그래프 로드 완료 (노드 {node_count}, 엣지 {edge_count}) - {npz_path}'
        )

    def _publish_static_graph_markers(self) -> None:
        """정적 그래프 마커를 한 번만 생성하여 퍼블리시 (TRANSIENT_LOCAL QoS로 지속)"""
        if (
            self.graph_marker_pub is None
            or self.local_graph_positions is None
            or self.local_graph_edges_from is None
            or self.local_graph_edges_to is None
        ):
            return

        marker_array = MarkerArray()
        stamp = self.get_clock().now().to_msg()

        # 노드 마커 (단순 파란색)
        node_marker = Marker()
        node_marker.header.frame_id = self.map_frame
        node_marker.header.stamp = stamp
        node_marker.ns = 'local_graph_nodes'
        node_marker.id = 0
        node_marker.type = Marker.POINTS
        node_marker.action = Marker.ADD
        node_marker.pose.orientation.w = 1.0
        node_marker.scale.x = self.local_graph_marker_scale
        node_marker.scale.y = self.local_graph_marker_scale
        node_marker.color = ColorRGBA(r=0.2, g=0.5, b=0.9, a=0.6)

        # 모든 노드 위치만 추가 (색상 계산 없음)
        node_marker.points = [
            Point(x=float(pos[0]), y=float(pos[1]), z=0.0)
            for pos in self.local_graph_positions
        ]
        marker_array.markers.append(node_marker)

        # 엣지 마커 (단순 회색)
        edge_marker = Marker()
        edge_marker.header.frame_id = self.map_frame
        edge_marker.header.stamp = stamp
        edge_marker.ns = 'local_graph_edges'
        edge_marker.id = 1
        edge_marker.type = Marker.LINE_LIST
        edge_marker.action = Marker.ADD
        edge_marker.pose.orientation.w = 1.0
        edge_marker.scale.x = self.local_graph_edge_scale
        edge_marker.color = ColorRGBA(r=0.3, g=0.3, b=0.3, a=0.4)

        # 모든 엣지 추가 (색상 계산 없음)
        positions = self.local_graph_positions
        for src_idx, dst_idx in zip(self.local_graph_edges_from, self.local_graph_edges_to):
            src = positions[src_idx]
            dst = positions[dst_idx]
            edge_marker.points.append(Point(x=float(src[0]), y=float(src[1]), z=0.0))
            edge_marker.points.append(Point(x=float(dst[0]), y=float(dst[1]), z=0.0))
        marker_array.markers.append(edge_marker)

        self.graph_marker_pub.publish(marker_array)
        self.get_logger().info(
            f'정적 그래프 마커 퍼블리시 완료 '
            f'(노드 {self.local_graph_positions.shape[0]}개, '
            f'엣지 {self.local_graph_edges_from.shape[0]}개)'
        )

    # --------------------------------------------------------------------- #
    # Graph-based planning helpers
    # --------------------------------------------------------------------- #
    def _graph_match_pose(self, x: float, y: float, yaw: float, max_distance: Optional[float] = None) -> Optional[int]:
        if self.local_graph_positions is None or self.local_graph_yaws is None:
            return None

        if max_distance is None:
            max_distance = self.graph_match_max_distance

        pos = np.array([x, y], dtype=np.float32)
        diffs = self.local_graph_positions - pos
        dist_sq = np.sum(diffs * diffs, axis=1)
        yaw_diff = np.abs(wrap_angle(self.local_graph_yaws - yaw))
        score = dist_sq + (self.graph_yaw_weight ** 2) * (yaw_diff ** 2)

        if score.size == 0:
            return None

        best_idx = int(np.argmin(score))
        best_dist = math.sqrt(float(dist_sq[best_idx]))
        if best_dist > max_distance:
            if best_dist <= max_distance * 2.0:
                self.get_logger().warn(
                    f'그래프 노드 매칭: 거리 {best_dist:.2f}m로 허용 범위({max_distance:.2f}m)를 초과하지만 '
                    '가장 가까운 노드를 임시 사용합니다.'
                )
                return best_idx
            return None
        return best_idx


    def _graph_heuristic(self, node_id: int, goal_pos: np.ndarray, goal_yaw: float) -> float:
        """
        새로운 휴리스틱:
        - 목표까지의 거리
        - 벽까지의 유클리드 거리 (가까울수록 페널티)
        - 곡률 기반 속도 지향 (바깥쪽 낮은 휴리스틱, 안쪽 높은 휴리스틱)
        """
        if self.local_graph_positions is None or self.local_graph_yaws is None:
            return 0.0

        node_pos = self.local_graph_positions[node_id]

        # 목표까지의 거리
        distance = math.hypot(float(node_pos[0] - goal_pos[0]), float(node_pos[1] - goal_pos[1]))

        # 벽까지의 거리 (가까울수록 페널티)
        wall_distance = 0.0
        if self.local_graph_wall_distances is not None:
            wall_dist = float(self.local_graph_wall_distances[node_id])
            # 벽에 가까울수록 큰 페널티 (역수 관계)
            if wall_dist > 1e-3:
                wall_distance = 1.0 / wall_dist
            else:
                wall_distance = 1000.0

        # 곡률 기반 휴리스틱: 바깥쪽 선호, 안쪽 페널티
        curvature_heuristic = 0.0
        if self.local_graph_curvatures is not None and self.local_graph_lateral_offsets is not None:
            curvature = float(self.local_graph_curvatures[node_id])
            lateral_offset = float(self.local_graph_lateral_offsets[node_id])
            # curvature > 0: 좌회전 (왼쪽이 안쪽, 오른쪽이 바깥쪽)
            # lateral_offset > 0: 오른쪽
            # 안쪽에 높은 페널티 (curvature * lateral_offset < 0이면 안쪽)
            if curvature * lateral_offset < 0:
                # 안쪽 - 높은 페널티
                curvature_heuristic = abs(curvature * lateral_offset) * 3.0
            else:
                # 바깥쪽 - 낮은 페널티 (보너스)
                curvature_heuristic = -abs(curvature * lateral_offset) * 0.5

        return (
            self.heuristic_weight * distance
            + self.wall_distance_heuristic_weight * wall_distance
            + self.curvature_heuristic_weight * curvature_heuristic
        )

    def _graph_reconstruct_path(self, goal_node: int, came_from: Dict[int, Optional[int]]) -> List[int]:
        path: List[int] = []
        current: Optional[int] = goal_node
        while current is not None:
            path.append(current)
            current = came_from.get(current)
        path.reverse()
        return path

    def _graph_nodes_to_states(self, node_ids: List[int]) -> List[HybridState]:
        if self.local_graph_positions is None or self.local_graph_yaws is None:
            return []

        states: List[HybridState] = []
        for node_id in node_ids:
            pos = self.local_graph_positions[node_id]
            yaw = float(self.local_graph_yaws[node_id])
            states.append(HybridState(x=float(pos[0]), y=float(pos[1]), yaw=yaw, steering=0.0))
        return states

    def _plan_graph_astar(self, goal: Dict[str, float]) -> Optional[List[HybridState]]:
        if (
            self.local_graph_positions is None
            or self.local_graph_neighbors is None
            or len(self.local_graph_neighbors) == 0
        ):
            return None
        if self.latest_pose is None:
            return None

        robot_x, robot_y, robot_yaw = self.latest_pose
        start_id = self._graph_match_pose(robot_x, robot_y, robot_yaw)
        if start_id is None:
            self.get_logger().warn('그래프 기반 플래너: 시작 노드를 찾지 못했습니다.')
            return None

        goal_pos = np.array([goal['x'], goal['y']], dtype=np.float32)
        goal_yaw = goal['yaw']

        open_list: List[Tuple[float, float, int]] = []
        g_scores: Dict[int, float] = {start_id: 0.0}
        came_from: Dict[int, Optional[int]] = {start_id: None}

        start_h = self._graph_heuristic(start_id, goal_pos, goal_yaw)
        heapq.heappush(open_list, (start_h, 0.0, start_id))

        expansions = 0
        edges_checked = 0

        while open_list and expansions < self.max_iterations:
            _, current_g, current_id = heapq.heappop(open_list)
            if current_g > g_scores.get(current_id, float('inf')) + 1e-6:
                continue

            node_pos = self.local_graph_positions[current_id]
            if math.hypot(float(node_pos[0] - goal_pos[0]), float(node_pos[1] - goal_pos[1])) <= self.graph_goal_tolerance:
                node_path = self._graph_reconstruct_path(current_id, came_from)
                states = self._graph_nodes_to_states(node_path)
                self.get_logger().debug(
                    f'그래프 기반 플래너 성공: 확장 {expansions}회, 검사 엣지 {edges_checked}개, 경로 길이 {len(states)}'
                )
                return states

            neighbor_ids = self.local_graph_neighbors[current_id]
            neighbor_costs = self.local_graph_neighbor_costs[current_id]
            expansions += 1

            for neigh_id, base_cost in zip(neighbor_ids, neighbor_costs):
                edges_checked += 1

                # 목적지 노드의 동적 장애물 비용 체크
                if self.local_graph_dynamic_costs is not None:
                    node_obstacle_cost = float(self.local_graph_dynamic_costs[int(neigh_id)])
                    if node_obstacle_cost >= self.dynamic_obstacle_threshold:
                        continue  # 목적지 노드가 장애물에 막힘
                else:
                    node_obstacle_cost = 0.0

                # 조향 변화 페널티
                yaw_penalty = self.steering_change_weight * abs(
                    wrap_angle(float(self.local_graph_yaws[int(neigh_id)]) - float(self.local_graph_yaws[current_id]))
                )

                # 글로벌 경로 이탈 페널티
                neighbour_pos = self.local_graph_positions[int(neigh_id)]
                path_penalty = self.path_distance_weight * self._get_path_deviation(
                    float(neighbour_pos[0]),
                    float(neighbour_pos[1])
                )

                # 동적 장애물 페널티
                obstacle_penalty = self.obstacle_cost_weight * node_obstacle_cost

                tentative_g = current_g + float(base_cost) + yaw_penalty + path_penalty + obstacle_penalty

                if tentative_g >= g_scores.get(int(neigh_id), float('inf')) - 1e-6:
                    continue

                g_scores[int(neigh_id)] = tentative_g
                came_from[int(neigh_id)] = current_id
                heuristic = self._graph_heuristic(int(neigh_id), goal_pos, goal_yaw)
                heapq.heappush(open_list, (tentative_g + heuristic, tentative_g, int(neigh_id)))

        self.get_logger().warn(
            f'그래프 기반 플래너 실패: 확장 {expansions}회, 검사 엣지 {edges_checked}개'
        )
        return None

    # --------------------------------------------------------------------- #
    # Dynamic cost utilities (Diamond propagation)
    # --------------------------------------------------------------------- #
    def _update_dynamic_costs(self) -> bool:
        """
        LiDAR 포인트를 그래프에 투영하고 마름모 모양으로 코스트 전파
        """
        if (
            self.latest_pose is None
            or self.latest_scan is None
            or self.local_graph_tree is None
            or self.local_graph_dynamic_costs is None
            or self.local_graph_indices is None
        ):
            return False

        self.local_graph_dynamic_costs.fill(0.0)
        robot_x, robot_y, robot_yaw = self.latest_pose
        scan = self.latest_scan

        cos_yaw = math.cos(robot_yaw)
        sin_yaw = math.sin(robot_yaw)
        max_range = scan.range_max

        # LiDAR 빔 좌표 변환
        obstacle_points = []
        angle = scan.angle_min
        for rng in scan.ranges:
            if not math.isfinite(rng) or rng < scan.range_min or rng > max_range:
                angle += scan.angle_increment
                continue
            lx = rng * math.cos(angle)
            ly = rng * math.sin(angle)
            obs_x = robot_x + lx * cos_yaw - ly * sin_yaw
            obs_y = robot_y + lx * sin_yaw + ly * cos_yaw
            obstacle_points.append([obs_x, obs_y])
            angle += scan.angle_increment

        if not obstacle_points:
            return True

        # 각 LiDAR 포인트를 그래프에 투영
        obstacle_array = np.array(obstacle_points, dtype=np.float32)
        distances, indices = self.local_graph_tree.query(obstacle_array, k=1)

        # 투영된 노드들로부터 마름모 전파
        projected_nodes = set(int(idx) for idx in indices if np.isfinite(distances[np.where(indices == idx)[0][0]]))

        # 마름모 전파 (맨해튼 거리 기반)
        self._propagate_diamond_cost(projected_nodes)

        return True

    def _propagate_diamond_cost(self, seed_nodes: set) -> None:
        """
        시드 노드들로부터 마름모/원형으로 코스트 전파 (월드 좌표 기반)
        inflation_radius 거리 이내의 모든 노드에 전파
        """
        if (
            self.local_graph_positions is None
            or self.local_graph_dynamic_costs is None
            or self.local_graph_tree is None
        ):
            return

        # 각 시드 노드 주변의 노드들을 KDTree로 빠르게 검색
        for seed_node in seed_nodes:
            if seed_node >= len(self.local_graph_positions):
                continue

            seed_pos = self.local_graph_positions[seed_node]

            # KDTree로 inflation_radius 이내의 모든 노드 검색
            # 마름모 대신 원형 영역 사용 (더 자연스러운 장애물 회피)
            indices = self.local_graph_tree.query_ball_point(
                seed_pos,
                r=self.inflation_radius,
                return_sorted=False
            )

            # 거리 기반 코스트 감쇠
            for node_id in indices:
                node_pos = self.local_graph_positions[node_id]
                dist = np.linalg.norm(seed_pos - node_pos)

                # 거리에 따라 선형 감쇠
                if dist < self.inflation_radius:
                    cost = 1.0 - (dist / self.inflation_radius)
                    self.local_graph_dynamic_costs[node_id] = max(
                        self.local_graph_dynamic_costs[node_id],
                        cost
                    )

        return


    def _get_path_deviation(self, x: float, y: float) -> float:
        """글로벌 경로로부터의 거리 계산 (KDTree 기반 O(log N) 쿼리)"""
        if self.global_path_tree is None:
            return 0.0

        # KDTree로 최근접 경로 포인트까지의 거리 쿼리
        dist, _ = self.global_path_tree.query([x, y])
        return float(dist)

    # --------------------------------------------------------------------- #
    # Goal selection and heuristics
    # --------------------------------------------------------------------- #
    def _select_goal(self) -> Optional[Dict[str, float]]:
        if self.global_path_np is None or len(self.global_path_np) == 0 or self.latest_pose is None:
            return None

        robot_x, robot_y, _ = self.latest_pose
        waypoints_xy = self.global_path_np[:, :2]
        dists = np.linalg.norm(waypoints_xy - np.array([[robot_x, robot_y]]), axis=1)
        closest_idx = int(np.argmin(dists))

        cumulative = 0.0
        goal_idx = closest_idx
        for i in range(closest_idx, len(self.global_path_np) - 1):
            seg = np.linalg.norm(self.global_path_np[i + 1, :2] - self.global_path_np[i, :2])
            cumulative += seg
            goal_idx = i + 1
            if cumulative >= self.lookahead_distance:
                break

        goal_point = self.global_path_np[goal_idx]
        return {'x': float(goal_point[0]), 'y': float(goal_point[1]), 'yaw': float(goal_point[2])}

    # --------------------------------------------------------------------- #
    # Graph-based search wrapper
    # --------------------------------------------------------------------- #
    def _plan_hybrid_astar(self, goal: Dict[str, float]) -> Optional[List[HybridState]]:
        if not self.use_local_graph_planner:
            self.get_logger().warn('로컬 그래프 플래너가 비활성화되어 경로를 생성할 수 없습니다.')
            return None

        graph_path = self._plan_graph_astar(goal)
        if graph_path:
            return graph_path

        self.get_logger().warn('그래프 기반 플래너가 경로를 찾지 못했습니다.')
        return None

    def _states_to_path(self, states: List[HybridState]) -> Path:
        """탐색 결과 상태들을 조밀한 경로로 변환 (보간 포함)"""

        path_msg = Path()
        path_msg.header.frame_id = self.map_frame
        stamp = self.get_clock().now().to_msg()
        path_msg.header.stamp = stamp

        if not states:
            return path_msg

        # 1. 로봇 현재 위치를 첫 번째 포인트로 추가 (보간 없이)
        if self.latest_pose is not None:
            robot_pose = PoseStamped()
            robot_pose.header.frame_id = self.map_frame
            robot_pose.header.stamp = stamp
            robot_pose.pose.position.x = self.latest_pose[0]
            robot_pose.pose.position.y = self.latest_pose[1]
            robot_pose.pose.position.z = 0.0
            half_yaw = self.latest_pose[2] * 0.5
            robot_pose.pose.orientation.z = math.sin(half_yaw)
            robot_pose.pose.orientation.w = math.cos(half_yaw)
            path_msg.poses.append(robot_pose)

        # 2. 탐색으로 얻은 경로를 Pose 시퀀스로 변환
        raw_poses = []
        for state in states:
            pose = PoseStamped()
            pose.header.frame_id = self.map_frame
            pose.pose.position.x = state.x
            pose.pose.position.y = state.y
            pose.pose.position.z = 0.0
            half_yaw = state.yaw * 0.5
            pose.pose.orientation.z = math.sin(half_yaw)
            pose.pose.orientation.w = math.cos(half_yaw)
            raw_poses.append(pose)

        # 3. 경로 보간 (간격을 0.15m로 조밀화)
        interpolated_poses = self._interpolate_path(raw_poses, interval=0.15)

        # 4. 보간된 경로를 메시지에 추가
        for pose in interpolated_poses:
            pose.header.stamp = stamp
            path_msg.poses.append(pose)

        return path_msg

    def _interpolate_path(self, poses: List[PoseStamped], interval: float) -> List[PoseStamped]:
        """경로를 일정 간격으로 보간"""
        if len(poses) < 2:
            return poses

        interpolated = [poses[0]]  # 시작점 추가

        for i in range(len(poses) - 1):
            p1 = poses[i].pose.position
            p2 = poses[i + 1].pose.position

            dx = p2.x - p1.x
            dy = p2.y - p1.y
            segment_length = math.hypot(dx, dy)

            if segment_length < 1e-6:
                continue

            # 세그먼트를 interval 간격으로 분할
            num_points = int(math.ceil(segment_length / interval))

            for j in range(1, num_points):
                t = j / num_points

                new_pose = PoseStamped()
                new_pose.header.frame_id = poses[i].header.frame_id
                new_pose.pose.position.x = p1.x + dx * t
                new_pose.pose.position.y = p1.y + dy * t
                new_pose.pose.position.z = 0.0

                # 방향은 세그먼트 방향 사용
                yaw = math.atan2(dy, dx)
                new_pose.pose.orientation.z = math.sin(yaw / 2.0)
                new_pose.pose.orientation.w = math.cos(yaw / 2.0)

                interpolated.append(new_pose)

            interpolated.append(poses[i + 1])  # 끝점 추가

        return interpolated


def main(args=None) -> None:
    rclpy.init(args=args)
    node = HybridAStarLocalPlanner()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Hybrid A* 로컬 플래너 종료')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
