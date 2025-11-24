#!/usr/bin/env python3
"""A* 기반 로컬 플래너 구현.

- 상태: 그래프 노드 인덱스
- g(n): 엣지를 통과할 수 있으면 동일(고정 또는 거리), 충돌이면 제거
- h(n): 글로벌 패스(lane_idx=0) 이탈 정도와 인코스 보너스만 사용
"""

import heapq
import math
import os
from typing import Dict, List, Optional, Set, Tuple

import numpy as np
import rclpy
from geometry_msgs.msg import Point, PoseStamped
from nav_msgs.msg import Odometry, Path
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray

try:
    from scipy.spatial import cKDTree  # type: ignore
except ImportError:  # pragma: no cover
    cKDTree = None  # type: ignore


class GraphPlanner:
    """그래프에서 A* 탐색을 수행하는 클래스."""

    def __init__(
        self,
        graph_file_path: str,
        *,
        collision_check_resolution: float,
        edge_cost_mode: str,
        heuristic_weight_center: float,
        heuristic_weight_incourse: float,
    ) -> None:
        if not os.path.exists(graph_file_path):
            raise FileNotFoundError(f'그래프 파일을 찾을 수 없습니다: {graph_file_path}')

        data = np.load(graph_file_path)
        if 'nodes' not in data or 'edges' not in data:
            raise ValueError('그래프 파일에는 nodes / edges 키가 포함되어야 합니다')

        self.nodes: np.ndarray = np.asarray(data['nodes'], dtype=np.float64)
        self.edges: np.ndarray = np.asarray(data['edges'], dtype=np.int64)
        if self.nodes.ndim != 2 or self.nodes.shape[1] < 5:
            raise ValueError('nodes 배열은 (N, 5) 이상의 형태여야 합니다')
        if self.edges.ndim != 2 or self.edges.shape[1] != 2:
            raise ValueError('edges 배열은 (M, 2) 형태여야 합니다')

        self.positions: np.ndarray = self.nodes[:, :2]
        self.node_theta: np.ndarray = self.nodes[:, 2]
        self.node_lane: np.ndarray = self.nodes[:, 3].astype(np.int64)
        self.node_long: np.ndarray = self.nodes[:, 4].astype(np.int64)

        # 속도 정보 (있으면 사용, 없으면 기본값)
        if self.nodes.shape[1] >= 7:
            self.node_max_velocity: np.ndarray = self.nodes[:, 5]
            self.node_curvature: np.ndarray = self.nodes[:, 6]
            self.has_velocity_info = True
        else:
            self.node_max_velocity = np.ones(len(self.nodes)) * 5.0  # 기본 속도
            self.node_curvature = np.zeros(len(self.nodes))
            self.has_velocity_info = False

        self.adjacency: Dict[int, List[int]] = self._build_adjacency()

        # 그래프 해시 정보 로드
        self.graph_hash: Optional[str] = None
        if 'graph_hash' in data:
            hash_data = data['graph_hash']
            if hash_data.size > 0:
                # numpy 문자열 배열에서 첫 번째 요소 추출
                self.graph_hash = str(hash_data.flatten()[0].decode('utf-8') if isinstance(hash_data.flatten()[0], bytes) else hash_data.flatten()[0])

        self.metadata = data.get('metadata')
        self.longitudinal_interval: float = 1.0
        self.lateral_interval: float = 1.0
        if self.metadata is not None:
            if len(self.metadata) >= 1:
                self.longitudinal_interval = float(self.metadata[0])
            if len(self.metadata) >= 2:
                self.lateral_interval = float(self.metadata[1])

        self.collision_check_resolution = max(0.05, float(collision_check_resolution))
        mode = str(edge_cost_mode).lower()
        self.edge_cost_mode = 'length' if mode not in ('unit', 'length', 'time') else mode
        self.weight_center = float(heuristic_weight_center)
        self.weight_incourse = float(heuristic_weight_incourse)

        # 속도 기반 비용 사용 여부
        self.use_velocity_cost = (self.edge_cost_mode == 'time' and self.has_velocity_info)

        self.center_lane_sequence: List[int] = self._compute_center_lane_sequence()
        self.center_index_by_long: Dict[int, int] = self._build_center_index_lookup()
        self.curvature_by_long: Dict[int, float] = self._compute_curvature_lookup()
        self.node_d_center: np.ndarray = np.abs(self.node_lane) * self.lateral_interval
        self.node_incourse: np.ndarray = self._compute_incourse_scores()

        self.dynamic_obstacles: List[Tuple[float, float, float]] = []
        self.kdtree: Optional[cKDTree] = None  # type: ignore[valid-type]
        if cKDTree is not None:
            self.kdtree = cKDTree(self.positions)  # type: ignore[assignment]

    def _build_adjacency(self) -> Dict[int, List[int]]:
        adjacency: Dict[int, List[int]] = {}
        for frm, to in self.edges:
            adjacency.setdefault(int(frm), []).append(int(to))
        return adjacency

    def _compute_center_lane_sequence(self) -> List[int]:
        center_indices = np.where(self.node_lane == 0)[0]
        if len(center_indices) == 0:
            return []
        order = np.argsort(self.node_long[center_indices])
        return [int(center_indices[idx]) for idx in order]

    def _build_center_index_lookup(self) -> Dict[int, int]:
        lookup: Dict[int, int] = {}
        for order, node_idx in enumerate(self.center_lane_sequence):
            lookup[int(self.node_long[node_idx])] = order
        return lookup

    def _compute_curvature_lookup(self) -> Dict[int, float]:
        lookup: Dict[int, float] = {}
        if len(self.center_lane_sequence) < 3:
            return lookup
        for i, node_idx in enumerate(self.center_lane_sequence):
            prev_idx = self.center_lane_sequence[i - 1] if i > 0 else None
            next_idx = self.center_lane_sequence[i + 1] if i + 1 < len(self.center_lane_sequence) else None
            if prev_idx is None or next_idx is None:
                lookup[int(self.node_long[node_idx])] = 0.0
                continue
            prev_pos = self.positions[prev_idx]
            curr_pos = self.positions[node_idx]
            next_pos = self.positions[next_idx]
            v1 = curr_pos - prev_pos
            v2 = next_pos - curr_pos
            cross = v1[0] * v2[1] - v1[1] * v2[0]
            if abs(cross) < 1e-6:
                lookup[int(self.node_long[node_idx])] = 0.0
            else:
                lookup[int(self.node_long[node_idx])] = 1.0 if cross > 0.0 else -1.0
        return lookup

    def _compute_incourse_scores(self) -> np.ndarray:
        scores = np.zeros(len(self.nodes), dtype=np.float64)
        for idx in range(len(self.nodes)):
            lane = int(self.node_lane[idx])
            if lane == 0:
                scores[idx] = 1.0
                continue
            curve_sign = self.curvature_by_long.get(int(self.node_long[idx]), 0.0)
            if curve_sign == 0.0:
                scores[idx] = 0.0
            elif lane * curve_sign > 0:
                scores[idx] = max(0.3, 0.8 / (abs(lane) + 1))
            else:
                scores[idx] = 0.0
        return scores

    def set_obstacles(self, obstacles: List[Tuple[float, float, float]]) -> None:
        self.dynamic_obstacles = obstacles

    def find_nearest_node(self, x: float, y: float) -> int:
        if len(self.positions) == 0:
            raise RuntimeError('그래프 노드가 없습니다')
        if self.kdtree is not None:
            _, idx = self.kdtree.query([x, y])  # type: ignore[call-arg]
            return int(idx)
        diffs = self.positions - np.array([x, y])
        dists = np.hypot(diffs[:, 0], diffs[:, 1])
        return int(np.argmin(dists))

    def find_goal_node(self, start_idx: int, lookahead_distance: float) -> int:
        if len(self.center_lane_sequence) == 0:
            return start_idx
        interval = max(self.longitudinal_interval, 1e-3)
        steps = max(1, int(round(lookahead_distance / interval)))
        start_long = int(self.node_long[start_idx])
        center_pos = self.center_index_by_long.get(start_long)
        if center_pos is None:
            diffs = [abs(start_long - int(self.node_long[idx])) for idx in self.center_lane_sequence]
            center_pos = int(np.argmin(diffs))
        goal_pos = (center_pos + steps) % len(self.center_lane_sequence)
        return int(self.center_lane_sequence[goal_pos])

    def plan_path(self, start_idx: int, goal_idx: int) -> List[int]:
        if start_idx == goal_idx:
            return [start_idx]
        return self._a_star(start_idx, goal_idx)

    def _a_star(self, start_idx: int, goal_idx: int) -> List[int]:
        open_heap: List[Tuple[float, int]] = []
        heapq.heappush(open_heap, (self._heuristic(start_idx), start_idx))
        g_score: Dict[int, float] = {start_idx: 0.0}
        came_from: Dict[int, int] = {}
        closed: Set[int] = set()

        while open_heap:
            _, current = heapq.heappop(open_heap)
            if current in closed:
                continue
            closed.add(current)

            if current == goal_idx:
                return self._reconstruct_path(came_from, current)

            current_cost = g_score[current]
            for neighbor in self.adjacency.get(current, []):
                if neighbor in closed:
                    continue
                if self._edge_collides(current, neighbor):
                    continue
                step_cost = self._edge_cost(current, neighbor)
                tentative = current_cost + step_cost
                if tentative >= g_score.get(neighbor, math.inf):
                    continue
                came_from[neighbor] = current
                g_score[neighbor] = tentative
                f_score = tentative + self._heuristic(neighbor)
                heapq.heappush(open_heap, (f_score, neighbor))
        return []

    def _heuristic(self, node_idx: int) -> float:
        return self.weight_center * self.node_d_center[node_idx] - self.weight_incourse * self.node_incourse[node_idx]

    def _edge_cost(self, from_idx: int, to_idx: int) -> float:
        if self.edge_cost_mode == 'unit':
            return 1.0

        start = self.positions[from_idx]
        end = self.positions[to_idx]
        distance = float(np.linalg.norm(end - start))

        if self.edge_cost_mode == 'time':
            # 시간 기반 비용: distance / avg_velocity
            # 엣지의 평균 속도 사용
            v_from = max(0.1, float(self.node_max_velocity[from_idx]))
            v_to = max(0.1, float(self.node_max_velocity[to_idx]))
            avg_velocity = (v_from + v_to) / 2.0
            return distance / avg_velocity
        else:
            # 거리 기반 비용
            return distance

    def _edge_collides(self, from_idx: int, to_idx: int) -> bool:
        if not self.dynamic_obstacles:
            return False
        start = self.positions[from_idx]
        end = self.positions[to_idx]
        delta = end - start
        length = float(np.linalg.norm(delta))
        if length < 1e-6:
            return self._point_blocked(float(start[0]), float(start[1]))
        steps = max(2, int(math.ceil(length / self.collision_check_resolution)))
        for step in range(steps):
            ratio = step / (steps - 1)
            sample = start + delta * ratio
            if self._point_blocked(float(sample[0]), float(sample[1])):
                return True
        return False

    def _point_blocked(self, x: float, y: float) -> bool:
        for ox, oy, radius in self.dynamic_obstacles:
            dx = x - ox
            dy = y - oy
            if dx * dx + dy * dy <= radius * radius:
                return True
        return False

    def _reconstruct_path(self, came_from: Dict[int, int], current: int) -> List[int]:
        path = [current]
        while current in came_from:
            current = came_from[current]
            path.append(current)
        path.reverse()
        return path


class LocalAStarV2(Node):
    """장애물 기반 A* 로컬 플래너 ROS2 노드."""

    def __init__(self) -> None:
        super().__init__('local_astar_v2_node')

        # 파라미터 선언
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('output_topic', '/local_path')
        self.declare_parameter('graph_topic', '/graph')
        self.declare_parameter('graph_file_path', '')
        self.declare_parameter('lookahead_distance', 12.0)
        self.declare_parameter('update_rate', 20.0)
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('obstacle_topic', '/obstacle')
        self.declare_parameter('obstacle_inflation_radius', 0.3)
        self.declare_parameter('collision_check_resolution', 0.2)
        self.declare_parameter('edge_cost_mode', 'length')
        self.declare_parameter('heuristic_weight_center', 0.4)
        self.declare_parameter('heuristic_weight_incourse', 0.8)

        # 파라미터 읽기
        self.odom_topic = self.get_parameter('odom_topic').value
        self.output_topic = self.get_parameter('output_topic').value
        self.graph_topic = self.get_parameter('graph_topic').value
        self.graph_file_path = self.get_parameter('graph_file_path').value
        self.lookahead_distance = float(self.get_parameter('lookahead_distance').value)
        self.update_rate = float(self.get_parameter('update_rate').value)
        self.map_frame = self.get_parameter('map_frame').value
        self.obstacle_topic = self.get_parameter('obstacle_topic').value
        self.obstacle_inflation = float(self.get_parameter('obstacle_inflation_radius').value)
        self.collision_check_resolution = float(self.get_parameter('collision_check_resolution').value)
        self.edge_cost_mode = str(self.get_parameter('edge_cost_mode').value)
        self.heuristic_weight_center = float(self.get_parameter('heuristic_weight_center').value)
        self.heuristic_weight_incourse = float(self.get_parameter('heuristic_weight_incourse').value)

        if self.heuristic_weight_center >= self.heuristic_weight_incourse:
            self.get_logger().warn('권장 조건은 a < b 입니다. 파라미터를 확인하세요.')

        self.graph_planner: Optional[GraphPlanner] = None
        self.current_odom: Optional[Odometry] = None
        self.dynamic_obstacles: List[Tuple[float, float, float]] = []
        self.graph_visualized = False

        self.load_graph()

        # ROS I/O
        self.odom_sub = self.create_subscription(Odometry, self.odom_topic, self.odom_callback, 10)
        self.path_pub = self.create_publisher(Path, self.output_topic, 10)
        self.obstacle_sub = None
        if self.obstacle_topic:
            self.obstacle_sub = self.create_subscription(MarkerArray, self.obstacle_topic, self.obstacle_callback, 10)

        graph_qos = QoSProfile(
            depth=1,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            reliability=QoSReliabilityPolicy.RELIABLE,
        )
        self.graph_pub = self.create_publisher(MarkerArray, self.graph_topic, graph_qos)

        timer_period = 1.0 / max(1.0, self.update_rate)
        self.timer = self.create_timer(timer_period, self.planning_callback)
        self.graph_timer = self.create_timer(1.0, self.publish_graph_markers)

        self.get_logger().info('로컬 하이브리드 A* 플래너 노드를 시작했습니다')

    def load_graph(self) -> None:
        if not self.graph_file_path:
            self.get_logger().error('graph_file_path 파라미터가 비어 있습니다')
            return
        try:
            self.graph_planner = GraphPlanner(
                self.graph_file_path,
                collision_check_resolution=self.collision_check_resolution,
                edge_cost_mode=self.edge_cost_mode,
                heuristic_weight_center=self.heuristic_weight_center,
                heuristic_weight_incourse=self.heuristic_weight_incourse,
            )
            self.graph_planner.set_obstacles(self.dynamic_obstacles)
            hash_info = f', 해시: {self.graph_planner.graph_hash[:8]}' if self.graph_planner.graph_hash else ''
            velocity_info = f', 속도 정보 포함' if self.graph_planner.has_velocity_info else ''
            cost_mode_info = f', 비용 모드: {self.graph_planner.edge_cost_mode}'
            self.get_logger().info(
                f'그래프 로드 완료 (노드 {len(self.graph_planner.nodes)}개, '
                f'간선 {len(self.graph_planner.edges)}개{hash_info}{velocity_info}{cost_mode_info})'
            )
        except Exception as exc:  # pylint: disable=broad-except
            self.get_logger().error(f'그래프 로드 실패: {exc}')
            self.graph_planner = None

    # ------------------------------------------------------------------
    # 콜백
    # ------------------------------------------------------------------
    def odom_callback(self, msg: Odometry) -> None:
        self.current_odom = msg

    def obstacle_callback(self, msg: MarkerArray) -> None:
        new_obstacles: List[Tuple[float, float, float]] = []
        if not msg.markers:
            self.dynamic_obstacles = []
            if self.graph_planner is not None:
                self.graph_planner.set_obstacles([])
            return

        for marker in msg.markers:
            if marker.action == Marker.DELETEALL:
                new_obstacles = []
                break
            if marker.action not in (Marker.ADD,):
                continue
            radius = 0.5 * max(marker.scale.x, marker.scale.y)
            radius += max(0.0, self.obstacle_inflation)
            new_obstacles.append((float(marker.pose.position.x), float(marker.pose.position.y), radius))

        self.dynamic_obstacles = new_obstacles
        if self.graph_planner is not None:
            self.graph_planner.set_obstacles(new_obstacles)

    # ------------------------------------------------------------------
    # 계획 루틴
    # ------------------------------------------------------------------
    def planning_callback(self) -> None:
        if self.graph_planner is None or self.current_odom is None:
            return
        robot_x = self.current_odom.pose.pose.position.x
        robot_y = self.current_odom.pose.pose.position.y
        start_idx = self.graph_planner.find_nearest_node(robot_x, robot_y)
        goal_idx = self.graph_planner.find_goal_node(start_idx, self.lookahead_distance)
        path_indices = self.graph_planner.plan_path(start_idx, goal_idx)
        if not path_indices:
            self.get_logger().warn('A* 탐색 실패: 경로가 없습니다')
            return

        path_msg = Path()
        path_msg.header.frame_id = self.map_frame
        path_msg.header.stamp = self.get_clock().now().to_msg()
        for idx in path_indices:
            pose = PoseStamped()
            pose.header = path_msg.header
            pose.pose.position.x = float(self.graph_planner.nodes[idx][0])
            pose.pose.position.y = float(self.graph_planner.nodes[idx][1])
            theta = float(self.graph_planner.nodes[idx][2])
            pose.pose.orientation.z = math.sin(theta / 2.0)
            pose.pose.orientation.w = math.cos(theta / 2.0)
            path_msg.poses.append(pose)
        self.path_pub.publish(path_msg)

    # ------------------------------------------------------------------
    # 시각화
    # ------------------------------------------------------------------
    def publish_graph_markers(self) -> None:
        if self.graph_planner is None:
            return
        marker_array = MarkerArray()
        node_marker = Marker()
        node_marker.header.frame_id = self.map_frame
        node_marker.header.stamp = self.get_clock().now().to_msg()
        node_marker.ns = 'graph_nodes'
        node_marker.id = 0
        node_marker.type = Marker.SPHERE_LIST
        node_marker.scale.x = 0.08
        node_marker.scale.y = 0.08
        node_marker.scale.z = 0.08
        node_marker.color = ColorRGBA(r=0.2, g=0.8, b=1.0, a=0.8)
        node_marker.pose.orientation.w = 1.0
        for node in self.graph_planner.nodes:
            point = Point()
            point.x = float(node[0])
            point.y = float(node[1])
            point.z = 0.05
            node_marker.points.append(point)
        marker_array.markers.append(node_marker)

        edge_marker = Marker()
        edge_marker.header.frame_id = self.map_frame
        edge_marker.header.stamp = node_marker.header.stamp
        edge_marker.ns = 'graph_edges'
        edge_marker.id = 1
        edge_marker.type = Marker.LINE_LIST
        edge_marker.scale.x = 0.02
        edge_marker.color = ColorRGBA(r=0.7, g=0.7, b=0.7, a=0.5)
        edge_marker.pose.orientation.w = 1.0
        for frm, to in self.graph_planner.edges:
            p1 = Point()
            p1.x = float(self.graph_planner.nodes[int(frm)][0])
            p1.y = float(self.graph_planner.nodes[int(frm)][1])
            p1.z = 0.05
            p2 = Point()
            p2.x = float(self.graph_planner.nodes[int(to)][0])
            p2.y = float(self.graph_planner.nodes[int(to)][1])
            p2.z = 0.05
            edge_marker.points.append(p1)
            edge_marker.points.append(p2)
        marker_array.markers.append(edge_marker)

        self.graph_pub.publish(marker_array)
        if not self.graph_visualized:
            self.get_logger().info('그래프 MarkerArray를 발행하기 시작했습니다')
            self.graph_visualized = True


def main(args=None) -> None:
    rclpy.init(args=args)
    node = LocalAStarV2()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
