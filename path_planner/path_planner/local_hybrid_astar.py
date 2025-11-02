#!/usr/bin/env python3
import math
import heapq
from dataclasses import dataclass
from typing import Dict, List, Optional, Tuple

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy, QoSHistoryPolicy
from sensor_msgs.msg import LaserScan


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
        self.declare_parameter('costmap_topic', '/local_costmap')
        self.declare_parameter('static_map_topic', '/map')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('publish_costmap', True)

        # Costmap / vehicle configuration
        self.declare_parameter('grid_resolution', 0.25)
        self.declare_parameter('grid_width', 20.0)
        self.declare_parameter('grid_height', 20.0)
        self.declare_parameter('inflation_radius', 0.5)
        self.declare_parameter('vehicle_radius', 0.35)

        # Planner parameters
        self.declare_parameter('heading_bins', 72)
        self.declare_parameter('num_steering_samples', 7)
        self.declare_parameter('max_steering_angle', 0.4)
        self.declare_parameter('wheelbase', 0.33)
        self.declare_parameter('step_size', 0.3)
        self.declare_parameter('lookahead_distance', 6.0)
        self.declare_parameter('goal_tolerance', 0.4)
        self.declare_parameter('goal_yaw_tolerance_deg', 30.0)
        self.declare_parameter('planner_hz', 10.0)
        self.declare_parameter('max_iterations', 3000)

        # Cost weights
        self.declare_parameter('steering_change_weight', 3.0)
        self.declare_parameter('obstacle_cost_weight', 25.0)
        self.declare_parameter('path_distance_weight', 3.0)
        self.declare_parameter('heuristic_weight', 1.0)
        self.declare_parameter('global_path_deviation_weight', 8.0)
        self.declare_parameter('static_map_penalty_weight', 20.0)

        # Parameter retrieval
        self.global_path_topic = self.get_parameter('global_path_topic').get_parameter_value().string_value
        self.scan_topic = self.get_parameter('scan_topic').get_parameter_value().string_value
        self.odom_topic = self.get_parameter('odom_topic').get_parameter_value().string_value
        self.output_topic = self.get_parameter('output_topic').get_parameter_value().string_value
        self.costmap_topic = self.get_parameter('costmap_topic').get_parameter_value().string_value
        self.static_map_topic = self.get_parameter('static_map_topic').get_parameter_value().string_value
        self.map_frame = self.get_parameter('map_frame').get_parameter_value().string_value
        self.base_frame = self.get_parameter('base_frame').get_parameter_value().string_value
        self.publish_costmap = bool(self.get_parameter('publish_costmap').value)

        self.grid_resolution = float(self.get_parameter('grid_resolution').value)
        self.grid_width = float(self.get_parameter('grid_width').value)
        self.grid_height = float(self.get_parameter('grid_height').value)
        self.inflation_radius = float(self.get_parameter('inflation_radius').value)
        self.vehicle_radius = float(self.get_parameter('vehicle_radius').value)

        self.heading_bins = int(self.get_parameter('heading_bins').value)
        self.num_steering_samples = max(3, int(self.get_parameter('num_steering_samples').value))
        self.max_steering_angle = float(self.get_parameter('max_steering_angle').value)
        self.wheelbase = max(1e-3, float(self.get_parameter('wheelbase').value))
        self.step_size = float(self.get_parameter('step_size').value)
        self.lookahead_distance = float(self.get_parameter('lookahead_distance').value)
        self.goal_tolerance = float(self.get_parameter('goal_tolerance').value)
        self.goal_yaw_tolerance = math.radians(float(self.get_parameter('goal_yaw_tolerance_deg').value))
        self.planner_hz = float(self.get_parameter('planner_hz').value)
        self.max_iterations = int(self.get_parameter('max_iterations').value)

        self.steering_change_weight = float(self.get_parameter('steering_change_weight').value)
        self.obstacle_cost_weight = float(self.get_parameter('obstacle_cost_weight').value)
        self.path_distance_weight = float(self.get_parameter('path_distance_weight').value)
        self.heuristic_weight = float(self.get_parameter('heuristic_weight').value)
        self.global_path_deviation_weight = float(self.get_parameter('global_path_deviation_weight').value)
        self.static_map_penalty_weight = float(self.get_parameter('static_map_penalty_weight').value)

        # Derived costmap sizes
        self.grid_cols = max(3, int(math.ceil(self.grid_width / self.grid_resolution)))
        self.grid_rows = max(3, int(math.ceil(self.grid_height / self.grid_resolution)))
        self.obstacle_grid = np.zeros((self.grid_rows, self.grid_cols), dtype=np.uint8)

        # Runtime state
        self.global_path_np: Optional[np.ndarray] = None  # shape (N, 3) for x, y, yaw
        self.latest_scan: Optional[LaserScan] = None
        self.latest_pose: Optional[Tuple[float, float, float]] = None  # x, y, yaw
        self.last_path_msg: Optional[Path] = None
        self.costmap_origin: Tuple[float, float] = (0.0, 0.0)

        # Static map state
        self.static_map: Optional[np.ndarray] = None
        self.static_map_info = None

        self._last_warnings: Dict[str, float] = {}

        # ROS interfaces
        self.global_path_sub = self.create_subscription(Path, self.global_path_topic, self._on_global_path, 10)
        self.scan_sub = self.create_subscription(LaserScan, self.scan_topic, self._on_scan, 10)
        self.odom_sub = self.create_subscription(Odometry, self.odom_topic, self._on_odom, 10)

        # Static map subscription with TRANSIENT_LOCAL QoS
        self.static_map_sub = self.create_subscription(
            OccupancyGrid,
            self.static_map_topic,
            self._on_static_map,
            QoSProfile(
                durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
                reliability=QoSReliabilityPolicy.RELIABLE,
                history=QoSHistoryPolicy.KEEP_LAST,
                depth=1
            )
        )

        self.path_pub = self.create_publisher(Path, self.output_topic, 10)
        self.costmap_pub = (
            self.create_publisher(OccupancyGrid, self.costmap_topic, 1)
            if self.publish_costmap
            else None
        )

        timer_period = 1.0 / max(0.1, self.planner_hz)
        self.timer = self.create_timer(timer_period, self._on_timer)

        self.get_logger().info(
            'Hybrid A* local planner 초기화 완료 '
            f'(grid {self.grid_cols}x{self.grid_rows}, resolution {self.grid_resolution:.2f} m)'
        )

    # --------------------------------------------------------------------- #
    # ROS callbacks
    # --------------------------------------------------------------------- #
    def _on_global_path(self, msg: Path) -> None:
        if not msg.poses:
            self.global_path_np = None
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

    def _on_scan(self, msg: LaserScan) -> None:
        self.latest_scan = msg

    def _on_odom(self, msg: Odometry) -> None:
        pos = msg.pose.pose.position
        ori = msg.pose.pose.orientation
        siny_cosp = 2.0 * (ori.w * ori.z + ori.x * ori.y)
        cosy_cosp = 1.0 - 2.0 * (ori.y * ori.y + ori.z * ori.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        self.latest_pose = (pos.x, pos.y, yaw)

    def _on_static_map(self, msg: OccupancyGrid) -> None:
        """정적 맵을 numpy array로 변환하여 저장"""
        width = msg.info.width
        height = msg.info.height

        # OccupancyGrid (-1=unknown, 0=free, 100=occupied) → numpy
        data = np.array(msg.data, dtype=np.int8).reshape((height, width))
        self.static_map = data
        self.static_map_info = msg.info

        self.get_logger().info(
            f'정적 맵 로드 완료: {width}x{height}, '
            f'해상도 {msg.info.resolution:.3f}m'
        )

    # --------------------------------------------------------------------- #
    # Planner loop
    # --------------------------------------------------------------------- #
    def _on_timer(self) -> None:
        if not self._data_ready():
            return

        if not self._update_costmap():
            self._throttled_warn('costmap', '코스트맵을 업데이트할 수 없습니다.')
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
    # Costmap utilities
    # --------------------------------------------------------------------- #
    def _update_costmap(self) -> bool:
        if self.latest_pose is None or self.latest_scan is None:
            return False

        self.obstacle_grid.fill(0)
        robot_x, robot_y, robot_yaw = self.latest_pose

        # 코스트맵을 로봇 앞쪽으로 배치 (로봇 방향 기준)
        # x축: 로봇 뒤쪽 20%, 앞쪽 80%
        # y축: 좌우 대칭 (50%, 50%)
        forward_ratio = 0.8  # 앞쪽 비율
        cos_yaw = math.cos(robot_yaw)
        sin_yaw = math.sin(robot_yaw)

        # 로봇 기준 로컬 좌표에서 오프셋 계산
        local_offset_x = (forward_ratio - 0.5) * self.grid_width
        local_offset_y = 0.0

        # 월드 좌표로 변환
        world_offset_x = local_offset_x * cos_yaw - local_offset_y * sin_yaw
        world_offset_y = local_offset_x * sin_yaw + local_offset_y * cos_yaw

        # 코스트맵 중심을 오프셋만큼 이동
        costmap_center_x = robot_x + world_offset_x
        costmap_center_y = robot_y + world_offset_y

        # 코스트맵 원점 계산 (중심에서 절반씩 이동)
        origin_x = costmap_center_x - (self.grid_cols * self.grid_resolution) * 0.5
        origin_y = costmap_center_y - (self.grid_rows * self.grid_resolution) * 0.5
        self.costmap_origin = (origin_x, origin_y)

        scan = self.latest_scan
        angle = scan.angle_min
        max_range = min(
            0.5 * math.hypot(self.grid_width, self.grid_height),
            scan.range_max
        )

        for rng in scan.ranges:
            if not math.isfinite(rng) or rng < scan.range_min or rng > max_range:
                angle += scan.angle_increment
                continue
            lx = rng * math.cos(angle)
            ly = rng * math.sin(angle)
            mx = robot_x + lx * cos_yaw - ly * sin_yaw
            my = robot_y + lx * sin_yaw + ly * cos_yaw
            self._mark_occupied(mx, my)
            angle += scan.angle_increment

        self._inflate_obstacles()
        if self.costmap_pub is not None:
            self._publish_costmap()
        return True

    def _mark_occupied(self, x: float, y: float) -> None:
        cell = self._world_to_cell(x, y)
        if cell is None:
            return
        row, col = cell
        self.obstacle_grid[row, col] = 255

    def _inflate_obstacles(self) -> None:
        inflation_cells = int(math.ceil(max(0.0, self.inflation_radius) / self.grid_resolution))
        if inflation_cells <= 0:
            return

        obstacle_indices = np.argwhere(self.obstacle_grid == 255)
        if obstacle_indices.size == 0:
            return

        for row, col in obstacle_indices:
            row_min = max(0, row - inflation_cells)
            row_max = min(self.grid_rows - 1, row + inflation_cells)
            col_min = max(0, col - inflation_cells)
            col_max = min(self.grid_cols - 1, col + inflation_cells)
            for r in range(row_min, row_max + 1):
                for c in range(col_min, col_max + 1):
                    if self.obstacle_grid[r, c] == 255:
                        continue
                    dr = (r - row) * self.grid_resolution
                    dc = (c - col) * self.grid_resolution
                    if math.hypot(dr, dc) <= self.inflation_radius + 1e-6:
                        self.obstacle_grid[r, c] = max(self.obstacle_grid[r, c], 200)

    def _world_to_cell(self, x: float, y: float) -> Optional[Tuple[int, int]]:
        origin_x, origin_y = self.costmap_origin
        col = int(math.floor((x - origin_x) / self.grid_resolution))
        row = int(math.floor((y - origin_y) / self.grid_resolution))
        if col < 0 or col >= self.grid_cols or row < 0 or row >= self.grid_rows:
            return None
        return row, col

    def _cell_value(self, x: float, y: float) -> float:
        cell = self._world_to_cell(x, y)
        if cell is None:
            return 1.0
        row, col = cell
        value = self.obstacle_grid[row, col]
        if value >= 250:
            return 1.0
        if value > 0:
            return 0.5
        return 0.0

    def _collision_check(self, state: HybridState, next_state: HybridState) -> bool:
        # Sample along the transition to detect collisions
        steps = max(2, int(math.ceil(self.step_size / (self.grid_resolution * 0.5))))
        for i in range(steps + 1):
            t = i / steps
            x = state.x + (next_state.x - state.x) * t
            y = state.y + (next_state.y - state.y) * t
            if self._point_in_collision(x, y):
                return True
        return False

    def _publish_costmap(self) -> None:
        msg = OccupancyGrid()
        msg.header.frame_id = self.map_frame
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.info.resolution = self.grid_resolution
        msg.info.width = self.grid_cols
        msg.info.height = self.grid_rows
        msg.info.origin.position.x = self.costmap_origin[0]
        msg.info.origin.position.y = self.costmap_origin[1]
        msg.info.origin.position.z = 0.0
        msg.info.origin.orientation.w = 1.0

        grid = self.obstacle_grid
        costmap_view = np.full(grid.shape, -1, dtype=np.int8)
        costmap_view[grid >= 250] = 100  # occupied
        inflated_mask = (grid > 0) & (grid < 250)
        costmap_view[inflated_mask] = 70  # inflated but free-ish
        msg.data = [int(v) for v in costmap_view.flatten(order='C')]
        self.costmap_pub.publish(msg)

    def _point_in_collision(self, x: float, y: float) -> bool:
        """동적 장애물(LiDAR) 충돌 체크 - 코스트맵 범위 내에서만"""
        # 코스트맵이 업데이트되지 않았으면 충돌 없음
        if self.costmap_origin is None:
            return False

        origin_x, origin_y = self.costmap_origin
        col_f = (x - origin_x) / self.grid_resolution
        row_f = (y - origin_y) / self.grid_resolution
        col = int(math.floor(col_f))
        row = int(math.floor(row_f))
        vehicle_cells = int(math.ceil(self.vehicle_radius / self.grid_resolution))

        for r in range(row - vehicle_cells, row + vehicle_cells + 1):
            # 코스트맵 범위 밖은 무시 (정적 맵에서만 체크)
            if r < 0 or r >= self.grid_rows:
                continue
            for c in range(col - vehicle_cells, col + vehicle_cells + 1):
                # 코스트맵 범위 밖은 무시 (정적 맵에서만 체크)
                if c < 0 or c >= self.grid_cols:
                    continue
                if self.obstacle_grid[r, c] > 0:
                    center_x = (c + 0.5) * self.grid_resolution + origin_x
                    center_y = (r + 0.5) * self.grid_resolution + origin_y
                    if math.hypot(center_x - x, center_y - y) <= self.vehicle_radius:
                        return True
        return False

    # --------------------------------------------------------------------- #
    # Static map utilities
    # --------------------------------------------------------------------- #
    def _static_map_collision(self, x: float, y: float) -> bool:
        """정적 맵에서 충돌 체크 (차량 반경 고려)"""
        if self.static_map is None or self.static_map_info is None:
            return False

        # 차량 중심 좌표 → 맵 좌표
        map_x = int((x - self.static_map_info.origin.position.x) / self.static_map_info.resolution)
        map_y = int((y - self.static_map_info.origin.position.y) / self.static_map_info.resolution)

        height, width = self.static_map.shape

        # 맵 밖은 충돌 아님 (페널티는 _get_map_based_penalty에서 처리)
        if map_x < 0 or map_x >= width or map_y < 0 or map_y >= height:
            return False

        # 차량 반경 내 모든 셀 체크
        vehicle_cells = int(math.ceil(self.vehicle_radius / self.static_map_info.resolution))

        for dy in range(-vehicle_cells, vehicle_cells + 1):
            for dx in range(-vehicle_cells, vehicle_cells + 1):
                check_x = map_x + dx
                check_y = map_y + dy

                # 맵 범위 체크는 건너뛰기
                if check_x < 0 or check_x >= width or check_y < 0 or check_y >= height:
                    continue

                # 원형 차량 모델
                dist = math.hypot(dx * self.static_map_info.resolution,
                                dy * self.static_map_info.resolution)
                if dist > self.vehicle_radius:
                    continue

                cell_value = self.static_map[check_y, check_x]
                if cell_value >= 50:  # Occupied or high uncertainty
                    return True

        return False

    def _get_map_based_penalty(self, x: float, y: float) -> float:
        """정적 맵 기반 페널티 계산"""
        if self.static_map is None or self.static_map_info is None:
            return 0.0

        # World 좌표 → 맵 좌표 변환
        map_x = int((x - self.static_map_info.origin.position.x) / self.static_map_info.resolution)
        map_y = int((y - self.static_map_info.origin.position.y) / self.static_map_info.resolution)

        height, width = self.static_map.shape

        # 맵 밖이면 매우 큰 페널티
        if map_x < 0 or map_x >= width or map_y < 0 or map_y >= height:
            return 10.0

        cell_value = self.static_map[map_y, map_x]

        # OccupancyGrid 값에 따른 페널티
        if cell_value >= 80:  # Occupied (거의 확실한 장애물)
            return 10.0
        elif cell_value >= 50:  # Probably occupied
            return 5.0
        elif cell_value >= 20:  # Low confidence
            return 1.0
        elif cell_value == -1:  # Unknown
            return 2.0
        else:  # Free (0~20)
            return 0.0

    def _point_to_segment_distance(self, point: np.ndarray, seg_start: np.ndarray, seg_end: np.ndarray) -> float:
        """점에서 선분까지의 최단 거리"""
        px, py = point[0], point[1]
        x1, y1 = seg_start[0], seg_start[1]
        x2, y2 = seg_end[0], seg_end[1]

        dx = x2 - x1
        dy = y2 - y1

        if abs(dx) < 1e-9 and abs(dy) < 1e-9:
            return math.hypot(px - x1, py - y1)

        # 선분에 투영
        t = max(0.0, min(1.0, ((px - x1) * dx + (py - y1) * dy) / (dx * dx + dy * dy)))
        proj_x = x1 + t * dx
        proj_y = y1 + t * dy

        return math.hypot(px - proj_x, py - proj_y)

    def _get_path_deviation(self, x: float, y: float) -> float:
        """글로벌 경로로부터의 거리 계산 (로봇 위치 기준 최적화)"""
        if self.global_path_np is None or len(self.global_path_np) < 2:
            return 0.0

        if self.latest_pose is None:
            return 0.0

        # 로봇 현재 위치 기준으로 가까운 경로 세그먼트만 검색
        robot_pos = np.array([self.latest_pose[0], self.latest_pose[1]])

        # 로봇 위치에서 가장 가까운 글로벌 경로 포인트 찾기
        distances_to_robot = np.linalg.norm(self.global_path_np[:, :2] - robot_pos, axis=1)
        closest_to_robot_idx = int(np.argmin(distances_to_robot))

        # 로봇 앞쪽 경로만 검색 (현재 위치 +10 ~ +40 범위, 총 30개)
        lookahead_offset = 10
        search_range = 30
        start_idx = min(len(self.global_path_np) - 1, closest_to_robot_idx + lookahead_offset)
        end_idx = min(len(self.global_path_np) - 1, start_idx + search_range)

        # 현재 상태에서 가까운 세그먼트까지의 거리 계산
        state_pos = np.array([x, y])
        min_dist = float('inf')

        for i in range(start_idx, end_idx):
            if i + 1 >= len(self.global_path_np):
                break
            p1 = self.global_path_np[i, :2]
            p2 = self.global_path_np[i + 1, :2]
            dist = self._point_to_segment_distance(state_pos, p1, p2)
            min_dist = min(min_dist, dist)

        return min_dist

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

    def _heuristic(self, state: HybridState, goal: Dict[str, float]) -> float:
        """
        3단계 비용:
        1. 목표까지 거리 (기본)
        2. 글로벌 경로로부터의 거리 (강한 페널티)
        3. 맵 외곽/불가능 영역 (매우 큰 페널티)
        """
        # 1. 기본 거리 + 방향
        distance = math.hypot(state.x - goal['x'], state.y - goal['y'])
        yaw_error = abs(wrap_angle(state.yaw - goal['yaw']))

        # 2. 글로벌 경로로부터의 거리 (세그먼트 기반) - 최적화
        path_deviation = self._get_path_deviation(state.x, state.y)

        # 3. 맵 기반 페널티 (주행 불가능 영역 근처)
        map_penalty = self._get_map_based_penalty(state.x, state.y)

        return (
            self.heuristic_weight * distance                          # 목표 거리
            + 0.3 * yaw_error                                         # 방향 오차
            + self.global_path_deviation_weight * path_deviation      # 글로벌 경로 이탈
            + self.static_map_penalty_weight * map_penalty            # 맵 외곽/불가 영역
        )

    def _is_goal(self, state: HybridState, goal: Dict[str, float]) -> bool:
        distance = math.hypot(state.x - goal['x'], state.y - goal['y'])
        if distance > self.goal_tolerance:
            return False
        yaw_error = abs(wrap_angle(state.yaw - goal['yaw']))
        return yaw_error <= self.goal_yaw_tolerance

    # --------------------------------------------------------------------- #
    # Hybrid A* search
    # --------------------------------------------------------------------- #
    def _plan_hybrid_astar(self, goal: Dict[str, float]) -> Optional[List[HybridState]]:
        if self.latest_pose is None:
            return None

        start_state = HybridState(
            x=self.latest_pose[0],
            y=self.latest_pose[1],
            yaw=self.latest_pose[2],
            steering=0.0,
        )

        # 디버깅: 시작 상태가 유효한지 확인
        if self._static_map_collision(start_state.x, start_state.y):
            self.get_logger().warn(
                f'시작 위치({start_state.x:.2f}, {start_state.y:.2f})가 정적 맵 장애물과 충돌합니다.'
            )
            return None

        start_key = self._discretize_state(start_state)
        if start_key is None:
            self.get_logger().warn('시작 상태를 이산화할 수 없습니다.')
            return None

        # 디버깅: 목표점 정보
        goal_dist = math.hypot(start_state.x - goal['x'], start_state.y - goal['y'])
        self.get_logger().debug(
            f'Hybrid A* 시작: 목표거리={goal_dist:.2f}m, '
            f'목표=({goal["x"]:.2f}, {goal["y"]:.2f})'
        )

        open_list: List[Tuple[float, float, Tuple[int, int, int], HybridState]] = []

        g_scores: Dict[Tuple[int, int, int], float] = {start_key: 0.0}
        came_from: Dict[Tuple[int, int, int], Optional[Tuple[int, int, int]]] = {start_key: None}
        state_lookup: Dict[Tuple[int, int, int], HybridState] = {start_key: start_state}

        start_h = self._heuristic(start_state, goal)
        heapq.heappush(open_list, (start_h, 0.0, start_key, start_state))

        iterations = 0
        expanded_count = 0

        while open_list and iterations < self.max_iterations:
            iterations += 1
            _, current_g, current_key, current_state = heapq.heappop(open_list)

            if current_g > g_scores.get(current_key, float('inf')) + 1e-6:
                continue

            if self._is_goal(current_state, goal):
                path = self._reconstruct_path(current_key, came_from, state_lookup)
                self.get_logger().debug(
                    f'Hybrid A* 성공: {iterations}회 반복, {expanded_count}개 확장, 경로길이={len(path)}'
                )
                return path

            expanded_this_state = 0
            expand_candidates = list(self._expand_state(current_state))

            discretize_fail = 0
            g_score_fail = 0
            lidar_col_fail = 0
            static_col_fail = 0

            for next_state, travel_cost in expand_candidates:
                next_key = self._discretize_state(next_state)
                if next_key is None:
                    discretize_fail += 1
                    continue

                # 동적 장애물 페널티 (LiDAR)
                obstacle_penalty = self.obstacle_cost_weight * self._cell_value(next_state.x, next_state.y)

                # 조향 변화 페널티
                steering_penalty = self.steering_change_weight * abs(next_state.steering - current_state.steering)

                # 글로벌 경로 이탈 페널티 (g-cost) - 최적화
                path_deviation = self._get_path_deviation(next_state.x, next_state.y)
                path_deviation_cost = self.path_distance_weight * path_deviation

                tentative_g = current_g + travel_cost + obstacle_penalty + steering_penalty + path_deviation_cost

                if tentative_g >= g_scores.get(next_key, float('inf')) - 1e-6:
                    g_score_fail += 1
                    continue

                # 동적 장애물 충돌 체크 (LiDAR)
                if self._collision_check(current_state, next_state):
                    lidar_col_fail += 1
                    continue

                # 정적 맵 충돌 체크
                if self._static_map_collision(next_state.x, next_state.y):
                    static_col_fail += 1
                    continue

                g_scores[next_key] = tentative_g
                came_from[next_key] = current_key
                state_lookup[next_key] = next_state
                h = self._heuristic(next_state, goal)
                heapq.heappush(open_list, (tentative_g + h, tentative_g, next_key, next_state))
                expanded_this_state += 1

            # 디버깅: 확장 실패 시 로그
            if len(expand_candidates) > 0 and expanded_this_state == 0:
                self.get_logger().warn(
                    f'확장 실패: 후보={len(expand_candidates)}, '
                    f'이산화실패={discretize_fail}, g-score={g_score_fail}, '
                    f'LiDAR충돌={lidar_col_fail}, 정적맵충돌={static_col_fail}'
                )

            expanded_count += expanded_this_state

        # 실패 원인 분석
        self.get_logger().warn(
            f'Hybrid A* 실패: {iterations}회 반복, {expanded_count}개 상태 확장, '
            f'목표거리={goal_dist:.2f}m'
        )
        return None

    def _expand_state(self, state: HybridState) -> List[Tuple[HybridState, float]]:
        controls = np.linspace(-self.max_steering_angle, self.max_steering_angle, self.num_steering_samples)
        next_candidates: List[Tuple[HybridState, float]] = []

        generated_count = 0
        lidar_collision_count = 0

        for steering in controls:
            beta = self.step_size * math.tan(steering) / self.wheelbase
            if abs(beta) < 1e-6:
                next_x = state.x + self.step_size * math.cos(state.yaw)
                next_y = state.y + self.step_size * math.sin(state.yaw)
                next_yaw = wrap_angle(state.yaw)
            else:
                next_yaw = wrap_angle(state.yaw + beta)
                r = self.step_size / beta
                next_x = state.x + r * (math.sin(next_yaw) - math.sin(state.yaw))
                next_y = state.y - r * (math.cos(next_yaw) - math.cos(state.yaw))

            next_state = HybridState(x=next_x, y=next_y, yaw=next_yaw, steering=float(steering))
            generated_count += 1

            # 직진 선호 (조향각에 비례한 비용)
            travel_cost = self.step_size * (1.0 + 0.3 * abs(steering))

            if self._point_in_collision(next_state.x, next_state.y):
                lidar_collision_count += 1
                continue
            next_candidates.append((next_state, travel_cost))

        # 디버깅: 첫 번째 상태 확장 시 정보 출력
        if generated_count > 0 and len(next_candidates) == 0:
            self.get_logger().warn(
                f'_expand_state: {generated_count}개 생성, '
                f'{lidar_collision_count}개 LiDAR 충돌 필터링, '
                f'최종 0개 반환'
            )

        return next_candidates

    def _discretize_state(self, state: HybridState) -> Optional[Tuple[int, int, int]]:
        cell = self._world_to_cell(state.x, state.y)
        if cell is None:
            return None
        row, col = cell
        yaw = wrap_angle(state.yaw)
        yaw_norm = (yaw + math.pi) / (2.0 * math.pi)
        yaw_bin = int(math.floor(yaw_norm * self.heading_bins)) % self.heading_bins
        return row, col, yaw_bin

    def _reconstruct_path(
        self,
        key: Tuple[int, int, int],
        came_from: Dict[Tuple[int, int, int], Optional[Tuple[int, int, int]]],
        state_lookup: Dict[Tuple[int, int, int], HybridState],
    ) -> List[HybridState]:
        path: List[HybridState] = []
        current_key: Optional[Tuple[int, int, int]] = key
        while current_key is not None:
            path.append(state_lookup[current_key])
            current_key = came_from.get(current_key)
        path.reverse()
        return path

    def _states_to_path(self, states: List[HybridState]) -> Path:
        """Hybrid A* 결과를 조밀한 경로로 변환 (보간 포함)"""

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

        # 2. Hybrid A* 경로 생성 (원본)
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
