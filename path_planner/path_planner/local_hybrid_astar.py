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
        self.declare_parameter('num_steering_samples', 5)
        self.declare_parameter('max_steering_angle', 0.6)
        self.declare_parameter('wheelbase', 0.33)
        self.declare_parameter('step_size', 0.6)
        self.declare_parameter('lookahead_distance', 8.0)
        self.declare_parameter('goal_tolerance', 0.6)
        self.declare_parameter('goal_yaw_tolerance_deg', 30.0)
        self.declare_parameter('planner_hz', 5.0)
        self.declare_parameter('max_iterations', 4000)

        # Cost weights
        self.declare_parameter('steering_change_weight', 2.0)
        self.declare_parameter('obstacle_cost_weight', 15.0)
        self.declare_parameter('path_distance_weight', 1.5)
        self.declare_parameter('heuristic_weight', 1.2)

        # Parameter retrieval
        self.global_path_topic = self.get_parameter('global_path_topic').get_parameter_value().string_value
        self.scan_topic = self.get_parameter('scan_topic').get_parameter_value().string_value
        self.odom_topic = self.get_parameter('odom_topic').get_parameter_value().string_value
        self.output_topic = self.get_parameter('output_topic').get_parameter_value().string_value
        self.costmap_topic = self.get_parameter('costmap_topic').get_parameter_value().string_value
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

        self._last_warnings: Dict[str, float] = {}

        # ROS interfaces
        self.global_path_sub = self.create_subscription(Path, self.global_path_topic, self._on_global_path, 10)
        self.scan_sub = self.create_subscription(LaserScan, self.scan_topic, self._on_scan, 10)
        self.odom_sub = self.create_subscription(Odometry, self.odom_topic, self._on_odom, 10)
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
        origin_x = robot_x - (self.grid_cols * self.grid_resolution) * 0.5
        origin_y = robot_y - (self.grid_rows * self.grid_resolution) * 0.5
        self.costmap_origin = (origin_x, origin_y)

        scan = self.latest_scan
        angle = scan.angle_min
        cos_yaw = math.cos(robot_yaw)
        sin_yaw = math.sin(robot_yaw)
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
        origin_x, origin_y = self.costmap_origin
        col_f = (x - origin_x) / self.grid_resolution
        row_f = (y - origin_y) / self.grid_resolution
        col = int(math.floor(col_f))
        row = int(math.floor(row_f))
        vehicle_cells = int(math.ceil(self.vehicle_radius / self.grid_resolution))

        for r in range(row - vehicle_cells, row + vehicle_cells + 1):
            if r < 0 or r >= self.grid_rows:
                return True
            for c in range(col - vehicle_cells, col + vehicle_cells + 1):
                if c < 0 or c >= self.grid_cols:
                    return True
                if self.obstacle_grid[r, c] > 0:
                    center_x = (c + 0.5) * self.grid_resolution + origin_x
                    center_y = (r + 0.5) * self.grid_resolution + origin_y
                    if math.hypot(center_x - x, center_y - y) <= self.vehicle_radius:
                        return True
        return False

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
        distance = math.hypot(state.x - goal['x'], state.y - goal['y'])
        yaw_error = abs(wrap_angle(state.yaw - goal['yaw']))
        path_dist = 0.0
        if self.global_path_np is not None and len(self.global_path_np) > 0:
            path_dist = float(np.min(np.linalg.norm(self.global_path_np[:, :2] - np.array([[state.x, state.y]]), axis=1)))
        return (
            self.heuristic_weight * distance
            + 0.3 * yaw_error
            + self.path_distance_weight * path_dist
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
        start_key = self._discretize_state(start_state)
        open_list: List[Tuple[float, float, Tuple[int, int, int], HybridState]] = []

        g_scores: Dict[Tuple[int, int, int], float] = {start_key: 0.0}
        came_from: Dict[Tuple[int, int, int], Optional[Tuple[int, int, int]]] = {start_key: None}
        state_lookup: Dict[Tuple[int, int, int], HybridState] = {start_key: start_state}

        start_h = self._heuristic(start_state, goal)
        heapq.heappush(open_list, (start_h, 0.0, start_key, start_state))

        iterations = 0

        while open_list and iterations < self.max_iterations:
            iterations += 1
            _, current_g, current_key, current_state = heapq.heappop(open_list)

            if current_g > g_scores.get(current_key, float('inf')) + 1e-6:
                continue

            if self._is_goal(current_state, goal):
                return self._reconstruct_path(current_key, came_from, state_lookup)

            for next_state, travel_cost in self._expand_state(current_state):
                next_key = self._discretize_state(next_state)
                if next_key is None:
                    continue
                obstacle_penalty = self.obstacle_cost_weight * self._cell_value(next_state.x, next_state.y)
                steering_penalty = self.steering_change_weight * abs(next_state.steering - current_state.steering)
                tentative_g = current_g + travel_cost + obstacle_penalty + steering_penalty

                if tentative_g >= g_scores.get(next_key, float('inf')) - 1e-6:
                    continue

                if self._collision_check(current_state, next_state):
                    continue

                g_scores[next_key] = tentative_g
                came_from[next_key] = current_key
                state_lookup[next_key] = next_state
                h = self._heuristic(next_state, goal)
                heapq.heappush(open_list, (tentative_g + h, tentative_g, next_key, next_state))

        self.get_logger().debug('Hybrid A* maximum iteration에 도달했습니다.')
        return None

    def _expand_state(self, state: HybridState) -> List[Tuple[HybridState, float]]:
        controls = np.linspace(-self.max_steering_angle, self.max_steering_angle, self.num_steering_samples)
        next_candidates: List[Tuple[HybridState, float]] = []

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
            travel_cost = self.step_size
            if self._point_in_collision(next_state.x, next_state.y):
                continue
            next_candidates.append((next_state, travel_cost))

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
        path_msg = Path()
        path_msg.header.frame_id = self.map_frame
        stamp = self.get_clock().now().to_msg()
        path_msg.header.stamp = stamp

        for state in states:
            pose = PoseStamped()
            pose.header.frame_id = self.map_frame
            pose.header.stamp = stamp
            pose.pose.position.x = state.x
            pose.pose.position.y = state.y
            pose.pose.position.z = 0.0
            half_yaw = state.yaw * 0.5
            pose.pose.orientation.z = math.sin(half_yaw)
            pose.pose.orientation.w = math.cos(half_yaw)
            path_msg.poses.append(pose)

        return path_msg


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
