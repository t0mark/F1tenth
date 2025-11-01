#!/usr/bin/env python3
"""
Local DWA Planner
=================
단일 노드에서 로컬 비용맵 생성과 Dynamic Window Approach 기반 속도 명령 및 로컬 경로를 계산한다.
"""

# 표준 수학 연산과 타입 힌트를 가져와 궤적 적분과 함수 시그니처를 명확히 관리한다.
# 삼각 함수 호출이 빈번한 DWA 루프에서 이러한 기본 라이브러리는 수치적 안정성을 보장한다.
import math
from typing import List, Optional, Tuple

# 비용맵 생성과 ROS 통신에 필요한 외부 라이브러리를 불러와 센서 데이터를 처리하고 메시지를 교환한다.
# OpenCV의 distanceTransform과 NumPy 벡터화 연산은 실시간 비용맵 업데이트 속도를 좌우한다.
import cv2
import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformListener


# 각도를 -pi ~ pi 범위로 정규화하여 연속 회전 연산에서 누적 오차가 커지지 않도록 한다.
# 헤딩 비용이나 차선 각도 비교 시 이 범위를 유지해야 비용 함수가 급격히 튀지 않는다.
def normalize_angle(angle: float) -> float:
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


# 지도 좌표계에서 측정된 좌표를 로봇(base_link) 좌표계로 변환해 로컬 계획 알고리즘이 활용할 수 있게 한다.
# 전역 경로를 로컬 프레임으로 내려야 곡률 계산이나 궤적 평가가 간단한 2D 평면 연산으로 정리된다.
def transform_point_base_from_map(x_m: float, y_m: float, tf_map_base) -> Tuple[float, float]:
    tx = tf_map_base.transform.translation.x
    ty = tf_map_base.transform.translation.y
    q = tf_map_base.transform.rotation
    qw, qx, qy, qz = q.w, q.x, q.y, q.z
    yaw = math.atan2(2.0 * (qw * qz + qx * qy), 1 - 2 * (qy * qy + qz * qz))

    dx = x_m - tx
    dy = y_m - ty

    cos_yaw = math.cos(-yaw)
    sin_yaw = math.sin(-yaw)
    x_b = cos_yaw * dx - sin_yaw * dy
    y_b = sin_yaw * dx + cos_yaw * dy
    return x_b, y_b


# 로봇 좌표계에서 표현된 점을 지도 좌표계로 변환하여 OccupancyGrid 퍼블리시 및 경로 시각화에 사용한다.
# 맵 프레임으로 투영된 좌표는 RViz 시각화나 전역 비용맵과의 융합에 즉시 활용할 수 있다.
def transform_point_map_from_base(x_b: float, y_b: float, tf_map_base) -> Tuple[float, float]:
    tx = tf_map_base.transform.translation.x
    ty = tf_map_base.transform.translation.y
    q = tf_map_base.transform.rotation
    qw, qx, qy, qz = q.w, q.x, q.y, q.z
    yaw = math.atan2(2.0 * (qw * qz + qx * qy), 1 - 2 * (qy * qy + qz * qz))

    x_m = tx + (math.cos(yaw) * x_b - math.sin(yaw) * y_b)
    y_m = ty + (math.sin(yaw) * x_b + math.cos(yaw) * y_b)
    return x_m, y_m


# 임의의 변환 관계를 받아 점을 변환해 라이다 프레임 등 다양한 센서 프레임을 일관되게 처리한다.
# 공통 보조 함수를 두어 좌표 변환 로직이 중복되지 않도록 하고 테스트도 집중시킨다.
def apply_transform(x: float, y: float, transform) -> Tuple[float, float]:
    tx = transform.translation.x
    ty = transform.translation.y
    q = transform.rotation
    qw, qx, qy, qz = q.w, q.x, q.y, q.z
    yaw = math.atan2(2.0 * (qw * qz + qx * qy), 1 - 2 * (qy * qy + qz * qz))
    x_out = tx + (math.cos(yaw) * x - math.sin(yaw) * y)
    y_out = ty + (math.sin(yaw) * x + math.cos(yaw) * y)
    return x_out, y_out


# 로컬 비용맵을 구성하고 DWA 기반 속도 명령을 계산해 퍼블리시하는 ROS2 노드를 정의한다.
# 센서 융합과 궤적 평가를 한 노드에서 처리해 전역 경로 계획과 저수준 제어 사이의 연결 고리 역할을 한다.
class LocalDwaNode(Node):
    def __init__(self):
        super().__init__('local_dwa_node')

        # 토픽과 좌표계 관련 파라미터를 선언해 런치 파일/설정에 따른 통신 경로를 구성한다.
        # 네임스페이스나 프레임 명칭이 다른 차량 프로젝트에도 쉽게 적용될 수 있도록 모두 외부화한다.
        self.declare_parameter('global_path_topic', '/global_path')
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('local_path_topic', '/local_path')
        self.declare_parameter('costmap_topic', '/local_costmap')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('global_map_topic', '/map')
        self.declare_parameter('publish_cmd_vel', False)
        self.declare_parameter('cmd_topic', '/cmd_vel')

        # 로컬 비용맵의 공간 크기와 해상도, 장애물 확장 정도를 정의해 지도 표현 방식을 결정한다.
        # 슬랏 크기를 조정해 센서 노이즈를 필터링하거나 급격한 회복을 가능하게 하는 등 성능 튜닝의 핵심이다.
        self.declare_parameter('costmap_size', 12.0)  # meters (square window)
        self.declare_parameter('costmap_resolution', 0.1)  # meters/cell
        self.declare_parameter('max_obstacle_range', 10.0)
        self.declare_parameter('inflation_radius', 0.5)

        # 로봇의 충돌 반경을 정의하여 여유 거리 계산시 안전 마진을 부여한다.
        # 실제 차량이 직사각형이라도 원 추정 반경을 쓰면 계산이 단순해지며 여유 확보가 용이하다.
        self.declare_parameter('robot_radius', 0.4)

        # 시뮬레이션 시간과 샘플 개수를 설정해 후보 궤적의 탐색 해상도를 조절한다.
        # 긴 sim_time은 먼 미래를 예측하지만 환경 변화에 둔감해질 수 있어 트랙 특성에 맞춰 조정해야 한다.
        self.declare_parameter('sim_time', 2.0)
        self.declare_parameter('sim_dt', 0.1)
        self.declare_parameter('vx_samples', 6)
        self.declare_parameter('omega_samples', 11)

        # 선속도/각속도 및 가속도 한계를 지정해 로봇 동역학을 만족하도록 한다.
        # 이 범위를 벗어난 후보는 애초에 샘플링하지 않으므로 실제 하드웨어 한계와 동일하게 맞추는 것이 좋다.
        self.declare_parameter('max_speed', 4.0)
        self.declare_parameter('min_speed', 0.0)
        self.declare_parameter('max_accel', 4.0)
        self.declare_parameter('max_ang_speed', 4.0)
        self.declare_parameter('max_ang_accel', 6.0)

        # 헤딩, 속도, 장애물 여유에 대한 비용 가중치를 설정해 주행 성향을 조정한다.
        # 튜닝 시 각 가중치가 커질수록 해당 항이 전체 비용에 미치는 영향력이 기하급수적으로 커진다.
        self.declare_parameter('heading_weight', 1.0)
        self.declare_parameter('velocity_weight', 1.0)
        self.declare_parameter('clearance_weight', 1.0)

        # 전역 경로에서 얼마만큼 앞을 목표로 삼을지 거리 기반 루ック어헤드 값을 설정한다.
        # lookahead 값은 목표점을 멀리 잡을수록 가속이 쉬워지지만 곡선 대응이 늦어지므로 주행 전략에 맞춰 튜닝한다.
        self.declare_parameter('goal_lookahead_dist', 3.0)

        # 선언된 파라미터를 서버에서 읽어와 내부 변수에 저장하고 이후 로직에 활용한다.
        # 파라미터 타입이 명확히 유지되도록 get_parameter_value()를 사용해 문자열/실수/불리언을 구분한다.
        self.global_path_topic = self.get_parameter('global_path_topic').get_parameter_value().string_value
        self.odom_topic = self.get_parameter('odom_topic').get_parameter_value().string_value
        self.scan_topic = self.get_parameter('scan_topic').get_parameter_value().string_value
        self.local_path_topic = self.get_parameter('local_path_topic').get_parameter_value().string_value
        self.costmap_topic = self.get_parameter('costmap_topic').get_parameter_value().string_value
        self.base_frame = self.get_parameter('base_frame').get_parameter_value().string_value
        self.map_frame = self.get_parameter('map_frame').get_parameter_value().string_value
        self.global_map_topic = self.get_parameter('global_map_topic').get_parameter_value().string_value
        self.publish_cmd_vel = bool(self.get_parameter('publish_cmd_vel').value)
        self.cmd_topic = self.get_parameter('cmd_topic').get_parameter_value().string_value

        self.costmap_size = float(self.get_parameter('costmap_size').value)
        self.resolution = float(self.get_parameter('costmap_resolution').value)
        self.max_obstacle_range = float(self.get_parameter('max_obstacle_range').value)
        self.inflation_radius = float(self.get_parameter('inflation_radius').value)
        self.robot_radius = float(self.get_parameter('robot_radius').value)

        self.sim_time = float(self.get_parameter('sim_time').value)
        self.sim_dt = float(self.get_parameter('sim_dt').value)
        self.vx_samples = int(self.get_parameter('vx_samples').value)
        self.omega_samples = int(self.get_parameter('omega_samples').value)
        self.max_speed = float(self.get_parameter('max_speed').value)
        self.min_speed = float(self.get_parameter('min_speed').value)
        self.max_accel = float(self.get_parameter('max_accel').value)
        self.max_ang_speed = float(self.get_parameter('max_ang_speed').value)
        self.max_ang_accel = float(self.get_parameter('max_ang_accel').value)
        self.heading_weight = float(self.get_parameter('heading_weight').value)
        self.velocity_weight = float(self.get_parameter('velocity_weight').value)
        self.clearance_weight = float(self.get_parameter('clearance_weight').value)
        self.goal_lookahead_dist = float(self.get_parameter('goal_lookahead_dist').value)

        # 비용맵 격자 크기를 계산하여 로봇 주변을 중심으로 대칭적인 정사각형 맵을 준비한다.
        # round를 통해 해상도 오류를 줄이고, 최소 크기를 3 이상으로 제한해 distanceTransform이 항상 동작하게 한다.
        grid_dim = max(3, int(round(self.costmap_size / self.resolution)))
        if grid_dim % 2 == 0:
            grid_dim += 1  # keep odd to maintain symmetry about origin
        self.grid_size = grid_dim
        self.half_extent = (self.grid_size * self.resolution) / 2.0

        # 점유 상태, 장애물 거리, 최신 센서 데이터를 저장할 내부 버퍼를 초기화한다.
        self.occupancy = np.zeros((self.grid_size, self.grid_size), dtype=np.int16)
        self.distance_map = np.ones((self.grid_size, self.grid_size), dtype=np.float32) * self.half_extent
        self.global_path: Optional[Path] = None
        self.latest_odom: Optional[Odometry] = None
        self.latest_scan: Optional[LaserScan] = None
        self.latest_map: Optional[OccupancyGrid] = None
        self.map_info = None
        self.map_array: Optional[np.ndarray] = None
        self.grid_origin_x = 0.0
        self.grid_origin_y = 0.0

        # TF 버퍼와 리스너를 준비하여 다양한 센서 프레임 간 좌표 변환을 수행할 수 있게 한다.
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self, spin_thread=True)

        # 전역 경로, 오도메트리, 라이다 토픽을 구독하여 계획에 필요한 입력을 수신한다.
        qos_path = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(Path, self.global_path_topic, self._path_cb, qos_path)
        self.create_subscription(Odometry, self.odom_topic, self._odom_cb, 10)
        self.create_subscription(
            LaserScan,
            self.scan_topic,
            self._scan_cb,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT),
        )
        self.create_subscription(
            OccupancyGrid,
            self.global_map_topic,
            self._map_cb,
            QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE),
        )

        # 로컬 경로와 비용맵, 속도 명령을 퍼블리시해 다른 노드가 결과를 활용하도록 한다.
        self.cmd_pub = None
        if self.publish_cmd_vel:
            self.cmd_pub = self.create_publisher(Twist, self.cmd_topic, 10)
        self.path_pub = self.create_publisher(Path, self.local_path_topic, 10)
        self.costmap_pub = self.create_publisher(OccupancyGrid, self.costmap_topic, 1)

        # 지정된 주기로 타이머를 설정하여 센서 처리와 DWA 평가를 반복 수행한다.
        self.timer = self.create_timer(self.sim_dt, self._on_timer)

        self._last_no_traj_warn = None

        self.get_logger().info(
            f'Local DWA ready (costmap_size={self.costmap_size}m, resolution={self.resolution}m, '
            f'vx_samples={self.vx_samples}, omega_samples={self.omega_samples})'
        )

    # ------------------------------------------------------------------ Callbacks
    # 전역 경로 메시지를 저장하여 로컬 목표 선택 및 추종에 참조한다.
    def _path_cb(self, msg: Path):
        self.global_path = msg

    # 최신 오도메트리 정보를 갱신해 로봇 속도/자세를 추적한다.
    def _odom_cb(self, msg: Odometry):
        self.latest_odom = msg

    # 가장 최근 라이다 스캔을 보관하여 비용맵 갱신 시 사용한다.
    def _scan_cb(self, msg: LaserScan):
        self.latest_scan = msg

    # 글로벌 OccupancyGrid 지도를 저장해 로컬 비용맵 초기 상태로 활용한다.
    def _map_cb(self, msg: OccupancyGrid):
        self.latest_map = msg
        self.map_info = msg.info
        if msg.data:
            self.map_array = np.array(msg.data, dtype=np.int16).reshape(msg.info.height, msg.info.width)
        else:
            self.map_array = None

    # ------------------------------------------------------------------ Timer logic
    # 타이머 콜백에서 센서 유효성 확인 후 비용맵 갱신, 목표 선정, 속도 명령 계산을 일괄 수행한다.
    def _on_timer(self):
        if self.latest_scan is None or self.latest_odom is None or self.global_path is None:
            return
        if not self.global_path.poses:
            return

        try:
            tf_map_base = self.tf_buffer.lookup_transform(
                self.map_frame,
                self.base_frame,
                rclpy.time.Time(),
                timeout=Duration(seconds=0.05)
            )
        except Exception as exc:
            self.get_logger().warn(f'TF lookup failed: {exc}')
            return

        self._update_costmap(self.latest_scan, tf_map_base)
        self._publish_costmap()

        segment = self._extract_path_segment(tf_map_base)
        if segment is None:
            self._warn_no_traj('Goal point unavailable for local planning.')
            self._publish_stop()
            return
        segment_indices, goal_point_map, goal_point_base, path_clear = segment
        if goal_point_map is None or goal_point_base is None:
            self._warn_no_traj('Goal point unavailable for local planning.')
            self._publish_stop()
            return

        if path_clear:
            self._publish_global_segment(segment_indices)
            self._follow_global_path(goal_point_base)
            return

        best = self._evaluate_velocities(goal_point_base, tf_map_base)
        if best is None:
            self._warn_no_traj('No valid DWA trajectory found; issuing stop.')
            self._publish_stop()
            return

        traj_points, best_v, best_w = best
        self._publish_path(traj_points, tf_map_base)
        self._publish_cmd(best_v, best_w)

    # ------------------------------------------------------------------ Costmap construction
    # 라이다 스캔을 따라 자유 공간을 채우고 장애물 셀을 표기해 DWA 평가에 사용할 비용맵을 생성한다.
    def _update_costmap(self, scan: LaserScan, tf_map_base):
        self.occupancy.fill(-1)

        base_x = tf_map_base.transform.translation.x
        base_y = tf_map_base.transform.translation.y
        self.grid_origin_x = base_x - self.half_extent
        self.grid_origin_y = base_y - self.half_extent

        if self.map_array is not None and self.map_info is not None:
            for r in range(self.grid_size):
                y_center = self.grid_origin_y + (r + 0.5) * self.resolution
                for c in range(self.grid_size):
                    x_center = self.grid_origin_x + (c + 0.5) * self.resolution
                    occ = self._lookup_static_map(x_center, y_center)
                    if occ is None:
                        continue
                    if occ >= 0:
                        self.occupancy[r, c] = occ

        tf_base_scan = None
        if scan.header.frame_id and scan.header.frame_id != self.base_frame:
            try:
                tf_base_scan = self.tf_buffer.lookup_transform(
                    self.base_frame,
                    scan.header.frame_id,
                    rclpy.time.Time(),
                    timeout=Duration(seconds=0.05)
                )
            except Exception as exc:
                self.get_logger().warn(
                    f'Failed to lookup transform {self.base_frame} <- {scan.header.frame_id}: {exc}'
                )

        def to_base(x_local: float, y_local: float) -> Tuple[float, float]:
            if tf_base_scan is None:
                return x_local, y_local
            return apply_transform(x_local, y_local, tf_base_scan.transform)

        # 레이 샘플링 간격은 격자 절반 이하로 유지해 빈틈없이 자유 공간을 채우되 계산 부하를 제한한다.
        step = max(self.resolution * 0.5, 0.02)

        # 스캔 각도를 초기화해 각 빔을 map 프레임으로 전개하며 점유 상태를 업데이트한다.
        angle = scan.angle_min
        for rng in scan.ranges:
            if math.isinf(rng) or math.isnan(rng):
                angle += scan.angle_increment
                continue
            if rng <= 0.0:
                angle += scan.angle_increment
                continue

            # 장애물까지의 거리를 제한해 비용맵 범위를 넘어가는 불필요한 샘플링을 줄인다.
            limited_range = min(rng, self.max_obstacle_range, self.costmap_size)
            ray_dist = 0.0
            while ray_dist < limited_range:
                x_ray = ray_dist * math.cos(angle)
                y_ray = ray_dist * math.sin(angle)
                x_b_ray, y_b_ray = to_base(x_ray, y_ray)
                x_m_ray, y_m_ray = transform_point_map_from_base(x_b_ray, y_b_ray, tf_map_base)
                idx_ray = self._map_to_grid(x_m_ray, y_m_ray)
                if idx_ray is not None:
                    r_ray, c_ray = idx_ray
                    if self.occupancy[r_ray, c_ray] < 100:
                        self.occupancy[r_ray, c_ray] = 0
                ray_dist += step

            x = min(rng, self.max_obstacle_range) * math.cos(angle)
            y = min(rng, self.max_obstacle_range) * math.sin(angle)
            angle += scan.angle_increment

            x_b, y_b = to_base(x, y)
            x_m, y_m = transform_point_map_from_base(x_b, y_b, tf_map_base)
            idx = self._map_to_grid(x_m, y_m)
            if idx is None:
                continue
            row, col = idx
            self.occupancy[row, col] = 100

        # 점유 결과를 0/1 마스크로 변환해 distanceTransform 입력 형태에 맞춘다.
        obstacle_mask = (self.occupancy >= 100).astype(np.uint8)
        free_mask = np.ones_like(obstacle_mask, dtype=np.uint8)
        free_mask[obstacle_mask == 1] = 0

        if np.count_nonzero(obstacle_mask) > 0:
            distance = cv2.distanceTransform(free_mask, cv2.DIST_L2, 3)
            self.distance_map = distance * self.resolution
        else:
            # 장애물이 전혀 없으면 모든 셀을 최대 반경 값으로 채워 충돌 비용이 0에 가깝게 유지된다.
            self.distance_map.fill(self.half_extent)

    # 계산된 점유/거리 정보를 OccupancyGrid 메시지로 변환해 ROS 네트워크에 배포한다.
    def _publish_costmap(self):
        msg = OccupancyGrid()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.map_frame
        msg.info.resolution = self.resolution
        msg.info.width = self.grid_size
        msg.info.height = self.grid_size
        msg.info.origin.position.x = float(self.grid_origin_x)
        msg.info.origin.position.y = float(self.grid_origin_y)
        msg.info.origin.position.z = 0.0
        msg.info.origin.orientation.w = 1.0
        msg.data = [int(v) for v in self.occupancy.flatten()]
        self.costmap_pub.publish(msg)

    # ------------------------------------------------------------------ Goal selection
    # 전역 경로에서 루크어헤드 구간을 추출하고 장애물 존재 여부를 동시에 평가한다.
    def _extract_path_segment(
        self, tf_map_base
    ) -> Optional[Tuple[List[int], Optional[Tuple[float, float]], Optional[Tuple[float, float]], bool]]:
        if self.global_path is None or not self.global_path.poses:
            return None
        base_x = tf_map_base.transform.translation.x
        base_y = tf_map_base.transform.translation.y

        poses = self.global_path.poses
        closest_idx = 0
        closest_dist = float('inf')
        for i, pose in enumerate(poses):
            dx = pose.pose.position.x - base_x
            dy = pose.pose.position.y - base_y
            dist = math.hypot(dx, dy)
            if dist < closest_dist:
                closest_dist = dist
                closest_idx = i

        segment_indices: List[int] = [closest_idx]
        segment_clear = True
        lookahead = self.goal_lookahead_dist
        accum = 0.0
        prev = poses[closest_idx].pose.position
        # 시작점도 장애물 여부를 확인해 둔다.
        start_grid = self._map_to_grid(prev.x, prev.y)
        if start_grid is not None:
            row, col = start_grid
            if self.occupancy[row, col] >= 100 or self.distance_map[row, col] <= self.robot_radius:
                segment_clear = False

        for offset in range(1, len(poses)):
            idx = min(closest_idx + offset, len(poses) - 1)
            curr = poses[idx].pose.position
            ds = math.hypot(curr.x - prev.x, curr.y - prev.y)
            accum += ds
            prev = curr
            if segment_indices[-1] != idx:
                segment_indices.append(idx)

            grid = self._map_to_grid(curr.x, curr.y)
            if grid is not None:
                row, col = grid
                if self.occupancy[row, col] >= 100 or self.distance_map[row, col] <= self.robot_radius:
                    segment_clear = False

            if accum >= lookahead or idx == len(poses) - 1:
                break

        if not segment_indices:
            return None

        goal_idx = segment_indices[-1]
        goal_pose = poses[goal_idx].pose
        goal_point_map = (goal_pose.position.x, goal_pose.position.y)
        goal_point_base = transform_point_base_from_map(goal_point_map[0], goal_point_map[1], tf_map_base)

        return segment_indices, goal_point_map, goal_point_base, segment_clear

    # ------------------------------------------------------------------ DWA evaluation
    # 현재 속도를 중심으로 가능한 속도/각속도 조합을 샘플링하고 비용 함수로 최적해를 탐색한다.
    def _evaluate_velocities(self, goal_point_base: Tuple[float, float], tf_map_base):
        current_v = self.latest_odom.twist.twist.linear.x
        current_w = self.latest_odom.twist.twist.angular.z

        # 현재 속도를 기준으로 가속도 제한 내에서 허용 가능한 최소/최대 속도를 계산한다.
        v_min = max(self.min_speed, current_v - self.max_accel * self.sim_dt)
        v_max = min(self.max_speed, current_v + self.max_accel * self.sim_dt)
        w_min = max(-self.max_ang_speed, current_w - self.max_ang_accel * self.sim_dt)
        w_max = min(self.max_ang_speed, current_w + self.max_ang_accel * self.sim_dt)

        if self.vx_samples <= 1:
            velocities = [max(self.min_speed, min(self.max_speed, current_v))]
        else:
            # linspace로 균일 간격 후보를 생성해 탐색 공간을 고르게 커버한다.
            velocities = np.linspace(v_min, v_max, self.vx_samples)

        if self.omega_samples <= 1:
            omegas = [max(-self.max_ang_speed, min(self.max_ang_speed, current_w))]
        else:
            # 선속도와 동일하게 각속도도 균등 분할해 조향 후보를 만든다.
            omegas = np.linspace(w_min, w_max, self.omega_samples)

        best_cost = float('inf')
        best_solution = None

        # 모든 조합을 브루트포스로 평가하되 비용이 무한대인 경우 즉시 배제한다.
        for v in velocities:
            for w in omegas:
                traj = self._simulate_trajectory(v, w)
                if not traj:
                    continue
                heading_cost, velocity_cost, clearance_cost = self._score_trajectory(
                    traj, v, goal_point_base, tf_map_base
                )
                if math.isinf(heading_cost) or math.isinf(clearance_cost):
                    continue
                total_cost = (
                    self.heading_weight * heading_cost +
                    self.velocity_weight * velocity_cost +
                    self.clearance_weight * clearance_cost
                )
                if total_cost < best_cost:
                    best_cost = total_cost
                    best_solution = (traj, float(v), float(w))

        return best_solution

    # 후보 속도에 대해 일정 시간 동안 로봇 궤적을 적분해 공간상의 경로를 예측한다.
    def _simulate_trajectory(self, v: float, w: float) -> List[Tuple[float, float, float]]:
        x = 0.0
        y = 0.0
        yaw = 0.0
        traj: List[Tuple[float, float, float]] = []

        steps = max(1, int(self.sim_time / self.sim_dt))
        for _ in range(steps):
            # DWA는 각 샘플 주기 동안 속도가 일정하다고 가정하고 평면 운동 방정식을 적분한다.
            x += v * math.cos(yaw) * self.sim_dt
            y += v * math.sin(yaw) * self.sim_dt
            yaw += w * self.sim_dt
            traj.append((x, y, yaw))
        return traj

    # 시뮬레이션된 궤적의 헤딩 정합, 속도 선호도, 장애물과의 거리 기반 비용을 계산한다.
    def _score_trajectory(
        self,
        traj: List[Tuple[float, float, float]],
        v: float,
        goal_point_base: Tuple[float, float],
        tf_map_base,
    ) -> Tuple[float, float, float]:
        min_clearance = float('inf')

        for (x, y, _) in traj:
            # 궤적의 모든 점을 비용맵에 투영해 단 하나라도 충돌 셀이 있는지 확인한다.
            x_m, y_m = transform_point_map_from_base(x, y, tf_map_base)
            idx = self._map_to_grid(x_m, y_m)
            if idx is None:
                return float('inf'), float('inf'), float('inf')
            row, col = idx
            occ = self.occupancy[row, col]
            if occ >= 100:
                return float('inf'), float('inf'), float('inf')
            clearance = self.distance_map[row, col]
            min_clearance = min(min_clearance, clearance)

        if min_clearance < self.robot_radius:
            return float('inf'), float('inf'), float('inf')

        final_x, final_y, final_yaw = traj[-1]
        goal_angle = math.atan2(goal_point_base[1], goal_point_base[0])
        heading_error = abs(normalize_angle(goal_angle - final_yaw))

        # 속도 비용은 최대 속도와의 차이를 정규화하고, 여유 비용은 장애물까지 남은 거리의 역수를 사용한다.
        velocity_cost = (self.max_speed - v) / max(self.max_speed, 1e-3)
        clearance_cost = 1.0 / max(min_clearance - self.robot_radius, 0.01)

        return heading_error, velocity_cost, clearance_cost

    # ------------------------------------------------------------------ Publications
    # 전역 경로 일부를 그대로 Path 메시지로 내보내 글로벌 경로 추종을 지원한다.
    def _publish_global_segment(self, segment_indices: List[int]):
        if not self.global_path or not self.global_path.poses:
            return
        path_msg = Path()
        stamp = self.get_clock().now().to_msg()
        path_msg.header.stamp = stamp
        path_msg.header.frame_id = self.map_frame

        for idx in segment_indices:
            if idx < 0 or idx >= len(self.global_path.poses):
                continue
            pose = PoseStamped()
            pose.header = path_msg.header
            pose.pose = self.global_path.poses[idx].pose
            path_msg.poses.append(pose)

        if not path_msg.poses:
            pose = PoseStamped()
            pose.header = path_msg.header
            pose.pose.orientation.w = 1.0
            path_msg.poses.append(pose)

        self.path_pub.publish(path_msg)

    # 글로벌 경로 목표를 향해 간단한 곡률 기반 속도 명령을 생성한다.
    def _follow_global_path(self, goal_point_base: Tuple[float, float]):
        x_b, y_b = goal_point_base
        distance = math.hypot(x_b, y_b)
        heading_error = math.atan2(y_b, x_b)

        if distance < 0.05:
            self._publish_stop()
            return

        v_cmd = min(self.max_speed, distance)
        v_cmd = max(0.0, v_cmd)

        lookahead = max(distance, 0.1)
        w_cmd = 2.0 * v_cmd * math.sin(heading_error) / lookahead
        w_cmd = max(-self.max_ang_speed, min(self.max_ang_speed, w_cmd))

        self._publish_cmd(v_cmd, w_cmd)

    # 유효한 퍼블리셔가 있을 때 최종 선택된 선속도와 각속도를 Twist 메시지로 보낸다.
    def _publish_cmd(self, v: float, w: float):
        if self.cmd_pub is None:
            return
        cmd = Twist()
        cmd.linear.x = v
        cmd.angular.z = w
        self.cmd_pub.publish(cmd)

    # 최적 궤적이 없을 경우 안전하게 정지하도록 0 속도를 발행한다.
    def _publish_stop(self):
        self._publish_cmd(0.0, 0.0)

    # 평가된 궤적을 Path 메시지로 변환해 시각화 및 후속 처리가 가능하도록 한다.
    def _publish_path(self, traj: List[Tuple[float, float, float]], tf_map_base):
        path_msg = Path()
        stamp = self.get_clock().now().to_msg()
        path_msg.header.stamp = stamp
        path_msg.header.frame_id = self.map_frame

        if not traj:
            pose = PoseStamped()
            pose.header = path_msg.header
            pose.pose.orientation.w = 1.0
            path_msg.poses.append(pose)
            self.path_pub.publish(path_msg)
            return

        # base_link 궤적을 map 좌표로 변환하면서 인접 점을 이용해 진행 방향을 추정한다.
        for i, (x_b, y_b, yaw_b) in enumerate(traj):
            pose = PoseStamped()
            pose.header = path_msg.header
            x_m, y_m = transform_point_map_from_base(x_b, y_b, tf_map_base)
            pose.pose.position.x = float(x_m)
            pose.pose.position.y = float(y_m)
            pose.pose.position.z = 0.0

            if i < len(traj) - 1:
                next_x, next_y, _ = traj[i + 1]
                yaw = math.atan2(next_y - y_b, next_x - x_b)
            else:
                yaw = yaw_b

            pose.pose.orientation.z = math.sin(yaw / 2.0)
            pose.pose.orientation.w = math.cos(yaw / 2.0)
            path_msg.poses.append(pose)

        self.path_pub.publish(path_msg)

    # ------------------------------------------------------------------ Helpers
    # 지도 좌표를 로컬 비용맵의 행/열 인덱스로 변환해 셀 접근을 가능하게 한다.
    # 격자 범위를 벗어난 경우 즉시 None을 반환해 상위 로직이 충돌로 간주하도록 한다.
    def _map_to_grid(self, x: float, y: float) -> Optional[Tuple[int, int]]:
        rel_x = x - self.grid_origin_x
        rel_y = y - self.grid_origin_y
        if rel_x < 0.0 or rel_y < 0.0:
            return None
        if rel_x >= self.costmap_size or rel_y >= self.costmap_size:
            return None
        col = int(rel_x / self.resolution)
        row = int(rel_y / self.resolution)
        if 0 <= row < self.grid_size and 0 <= col < self.grid_size:
            return row, col
        return None

    # 글로벌 지도 좌표에서 점유값을 조회해 로컬 비용맵 초기화에 활용한다.
    def _lookup_static_map(self, x: float, y: float) -> Optional[int]:
        if self.map_array is None or self.map_info is None:
            return None
        origin = self.map_info.origin
        res = self.map_info.resolution
        q = origin.orientation
        qw, qx, qy, qz = q.w, q.x, q.y, q.z
        yaw = math.atan2(2.0 * (qw * qz + qx * qy), 1 - 2 * (qy * qy + qz * qz))
        dx = x - origin.position.x
        dy = y - origin.position.y
        cos_yaw = math.cos(-yaw)
        sin_yaw = math.sin(-yaw)
        mx = cos_yaw * dx - sin_yaw * dy
        my = sin_yaw * dx + cos_yaw * dy
        if mx < 0.0 or my < 0.0:
            return None
        col = int(mx / res)
        row = int(my / res)
        if 0 <= row < self.map_info.height and 0 <= col < self.map_info.width:
            return int(self.map_array[row, col])
        return None

    # 동일 경고가 짧은 시간에 반복되지 않도록 타임스탬프를 기록하며 출력을 제한한다.
    # 1초 미만 간격으로 재호출되면 조용히 무시해 디버깅 로그의 가독성을 확보한다.
    def _warn_no_traj(self, message: str):
        now = self.get_clock().now()
        if (
            self._last_no_traj_warn is None
            or (now - self._last_no_traj_warn).nanoseconds > int(1e9)
        ):
            self.get_logger().warn(message)
            self._last_no_traj_warn = now


# 메인 진입점에서 노드를 생성하고 스핀하며 KeyboardInterrupt 시 정리 절차를 수행한다.
def main(args=None):
    rclpy.init(args=args)
    node = LocalDwaNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Local DWA planner stopped by user.')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
