#!/usr/bin/env python3
import math
from typing import List, Tuple

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from nav_msgs.msg import Path
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseStamped
from tf2_ros import Buffer, TransformListener


def transform_point_map_from_base(x_b, y_b, tf):
    # tf: geometry_msgs/TransformStamped (map->base_link) 메시지입니다.
    tx = tf.transform.translation.x
    ty = tf.transform.translation.y
    q = tf.transform.rotation
    # 쿼터니언에서 요 각도를 추출합니다.
    # tf_transformations를 추가 의존성 없이 사용하기 위해 간단한 보조 계산을 수행합니다.
    # 요 각도만 추출할 수 있도록 회전 행렬 요소를 계산합니다.
    import numpy as np
    qw, qx, qy, qz = q.w, q.x, q.y, q.z
    # 요 각
    yaw = math.atan2(2.0*(qw*qz + qx*qy), 1 - 2*(qy*qy + qz*qz))
    # 회전과 평행이동을 적용합니다.
    x_m = tx + (math.cos(yaw)*x_b - math.sin(yaw)*y_b)
    y_m = ty + (math.sin(yaw)*x_b + math.cos(yaw)*y_b)
    return x_m, y_m, yaw


class LocalAvoidanceNode(Node):
    def __init__(self):
        super().__init__('local_avoidance_node')

        # 파라미터 설정
        self.declare_parameter('global_path_topic', '/global_path')
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('local_horizon', 8.0)  # 미터 단위
        self.declare_parameter('path_resolution', 0.2)  # 미터 단위
        self.declare_parameter('lateral_offsets', [0.0, 0.4, -0.4])  # 미터 단위
        self.declare_parameter('lookahead_distance', 2.0)  # 미터 단위
        self.declare_parameter('safety_radius', 0.4)  # 미터 단위
        self.declare_parameter('output_topic', '/local_path')
        self.declare_parameter('base_frame', '/base_link')
        self.declare_parameter('map_frame', 'map')

        # 파라미터를 읽어옵니다.
        self.global_path_topic = self.get_parameter('global_path_topic').get_parameter_value().string_value
        self.scan_topic = self.get_parameter('scan_topic').get_parameter_value().string_value
        self.local_horizon = float(self.get_parameter('local_horizon').value)
        self.ds = float(self.get_parameter('path_resolution').value)
        self.lateral_offsets = [float(v) for v in self.get_parameter('lateral_offsets').value]
        self.safety_radius = float(self.get_parameter('safety_radius').value)
        self.lookahead_distance = float(self.get_parameter('lookahead_distance').value)
        self.output_topic = self.get_parameter('output_topic').get_parameter_value().string_value
        self.base_frame = self.get_parameter('base_frame').get_parameter_value().string_value
        self.map_frame = self.get_parameter('map_frame').get_parameter_value().string_value

        # TF 버퍼를 초기화합니다.
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # 구독 설정
        self.global_path: Path = None
        self.sub_path = self.create_subscription(Path, self.global_path_topic, self._path_cb, 10)
        qos = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.sub_scan = self.create_subscription(LaserScan, self.scan_topic, self._scan_cb, qos)

        # 퍼블리셔 설정
        self.pub = self.create_publisher(Path, self.output_topic, 10)

        # 상태 변수
        self.latest_scan: LaserScan = None

        # 데이터가 준비되면 일정 주기로 퍼블리시하도록 타이머를 설정합니다.
        self.timer = self.create_timer(0.05, self._on_timer)  # 20 Hz 주기

    def _path_cb(self, msg: Path):
        self.global_path = msg

    def _scan_cb(self, msg: LaserScan):
        self.latest_scan = msg

    def _get_tf(self):
        try:
            return self.tf_buffer.lookup_transform(self.map_frame, self.base_frame, rclpy.time.Time())
        except Exception as e:
            self.get_logger().warn(f'TF lookup failed: {e}')
            return None

    def _closest_path_index(self, x, y) -> int:
        if self.global_path is None or not self.global_path.poses:
            return -1
        best_i = -1
        best_d2 = 1e18
        for i, ps in enumerate(self.global_path.poses):
            dx = ps.pose.position.x - x
            dy = ps.pose.position.y - y
            d2 = dx*dx + dy*dy
            if d2 < best_d2:
                best_d2 = d2
                best_i = i
        return best_i
    
    def _get_global_path_points(self, current_x, current_y, current_yaw) -> List[Tuple[float, float]]:
        """로봇 진행 방향으로 일정한 길이만큼 전역 경로 점을 추출합니다."""
        if self.global_path is None or not self.global_path.poses:
            return []
        
        # 전역 경로에서 현재 위치와 가장 가까운 점을 찾습니다.
        closest_idx = self._closest_path_index(current_x, current_y)
        if closest_idx < 0:
            return []
        
        # 전역 경로에서 로봇 진행 방향을 결정합니다.
        forward_idx, forward_direction = self._find_forward_direction(closest_idx, current_x, current_y, current_yaw)
        
        global_points = []
        cumulative_dist = 0.0
        prev_x, prev_y = current_x, current_y
        
        # 로봇의 진행 방향으로 로컬 가시 거리 이내의 점들을 모읍니다.
        if forward_direction > 0:
            # 정방향(인덱스 증가)
            for i in range(forward_idx, len(self.global_path.poses)):
                pose = self.global_path.poses[i]
                x = pose.pose.position.x
                y = pose.pose.position.y
                
                dist = math.hypot(x - prev_x, y - prev_y)
                cumulative_dist += dist
                
                if cumulative_dist > self.local_horizon:
                    break
                    
                global_points.append((x, y))
                prev_x, prev_y = x, y
        else:
            # 역방향(인덱스 감소)
            for i in range(forward_idx, -1, -1):
                pose = self.global_path.poses[i]
                x = pose.pose.position.x
                y = pose.pose.position.y
                
                dist = math.hypot(x - prev_x, y - prev_y)
                cumulative_dist += dist
                
                if cumulative_dist > self.local_horizon:
                    break
                    
                global_points.append((x, y))
                prev_x, prev_y = x, y
            
        return global_points
    
    def _find_forward_direction(self, closest_idx, current_x, current_y, current_yaw) -> Tuple[int, int]:
        """전역 경로 중 로봇의 진행 방향과 정렬되는 쪽을 판단합니다."""
        if not self.global_path or not self.global_path.poses:
            return closest_idx, 1
        
        # 로봇의 전방 방향 벡터를 계산합니다.
        robot_forward_x = math.cos(current_yaw)
        robot_forward_y = math.sin(current_yaw)
        
        best_idx = closest_idx
        best_direction = 1
        best_alignment = -2.0  # 가능한 최악의 내적 값으로 초기화합니다.
        
        # 정방향(인덱스 증가)을 확인합니다.
        if closest_idx + 1 < len(self.global_path.poses):
            next_pose = self.global_path.poses[closest_idx + 1]
            path_vec_x = next_pose.pose.position.x - current_x
            path_vec_y = next_pose.pose.position.y - current_y
            path_len = math.hypot(path_vec_x, path_vec_y)
            
            if path_len > 1e-6:
                path_vec_x /= path_len
                path_vec_y /= path_len
                alignment = robot_forward_x * path_vec_x + robot_forward_y * path_vec_y
                
                if alignment > best_alignment:
                    best_alignment = alignment
                    best_idx = closest_idx
                    best_direction = 1
        
        # 역방향(인덱스 감소)을 확인합니다.
        if closest_idx - 1 >= 0:
            prev_pose = self.global_path.poses[closest_idx - 1]
            path_vec_x = prev_pose.pose.position.x - current_x
            path_vec_y = prev_pose.pose.position.y - current_y
            path_len = math.hypot(path_vec_x, path_vec_y)
            
            if path_len > 1e-6:
                path_vec_x /= path_len
                path_vec_y /= path_len
                alignment = robot_forward_x * path_vec_x + robot_forward_y * path_vec_y
                
                if alignment > best_alignment:
                    best_alignment = alignment
                    best_idx = closest_idx
                    best_direction = -1
        
        return best_idx, best_direction
    
    def _transform_point_base_from_map(self, x_m, y_m, tf) -> Tuple[float, float]:
        """맵 프레임 좌표를 base_link 프레임으로 변환합니다."""
        tx = tf.transform.translation.x
        ty = tf.transform.translation.y
        q = tf.transform.rotation
        qw, qx, qy, qz = q.w, q.x, q.y, q.z
        yaw = math.atan2(2.0*(qw*qz + qx*qy), 1 - 2*(qy*qy + qz*qz))
        
        # 차량 기준점으로 평행이동합니다.
        dx = x_m - tx
        dy = y_m - ty
        
        # base_link 프레임으로 회전합니다.
        cos_yaw = math.cos(-yaw)
        sin_yaw = math.sin(-yaw)
        x_b = cos_yaw * dx - sin_yaw * dy
        y_b = sin_yaw * dx + cos_yaw * dy
        
        return x_b, y_b
    
    def _has_obstacle_in_direction(self, x_b, y_b, scan: LaserScan) -> bool:
        """목표 지점 방향에 장애물이 있는지 확인합니다."""
        if abs(x_b) < 1e-3 and abs(y_b) < 1e-3:
            return False
        
        theta = math.atan2(y_b, x_b)
        if theta < scan.angle_min or theta > scan.angle_max:
            return False
        
        idx = int(round((theta - scan.angle_min) / scan.angle_increment))
        if idx < 0 or idx >= len(scan.ranges):
            return False
        
        rng = scan.ranges[idx]
        if math.isnan(rng) or math.isinf(rng):
            return False
        
        target_dist = math.hypot(x_b, y_b)
        return rng < (target_dist + self.safety_radius)

    def _is_point_safe(self, x_b, y_b, scan: LaserScan) -> bool:
        r = math.hypot(x_b, y_b)
        if r < 1e-3:
            return True
        theta = math.atan2(y_b, x_b)
        # 라이다 스캔 각도 범위로 제한합니다.
        if theta < scan.angle_min or theta > scan.angle_max:
            return True  # 가시 범위 밖은 과도한 제약을 피하기 위해 안전하다고 간주합니다.
        idx = int(round((theta - scan.angle_min) / scan.angle_increment))
        if idx < 0 or idx >= len(scan.ranges):
            return True
        rng = scan.ranges[idx]
        if math.isnan(rng) or math.isinf(rng):
            return True
        return (rng - r) > self.safety_radius

    def _build_local_path(self, tf_map_base, scan: LaserScan, current_x, current_y) -> List[Tuple[float, float]]:
        # 변환에서 현재 로봇의 요 각도를 추출합니다.
        q = tf_map_base.transform.rotation
        qw, qx, qy, qz = q.w, q.x, q.y, q.z
        current_yaw = math.atan2(2.0*(qw*qz + qx*qy), 1 - 2*(qy*qy + qz*qz))
        
        # 로봇 진행 방향으로 따라야 할 전역 경로 점을 가져옵니다.
        global_points = self._get_global_path_points(current_x, current_y, current_yaw)
        
        if not global_points:
            # 전역 경로가 없으면 직선 경로를 생성하는 대안을 사용합니다.
            return self._build_fallback_path(tf_map_base, scan)
        
        local_pts = []  # 맵 프레임 좌표
        
        # 전역 경로 점을 따라가되 필요한 경우 장애물을 회피합니다.
        for global_point in global_points:
            gx, gy = global_point
            
            # 장애물 확인을 위해 전역 점을 base_link 프레임으로 변환합니다.
            gx_b, gy_b = self._transform_point_base_from_map(gx, gy, tf_map_base)
            
            # 전역 점까지 직선 경로가 안전한지 확인합니다.
            if not self._has_obstacle_in_direction(gx_b, gy_b, scan):
                # 전역 경로를 그대로 따라갈 수 있습니다.
                local_pts.append((gx, gy))
            else:
                # 장애물이 감지되면 전역 점에서 측면 오프셋을 시도합니다.
                best_point = None
                
                # 전역 점 주변으로 측면 오프셋을 적용해 봅니다.
                for offset in self.lateral_offsets[1:]:  # 중앙은 이미 확인했으므로 건너뜁니다.
                    # 경로 방향에 수직한 오프셋 좌표를 계산합니다.
                    if len(local_pts) > 0:
                        # 마지막 로컬 점에서 전역 점으로 향하는 방향을 사용합니다.
                        dx = gx - local_pts[-1][0]
                        dy = gy - local_pts[-1][1]
                    else:
                        # 현재 위치에서 전역 점으로 향하는 방향을 사용합니다.
                        dx = gx - current_x
                        dy = gy - current_y
                    
                    if abs(dx) < 1e-6 and abs(dy) < 1e-6:
                        continue
                        
                    # 방향 벡터를 정규화합니다.
                    norm = math.hypot(dx, dy)
                    dx_norm = dx / norm
                    dy_norm = dy / norm
                    
                    # 수직 방향 오프셋 좌표를 계산합니다.
                    offset_x = gx - offset * dy_norm  # 수직 방향
                    offset_y = gy + offset * dx_norm
                    
                    # 오프셋 지점이 안전한지 확인합니다.
                    offset_x_b, offset_y_b = self._transform_point_base_from_map(offset_x, offset_y, tf_map_base)
                    
                    if not self._has_obstacle_in_direction(offset_x_b, offset_y_b, scan):
                        best_point = (offset_x, offset_y)
                        break
                
                if best_point:
                    local_pts.append(best_point)
                else:
                    # 안전한 경로가 없으면 여기서 계획을 중단합니다.
                    break
                    
            # 경로 길이를 제한합니다.
            if len(local_pts) >= int(self.local_horizon / self.ds):
                break
        
        return local_pts
    
    def _build_fallback_path(self, tf_map_base, scan: LaserScan) -> List[Tuple[float, float]]:
        """전역 경로를 사용할 수 없을 때 생성하는 대체 경로입니다."""
        local_pts = []  # base_link 프레임 좌표
        steps = max(2, int(self.local_horizon / self.ds))
        
        for k in range(steps):
            x = k * self.ds
            # 가장 여유가 있는 측면 오프셋을 선택합니다.
            best_off = None
            for off in self.lateral_offsets:
                if self._is_point_safe(x, off, scan):
                    best_off = off
                    break
            if best_off is None:
                # 안전한 오프셋이 없으면 전방 거리를 줄입니다.
                break
            local_pts.append((x, best_off))
        
        # 맵 프레임 좌표로 변환합니다.
        out = []
        for (xb, yb) in local_pts:
            xm, ym, _ = transform_point_map_from_base(xb, yb, tf_map_base)
            out.append((xm, ym))
        return out

    def _on_timer(self):
        if self.global_path is None or self.latest_scan is None:
            return
        tfmb = self._get_tf()
        if tfmb is None:
            return

        # 현재 자세와 가장 가까운 전역 경로 점을 찾아 로컬 경로 각도를 맞춥니다.
        cx = tfmb.transform.translation.x
        cy = tfmb.transform.translation.y
        idx = self._closest_path_index(cx, cy)
        if idx < 0:
            return

        # Path 메시지를 구성하고 퍼블리시합니다.
        pts_map = self._build_local_path(tfmb, self.latest_scan, cx, cy)
        if not pts_map:
            return
        path = Path()
        path.header.stamp = self.get_clock().now().to_msg()
        path.header.frame_id = self.map_frame
        # 첫 번째와 마지막 점을 이용해 자세의 요 각도를 추정합니다.
        yaw = 0.0
        if len(pts_map) >= 2:
            dx = pts_map[-1][0] - pts_map[0][0]
            dy = pts_map[-1][1] - pts_map[0][1]
            yaw = math.atan2(dy, dx)
        qz = math.sin(yaw/2.0)
        qw = math.cos(yaw/2.0)
        for (x, y) in pts_map:
            ps = PoseStamped()
            ps.header = path.header
            ps.pose.position.x = float(x)
            ps.pose.position.y = float(y)
            ps.pose.orientation.z = qz
            ps.pose.orientation.w = qw
            path.poses.append(ps)
        self.pub.publish(path)


def main(args=None):
    rclpy.init(args=args)
    node = LocalAvoidanceNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
