#!/usr/bin/env python3
"""
Graph Generator Node
====================
글로벌 경로를 구독하고 맵 정보를 사용하여 주행 가능한 그래프를 생성하는 노드
생성된 그래프는 npz 파일로 저장됨
"""

import os
import math
import hashlib
import shutil
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy
from nav_msgs.msg import Path, OccupancyGrid
from geometry_msgs.msg import PoseStamped


class GraphGenerator(Node):
    """글로벌 경로와 맵으로부터 주행 가능한 그래프를 생성하는 노드"""

    def __init__(self):
        super().__init__('graph_generator')

        # 파라미터 선언
        self.declare_parameter('global_path_topic', '/global_path')
        self.declare_parameter('map_topic', '/map')
        self.declare_parameter('output_graph_path', '')  # 런치 파일에서 설정
        self.declare_parameter('longitudinal_sampling_interval', 0.5)  # 글로벌 경로 샘플링 간격 (m)
        self.declare_parameter('lateral_sampling_interval', 0.5)  # 법선 방향 샘플링 간격 (m)
        self.declare_parameter('max_lateral_distance', 3.0)  # 법선 방향 최대 거리 (m)
        self.declare_parameter('min_wall_clearance', 0.2)  # 벽까지의 최소 거리 (m)

        # Adaptive sampling 파라미터
        self.declare_parameter('enable_adaptive_sampling', True)  # 적응형 샘플링 활성화
        self.declare_parameter('curvature_threshold_high', 0.3)  # 높은 곡률 임계값 (1/m)
        self.declare_parameter('curvature_threshold_low', 0.1)  # 낮은 곡률 임계값 (1/m)
        self.declare_parameter('dense_sampling_factor', 0.5)  # 곡선 구간 샘플링 밀도 증가 (곱셈)
        self.declare_parameter('sparse_sampling_factor', 1.5)  # 직선 구간 샘플링 밀도 감소 (곱셈)

        # Velocity-aware 파라미터
        self.declare_parameter('enable_velocity_estimation', True)  # 속도 추정 활성화
        self.declare_parameter('max_velocity_straight', 8.0)  # 직선 최대 속도 (m/s)
        self.declare_parameter('max_velocity_curve', 3.0)  # 곡선 최대 속도 (m/s)
        self.declare_parameter('max_velocity_lane_change', 5.0)  # 차선 변경 최대 속도 (m/s)

        # 파라미터 가져오기
        self.global_path_topic = self.get_parameter('global_path_topic').value
        self.map_topic = self.get_parameter('map_topic').value
        self.output_graph_path = self.get_parameter('output_graph_path').value
        self.long_interval = self.get_parameter('longitudinal_sampling_interval').value
        self.lat_interval = self.get_parameter('lateral_sampling_interval').value
        self.max_lat_distance = self.get_parameter('max_lateral_distance').value
        self.min_wall_clearance = self.get_parameter('min_wall_clearance').value

        # Adaptive sampling 파라미터 가져오기
        self.enable_adaptive = self.get_parameter('enable_adaptive_sampling').value
        self.curv_thresh_high = self.get_parameter('curvature_threshold_high').value
        self.curv_thresh_low = self.get_parameter('curvature_threshold_low').value
        self.dense_factor = self.get_parameter('dense_sampling_factor').value
        self.sparse_factor = self.get_parameter('sparse_sampling_factor').value

        # Velocity-aware 파라미터 가져오기
        self.enable_velocity = self.get_parameter('enable_velocity_estimation').value
        self.max_vel_straight = self.get_parameter('max_velocity_straight').value
        self.max_vel_curve = self.get_parameter('max_velocity_curve').value
        self.max_vel_lane_change = self.get_parameter('max_velocity_lane_change').value

        # 상태 변수
        self.global_path = None
        self.map_data = None
        self.map_resolution = None
        self.map_origin = None
        self.map_width = None
        self.map_height = None

        # QoS 설정: map_server는 TRANSIENT_LOCAL을 사용하므로 동일하게 설정
        map_qos = QoSProfile(
            depth=10,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            reliability=QoSReliabilityPolicy.RELIABLE
        )

        # 구독자
        self.path_sub = self.create_subscription(
            Path,
            self.global_path_topic,
            self.path_callback,
            10
        )
        self.map_sub = self.create_subscription(
            OccupancyGrid,
            self.map_topic,
            self.map_callback,
            map_qos
        )

        self.get_logger().info('그래프 생성 노드 초기화 완료')
        self.get_logger().info(f'종방향 샘플링 간격: {self.long_interval}m')
        self.get_logger().info(f'횡방향 샘플링 간격: {self.lat_interval}m')
        self.get_logger().info(f'최대 횡방향 거리: {self.max_lat_distance}m')
        self.get_logger().info(f'최소 벽 간격: {self.min_wall_clearance}m')

    def map_callback(self, msg):
        """맵 데이터 수신 콜백"""
        self.map_width = msg.info.width
        self.map_height = msg.info.height
        self.map_resolution = msg.info.resolution
        self.map_origin = (msg.info.origin.position.x, msg.info.origin.position.y)
        self.map_data = np.array(msg.data).reshape((self.map_height, self.map_width))

        self.get_logger().info(f'맵 데이터 수신 완료: {self.map_width}x{self.map_height}, 해상도={self.map_resolution}m')
        self.try_generate_graph()

    def path_callback(self, msg):
        """글로벌 경로 수신 콜백"""
        if len(msg.poses) == 0:
            self.get_logger().warn('빈 경로 수신됨')
            return

        self.global_path = msg
        self.get_logger().info(f'글로벌 경로 수신 완료: {len(msg.poses)}개 포즈')
        self.try_generate_graph()

    def try_generate_graph(self):
        """맵과 경로가 모두 준비되면 그래프 생성 시도"""
        if self.global_path is not None and self.map_data is not None:
            self.get_logger().info('맵과 경로가 준비됨. 그래프 생성 시작...')
            self.generate_graph()

    def world_to_map(self, x, y):
        """월드 좌표를 맵 그리드 좌표로 변환"""
        mx = int((x - self.map_origin[0]) / self.map_resolution)
        my = int((y - self.map_origin[1]) / self.map_resolution)
        return mx, my

    def is_position_free(self, x, y):
        """주어진 월드 좌표가 주행 가능한지 확인"""
        mx, my = self.world_to_map(x, y)

        # 맵 경계 확인
        if mx < 0 or mx >= self.map_width or my < 0 or my >= self.map_height:
            return False

        # occupancy grid 값 확인 (0: free, 100: occupied, -1: unknown)
        # 안전을 위해 50 이상이면 occupied로 간주
        return self.map_data[my, mx] >= 0 and self.map_data[my, mx] < 50

    def is_path_clear(self, x1, y1, x2, y2, check_interval=0.1):
        """두 점 사이의 경로가 주행 가능한지 확인 (선형 보간으로 체크)"""
        dx = x2 - x1
        dy = y2 - y1
        distance = math.sqrt(dx*dx + dy*dy)

        if distance < 1e-6:
            return self.is_position_free(x1, y1)

        # 체크할 포인트 수 계산 (check_interval 간격으로)
        num_checks = max(2, int(distance / check_interval) + 1)

        for i in range(num_checks):
            ratio = i / (num_checks - 1)
            check_x = x1 + dx * ratio
            check_y = y1 + dy * ratio

            if not self.is_position_free(check_x, check_y):
                return False

        return True

    def has_wall_clearance(self, x, y):
        """주어진 위치가 벽으로부터 충분한 거리를 유지하는지 확인"""
        if self.min_wall_clearance <= 0:
            return True  # 벽 간격 체크 비활성화

        # 체크할 방향 (8방향 + 추가 방향)
        num_directions = 16
        for i in range(num_directions):
            angle = 2.0 * math.pi * i / num_directions
            dx = math.cos(angle) * self.min_wall_clearance
            dy = math.sin(angle) * self.min_wall_clearance

            check_x = x + dx
            check_y = y + dy

            # 해당 방향으로 min_wall_clearance 거리만큼 떨어진 곳이 장애물이면 안 됨
            if not self.is_position_free(check_x, check_y):
                return False

        return True

    def calculate_path_length(self, poses):
        """경로의 총 길이 계산"""
        total_length = 0.0
        for i in range(len(poses) - 1):
            p1 = poses[i].pose.position
            p2 = poses[i + 1].pose.position
            dx = p2.x - p1.x
            dy = p2.y - p1.y
            total_length += math.sqrt(dx*dx + dy*dy)
        return total_length

    def sample_path_by_distance(self, poses, interval):
        """경로를 일정 간격으로 샘플링"""
        if len(poses) == 0:
            return []

        sampled = [poses[0]]  # 첫 포즈 추가
        total_dist = 0.0  # 경로 시작부터의 누적 거리

        for i in range(len(poses) - 1):
            p1 = poses[i].pose.position
            p2 = poses[i + 1].pose.position
            dx = p2.x - p1.x
            dy = p2.y - p1.y
            segment_length = math.sqrt(dx*dx + dy*dy)

            segment_start_dist = total_dist
            segment_end_dist = total_dist + segment_length

            # 이 세그먼트 내에서 샘플링해야 할 포인트들 찾기
            next_sample_dist = len(sampled) * interval

            while next_sample_dist < segment_end_dist:
                # 세그먼트 내에서의 비율 계산
                ratio = (next_sample_dist - segment_start_dist) / segment_length

                # 새 포즈 생성
                new_pose = PoseStamped()
                new_pose.header = poses[i].header
                new_pose.pose.position.x = p1.x + dx * ratio
                new_pose.pose.position.y = p1.y + dy * ratio

                # 방향은 세그먼트 방향 사용
                new_pose.pose.orientation = poses[i + 1].pose.orientation

                sampled.append(new_pose)
                next_sample_dist = len(sampled) * interval

            total_dist += segment_length

        return sampled

    def sample_path_adaptive(self, poses):
        """곡률 기반 적응형 샘플링 (간단한 구현)"""
        # 현재는 균일 샘플링 사용 (adaptive는 향후 개선 가능)
        # TODO: 실제 곡률 기반 adaptive sampling 구현
        return self.sample_path_by_distance(poses, self.long_interval)

    def quaternion_to_yaw(self, q):
        """쿼터니언을 yaw 각도로 변환"""
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    def compute_curvature(self, p0, p1, p2):
        """세 점으로 곡률 계산 (Menger curvature)"""
        # 삼각형 면적 계산
        x1, y1 = p0.pose.position.x, p0.pose.position.y
        x2, y2 = p1.pose.position.x, p1.pose.position.y
        x3, y3 = p2.pose.position.x, p2.pose.position.y

        area = 0.5 * abs((x2 - x1) * (y3 - y1) - (x3 - x1) * (y2 - y1))

        # 세 변의 길이
        a = math.sqrt((x2 - x1)**2 + (y2 - y1)**2)
        b = math.sqrt((x3 - x2)**2 + (y3 - y2)**2)
        c = math.sqrt((x3 - x1)**2 + (y3 - y1)**2)

        if a < 1e-6 or b < 1e-6 or c < 1e-6:
            return 0.0

        # Menger curvature: K = 4 * Area / (a * b * c)
        curvature = 4.0 * area / (a * b * c + 1e-9)
        return curvature

    def compute_adaptive_interval(self, curvature):
        """곡률에 따라 적응형 샘플링 간격 계산"""
        if not self.enable_adaptive:
            return self.long_interval

        if curvature >= self.curv_thresh_high:
            # 높은 곡률 (급커브) -> 촘촘하게
            return self.long_interval * self.dense_factor
        elif curvature <= self.curv_thresh_low:
            # 낮은 곡률 (직선) -> 성기게
            return self.long_interval * self.sparse_factor
        else:
            # 중간 곡률 -> 선형 보간
            ratio = (curvature - self.curv_thresh_low) / (self.curv_thresh_high - self.curv_thresh_low)
            factor = self.sparse_factor + (self.dense_factor - self.sparse_factor) * ratio
            return self.long_interval * factor

    def compute_max_velocity(self, curvature, lane_idx):
        """곡률과 레인 인덱스로 최대 허용 속도 계산"""
        if not self.enable_velocity:
            return self.max_vel_straight  # 기본값

        # 곡률 기반 속도 (중심선 기준)
        if curvature >= self.curv_thresh_high:
            base_velocity = self.max_vel_curve
        elif curvature <= self.curv_thresh_low:
            base_velocity = self.max_vel_straight
        else:
            # 선형 보간
            ratio = (curvature - self.curv_thresh_low) / (self.curv_thresh_high - self.curv_thresh_low)
            base_velocity = self.max_vel_straight + (self.max_vel_curve - self.max_vel_straight) * ratio

        # 레인 변경 페널티 (중심선이 아닌 경우)
        if lane_idx != 0:
            # 차선 변경은 더 낮은 속도
            base_velocity = min(base_velocity, self.max_vel_lane_change)

        return base_velocity

    def generate_graph(self):
        """그래프 생성 메인 로직"""
        self.get_logger().info('=' * 60)
        self.get_logger().info('그래프 생성 시작')

        # 1. 글로벌 경로 adaptive 샘플링
        if self.enable_adaptive:
            self.get_logger().info(f'1단계: 곡률 기반 적응형 샘플링 (기본 간격: {self.long_interval}m)')
            centerline_poses = self.sample_path_adaptive(self.global_path.poses)
        else:
            self.get_logger().info(f'1단계: 균일 샘플링 (간격: {self.long_interval}m)')
            centerline_poses = self.sample_path_by_distance(
                self.global_path.poses,
                self.long_interval
            )
        self.get_logger().info(f'   샘플링된 중심선 노드 수: {len(centerline_poses)}')

        # 곡률 정보 미리 계산 (velocity 추정용)
        curvatures = []
        for j in range(len(centerline_poses)):
            num_poses = len(centerline_poses)
            prev_idx = (j - 1) % num_poses
            next_idx = (j + 1) % num_poses
            curv = self.compute_curvature(
                centerline_poses[prev_idx],
                centerline_poses[j],
                centerline_poses[next_idx]
            )
            curvatures.append(curv)

        # 그래프 데이터 구조
        # nodes: list of dict with keys: 'x', 'y', 'theta', 'lane_idx', 'long_idx', 'max_velocity', 'curvature'
        # edges: list of tuples (from_idx, to_idx)
        nodes = []
        edges = []
        node_map = {}  # (lane_idx, long_idx) -> node_idx in nodes list

        # 2. 각 중심선 노드에서 법선 방향으로 샘플링
        self.get_logger().info('2단계: 법선 방향 샘플링 시작')
        self.get_logger().info('   레인 인덱스 규칙: i=0은 글로벌 패스, i>0은 왼쪽, i<0은 오른쪽')

        for j, pose in enumerate(centerline_poses):
            cx = pose.pose.position.x
            cy = pose.pose.position.y

            # 중심선 노드의 theta는 이전-현재-다음 노드를 고려하여 계산
            # 폐곡선이므로 첫/마지막 노드도 이전/다음 노드 사용
            num_poses = len(centerline_poses)
            prev_idx = (j - 1) % num_poses
            next_idx = (j + 1) % num_poses

            prev_pose = centerline_poses[prev_idx].pose.position
            next_pose = centerline_poses[next_idx].pose.position

            # 이전->다음 방향으로 theta 계산 (현재 노드를 중심으로)
            theta = math.atan2(next_pose.y - prev_pose.y, next_pose.x - prev_pose.x)

            # 법선 방향 계산 (왼쪽 방향)
            normal_x = -math.sin(theta)
            normal_y = math.cos(theta)

            # 각 종방향 위치에서 생성된 레인들을 저장
            # lane_idx는 글로벌 패스 기준으로 고정: i=0 (중심), i>0 (왼쪽), i<0 (오른쪽)
            lanes_at_j = {}  # lane_idx -> (x, y, theta)

            # 중심선 노드 (레인 i=0, 글로벌 패스)
            # 벽 간격 체크
            if self.has_wall_clearance(cx, cy):
                lanes_at_j[0] = {'x': cx, 'y': cy, 'theta': theta}

            # 왼쪽 방향 샘플링 (i=1, 2, 3, ...)
            current_dist = self.lat_interval
            lane_idx = 1
            prev_x, prev_y = cx, cy

            while current_dist <= self.max_lat_distance:
                sample_x = cx + normal_x * current_dist
                sample_y = cy + normal_y * current_dist

                # 경로 체크
                if not self.is_path_clear(prev_x, prev_y, sample_x, sample_y):
                    break

                # 벽 간격 체크
                if not self.has_wall_clearance(sample_x, sample_y):
                    break

                lanes_at_j[lane_idx] = {
                    'x': sample_x,
                    'y': sample_y,
                    'theta': theta
                }

                prev_x, prev_y = sample_x, sample_y
                current_dist += self.lat_interval
                lane_idx += 1

            # 오른쪽 방향 샘플링 (i=-1, -2, -3, ...)
            current_dist = self.lat_interval
            lane_idx = -1
            prev_x, prev_y = cx, cy

            while current_dist <= self.max_lat_distance:
                sample_x = cx - normal_x * current_dist
                sample_y = cy - normal_y * current_dist

                # 경로 체크
                if not self.is_path_clear(prev_x, prev_y, sample_x, sample_y):
                    break

                # 벽 간격 체크
                if not self.has_wall_clearance(sample_x, sample_y):
                    break

                lanes_at_j[lane_idx] = {
                    'x': sample_x,
                    'y': sample_y,
                    'theta': theta
                }

                prev_x, prev_y = sample_x, sample_y
                current_dist += self.lat_interval
                lane_idx -= 1

            # 생성된 레인들을 노드 리스트에 추가
            for lane_i, lane_data in lanes_at_j.items():
                node_idx = len(nodes)
                # 최대 속도 계산
                max_velocity = self.compute_max_velocity(curvatures[j], lane_i)
                nodes.append({
                    'x': lane_data['x'],
                    'y': lane_data['y'],
                    'theta': lane_data['theta'],
                    'lane_idx': lane_i,  # 글로벌 패스 기준 고정 인덱스
                    'long_idx': j,
                    'max_velocity': max_velocity,
                    'curvature': curvatures[j]
                })
                node_map[(lane_i, j)] = node_idx

        self.get_logger().info(f'   총 생성된 노드 수: {len(nodes)}')

        # 3. 간선 연결
        self.get_logger().info('3단계: 간선 연결 시작')

        # 현재 경로의 총 longitudinal 인덱스 수
        num_long = len(centerline_poses)

        # 각 longitudinal index에서 사용 가능한 lane index 범위 구하기
        lane_ranges = {}  # long_idx -> (min_lane, max_lane)
        for (lane_idx, long_idx), _ in node_map.items():
            if long_idx not in lane_ranges:
                lane_ranges[long_idx] = [lane_idx, lane_idx]
            else:
                lane_ranges[long_idx][0] = min(lane_ranges[long_idx][0], lane_idx)
                lane_ranges[long_idx][1] = max(lane_ranges[long_idx][1], lane_idx)

        edge_count = 0
        for (lane_i, long_j), from_idx in node_map.items():
            # 다음 longitudinal index로 연결 (폐곡선이므로 마지막은 처음으로)
            next_long = (long_j + 1) % num_long

            # 다음 위치에서 사용 가능한 레인 범위
            if next_long not in lane_ranges:
                continue

            min_lane, max_lane = lane_ranges[next_long]
            curr_min_lane, curr_max_lane = lane_ranges[long_j]

            # 현재 노드의 좌표
            from_node = nodes[from_idx]
            from_x, from_y = from_node['x'], from_node['y']

            # 연결 가능한 후보 노드들 (최대 3개: 직진, 좌, 우)
            candidates = []

            # Case 1: [i,j] -> [i,j+1] (직진 - 같은 레인 인덱스)
            if (lane_i, next_long) in node_map:
                candidates.append((lane_i, next_long))

            # Case 2: [i,j] -> [i-1,j+1] (오른쪽으로 차선 변경)
            if (lane_i - 1, next_long) in node_map:
                candidates.append((lane_i - 1, next_long))

            # Case 3: [i,j] -> [i+1,j+1] (왼쪽으로 차선 변경)
            if (lane_i + 1, next_long) in node_map:
                candidates.append((lane_i + 1, next_long))

            # 후보가 없으면 가장 가까운 레인 1개만 찾기 (코너에서 레인 수가 줄어드는 경우)
            if not candidates:
                # 현재 레인과 가장 가까운 레인 찾기
                closest_lane = None
                min_lane_diff = float('inf')

                for next_lane_i in range(min_lane, max_lane + 1):
                    if (next_lane_i, next_long) in node_map:
                        lane_diff = abs(next_lane_i - lane_i)
                        if lane_diff < min_lane_diff:
                            min_lane_diff = lane_diff
                            closest_lane = next_lane_i

                if closest_lane is not None:
                    candidates.append((closest_lane, next_long))

            # 모든 후보에 대해 간선 연결
            for next_lane_i, next_long_idx in candidates:
                to_idx = node_map[(next_lane_i, next_long_idx)]
                to_node = nodes[to_idx]
                to_x, to_y = to_node['x'], to_node['y']

                # 간선 경로가 주행 가능한지 확인
                if not self.is_path_clear(from_x, from_y, to_x, to_y):
                    continue  # 경로에 장애물이 있으면 간선 생성 안 함

                # 중복 방지
                if (from_idx, to_idx) not in edges:
                    edges.append((from_idx, to_idx))
                    edge_count += 1

        self.get_logger().info(f'   총 생성된 간선 수: {edge_count} (폐곡선 연결 포함, 경로 체크 후)')

        # 4. 그래프 데이터 저장
        self.save_graph(nodes, edges)

        self.get_logger().info('그래프 생성 완료!')
        self.get_logger().info('=' * 60)

        # 그래프 생성 후 노드 종료
        self.get_logger().info('그래프 생성이 완료되어 노드를 종료합니다.')
        rclpy.shutdown()

    def save_graph(self, nodes, edges):
        """그래프를 npz 파일로 저장"""
        self.get_logger().info('4단계: 그래프 데이터 저장')

        # 노드 데이터를 numpy 배열로 변환
        # [x, y, theta, lane_idx, long_idx, max_velocity, curvature]
        node_data = np.array([
            [n['x'], n['y'], n['theta'], n['lane_idx'], n['long_idx'], n['max_velocity'], n['curvature']]
            for n in nodes
        ], dtype=np.float64)

        # 간선 데이터를 numpy 배열로 변환
        if edges:
            edge_data = np.array(edges, dtype=np.int64)
        else:
            edge_data = np.empty((0, 2), dtype=np.int64)

        metadata = np.array([
            self.long_interval,
            self.lat_interval,
            self.max_lat_distance,
            self.min_wall_clearance
        ], dtype=np.float64)

        # 그래프 해시 계산
        graph_hash = self.compute_graph_hash(node_data, edge_data, metadata)
        hashed_path = self.get_hashed_output_path(graph_hash)

        # 해시 기반 경로가 이미 존재하는지 확인
        if os.path.exists(hashed_path):
            self.get_logger().info(f'   동일한 그래프가 이미 존재함: {hashed_path}')
            self.get_logger().info(f'   그래프 해시: {graph_hash}')

            # 요청된 경로가 있으면 심볼릭 링크 또는 복사 생성
            target_path = self.output_graph_path
            if target_path and os.path.abspath(target_path) != os.path.abspath(hashed_path):
                target_dir = os.path.dirname(target_path)
                if target_dir and not os.path.exists(target_dir):
                    os.makedirs(target_dir, exist_ok=True)

                # 기존 파일/링크 제거
                if os.path.exists(target_path) or os.path.islink(target_path):
                    os.remove(target_path)

                # 심볼릭 링크 생성 시도 (실패 시 복사)
                try:
                    os.symlink(os.path.abspath(hashed_path), target_path)
                    self.get_logger().info(f'   심볼릭 링크 생성: {target_path} -> {hashed_path}')
                except OSError:
                    shutil.copy2(hashed_path, target_path)
                    self.get_logger().info(f'   파일 복사: {target_path}')
        else:
            # 새로운 그래프 저장
            hashed_dir = os.path.dirname(hashed_path)
            if hashed_dir and not os.path.exists(hashed_dir):
                os.makedirs(hashed_dir)
                self.get_logger().info(f'   출력 디렉토리 생성: {hashed_dir}')

            np.savez(
                hashed_path,
                nodes=node_data,
                edges=edge_data,
                metadata=metadata,
                graph_hash=np.array([graph_hash], dtype='S64')
            )

            self.get_logger().info(f'   그래프 해시: {graph_hash}')
            self.get_logger().info(f'   해시 기반 그래프 저장: {hashed_path}')

            # 요청된 경로가 있으면 심볼릭 링크 또는 복사 생성
            target_path = self.output_graph_path
            if target_path and os.path.abspath(target_path) != os.path.abspath(hashed_path):
                target_dir = os.path.dirname(target_path)
                if target_dir and not os.path.exists(target_dir):
                    os.makedirs(target_dir, exist_ok=True)

                # 기존 파일/링크 제거
                if os.path.exists(target_path) or os.path.islink(target_path):
                    os.remove(target_path)

                # 심볼릭 링크 생성 시도 (실패 시 복사)
                try:
                    os.symlink(os.path.abspath(hashed_path), target_path)
                    self.get_logger().info(f'   심볼릭 링크 생성: {target_path} -> {hashed_path}')
                except OSError:
                    shutil.copy2(hashed_path, target_path)
                    self.get_logger().info(f'   파일 복사: {target_path}')

        self.get_logger().info(f'   - 노드 수: {len(nodes)}')
        self.get_logger().info(f'   - 간선 수: {len(edges)}')
        self.get_logger().info(f'   - 노드 형식: [x, y, theta, lane_idx, long_idx, max_velocity, curvature]')
        if self.enable_velocity and len(nodes) > 0:
            velocities = [n['max_velocity'] for n in nodes]
            self.get_logger().info(f'   - 속도 범위: {min(velocities):.2f} ~ {max(velocities):.2f} m/s')

    def compute_graph_hash(self, node_data, edge_data, metadata):
        """노드/간선/메타데이터를 기반으로 SHA-256 해시 생성"""
        hasher = hashlib.sha256()
        hasher.update(node_data.tobytes())
        hasher.update(edge_data.tobytes())
        hasher.update(metadata.tobytes())
        hasher.update(str(len(node_data)).encode('utf-8'))
        hasher.update(str(len(edge_data)).encode('utf-8'))
        return hasher.hexdigest()

    def get_hashed_output_path(self, graph_hash):
        """요청된 저장 경로를 바탕으로 해시 기반 파일 경로 생성"""
        base_path = self.output_graph_path
        if not base_path:
            base_path = os.path.join(os.getcwd(), 'graph.npz')

        base_dir = os.path.dirname(base_path)
        base_name = os.path.basename(base_path) or 'graph.npz'
        name, ext = os.path.splitext(base_name)
        if not ext:
            ext = '.npz'

        hashed_filename = f'{name}_{graph_hash[:8]}{ext}'
        return os.path.join(base_dir or '.', hashed_filename)


def main(args=None):
    """메인 함수"""
    rclpy.init(args=args)
    node = GraphGenerator()

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
